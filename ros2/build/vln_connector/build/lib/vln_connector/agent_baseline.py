import os
import cv2
import time
import json
import rclpy
import numpy as np
import math
import re
import numpy as np
from pathlib import Path
from dotenv import load_dotenv
import tempfile
import threading

# 引入 AgentFlow 依赖
from agentflow.agents.solver_embodied import construct_solver_embodied

# 引入 ROS 消息
from simulator_messages.msg import SimulatorCommand  # 自定义消息
from .vln_connector import VLNConnector
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R

from .events import event_manager

class AgentBaseline(VLNConnector):
    def __init__(self):
        super().__init__()  # 初始化 ROS Node + RGBD Subscriber

        # 临时目录
        self._temp_dir = tempfile.TemporaryDirectory()
        self.get_logger().info(f"Temporary directory created: {self._temp_dir.name}")
        self._lock = threading.Lock()  # 推理锁
        self._inference_thread = None

        load_dotenv(dotenv_path="agentflow/.env")
        self.get_logger().info(
            f"OpenAI Key Loaded: {'OPENAI_API_KEY' in os.environ}"
        )

        self.llm_engine_name = "gpt-4o"

        self.solver = construct_solver_embodied(
            llm_engine_name=self.llm_engine_name,
            enabled_tools=[
                "Base_Generator_Tool",
                "GroundedSAM2_Tool"
            ],
            tool_engine=["gpt-4o"],
            model_engine=["gpt-4o", "gpt-4o", "gpt-4o"],
            output_types="direct",
            max_time=300,
            max_steps=1,
            enable_multimodal=True
        )

        # prompts
        self.system_prompt = (
            "# Task Complete Criterior: Reach target within 3 meters\n"
            "# You will receive two images: \n"
            "## 1. First-person view (RGB).\n"
            "## 2. A decision value map:\n"
            "- Warm colors (red/yellow): preferred regions\n"
            "- Cold colors (blue): avoid if possiblen\n"
            "- Black: obstacles\n"
        )
        self.task_prompt = None
        self.data_prompt = None

        self.temp_dir = Path("tmp/agent_baseline")
        self.temp_dir.mkdir(parents=True, exist_ok=True)

        self.step_counter = 0
        self.last_infer_step = -1  # 防止同一帧重复推理

        self.get_logger().info("AgentBaseline Initialized")

        self._stop_event = threading.Event()

        # map 
        self.raw_map_data = None
        self.raw_map_info = None
        self.raw_map_sub = self.create_subscription(
            OccupancyGrid,
            '/rtabmap/map',
            self.raw_map_callback,
            10
        )
        self.decision_map_data = None
        self.decision_map_sub = self.create_subscription(
            OccupancyGrid,
            '/agent/decision_costmap',
            self.decision_map_callback,
            10
        )
        self.latest_map_img = None

        # events
        event_manager.register("task_received", self.handle_task_received)

    # =====================================================
    # control logic（one step）
    # =====================================================
    def control_once_async(self):
        # 如果上一轮推理还在执行，不启动新推理
        if self._inference_thread is not None and self._inference_thread.is_alive():
            return

        # 通过 get_observation 获取最新观测
        obs = self.get_observation()
        if obs is None:
            return  # 数据不完整，直接返回

        rgb_snapshot = obs["rgb"].copy() if obs["rgb"] is not None else None
        depth_snapshot = obs["depth"].copy() if obs["depth"] is not None else None
        base_pose_snapshot = obs["base_pose"]  # 如果需要，也可以 deepcopy

        # 非阻塞调用 Inference
        self._inference_thread = threading.Thread(
            target=self.Inference,
            kwargs={
                "rgb": rgb_snapshot,
                "depth": depth_snapshot,
                "base_pose": base_pose_snapshot
            }
        )
        self._inference_thread.start()


    # =====================================================
    # Input Adapter
    # =====================================================
    def InputData(self, **kwargs):
        rgb_img = kwargs.get("rgb")
        
        # 保存第一视角 RGB
        rgb_path = self.temp_dir / f"rgb.jpg"
        cv2.imwrite(str(rgb_path), rgb_img)

        # 保存上帝视角 Map (如果有)
        img_paths = [str(rgb_path)]
        
        if self.latest_map_img is not None:
            map_path = self.temp_dir / f"map.jpg"
            cv2.imwrite(str(map_path), self.latest_map_img)
            img_paths.append(str(map_path)) # 此时列表里有两张图
            self.get_logger().info(f"Attached Map Image to LLM input")

        self.step_counter += 1
        return img_paths

    # =====================================================
    # Inference Adapter
    # =====================================================
    def Inference(self, **args):
        """
        通用 LLM 推理接口（可接收任意输入 via **args）
        线程安全，返回 SimulatorCommand
        """
        image_paths = args.get("image_paths", None)
        base_pose = args.get("base_pose", None)

        if image_paths is None:
            rgb = args.get("rgb")
            depth = args.get("depth")  # depth 可以留着以后用
            if rgb is None:
                self.get_logger().warn("No RGB input for inference, skipping", throttle_duration_sec=2.0)
                return None
            image_paths = self.InputData(rgb=rgb, depth=depth)

        if self.task_prompt is not None:
            try:
                propmt = self.system_prompt + self.task_prompt
                output = self.solver.solve(
                    propmt,
                    image_paths=image_paths
                )

                raw_text = output.get("direct_output", "")
                cmd = self._parse_llm_to_ros(raw_text)

                if cmd is not None:
                    self.publish_simulator_command(cmd)
                    
                self.get_logger().info(f"[LLM] Thinking... input={image_paths[-1]}")
                return cmd
            
            except Exception as e:
                self.get_logger().error(f"Inference Error: {e}")
                return None
        else:
            self.get_logger().warning(f"Waiting for task", throttle_duration_sec=2.0)
        
    def destroy_node(self):
        super().destroy_node()
        self._temp_dir.cleanup()
        self.get_logger().info("Temporary directory cleaned up.")

    def handle_task_received(self, task:str):
        if task is None:
            self.get_logger().warning(f"Task is empty")
            return
        print(f"🎯 任务来了")
        self.latest_task = task
        self.task_prompt = "[Updated Task]: " + task + "\n"

    def raw_map_callback(self, msg: OccupancyGrid):
        """保存原始地图数据"""
        h, w = msg.info.height, msg.info.width
        # 原始地图数据：-1(未知), 0(空闲), 100(障碍)
        self.raw_map_data = np.array(msg.data, dtype=np.int8).reshape(h, w)
        self.raw_map_info = msg.info
        
        # 尝试融合更新
        self._fuse_and_update_map_img()

    def decision_map_callback(self, msg: OccupancyGrid):
        """保存决策地图数据"""
        h, w = msg.info.height, msg.info.width
        # 决策地图数据：0-100 (Cost)
        self.decision_map_data = np.array(msg.data, dtype=np.int8).reshape(h, w)
        
        # 尝试融合更新
        self._fuse_and_update_map_img()

    def _fuse_and_update_map_img(self):
        """核心融合函数：将决策热力图与原始障碍物遮罩叠加"""
        if self.raw_map_data is None or self.decision_map_data is None:
            return

        # 检查尺寸是否匹配 (防止 RTAB-Map 动态扩图时导致的不一致)
        if self.raw_map_data.shape != self.decision_map_data.shape:
            # 如果尺寸不一致，通常以 raw_map 为准，等待 decision_map 更新
            return

        # --- Step 1: 制作底图 (Decision Heatmap) ---
        # 归一化 cost (0-100) -> (0.0-1.0)
        # Cost 越高(100) -> 越不推荐(Blue/Cold)
        # Cost 越低(0)   -> 越推荐(Red/Warm)
        # OpenCV Jet: 0=Blue, 255=Red. 所以我们需要反转 Cost。
        
        # 将 int8 转 float 防止溢出
        cost_float = self.decision_map_data.astype(np.float32)
        
        # 归一化并反转: cost 0 -> val 1.0 (Red), cost 100 -> val 0.0 (Blue)
        heatmap_val = 1.0 - (cost_float / 100.0)
        heatmap_val = np.clip(heatmap_val, 0.0, 1.0)
        
        # 转为 0-255 并应用色谱
        heatmap_gray = (heatmap_val * 255).astype(np.uint8)
        fused_img = cv2.applyColorMap(heatmap_gray, cv2.COLORMAP_JET)

        # --- Step 2: 制作遮罩 (Raw Map Overlays) ---
        
        # 掩码 A: 障碍物 (Raw Map == 100) -> 黑色
        obstacle_mask = (self.raw_map_data == 100)
        fused_img[obstacle_mask] = [0, 0, 0]  # BGR = Black

        # 掩码 B: 未知区域 (Raw Map == -1) -> 灰色
        unknown_mask = (self.raw_map_data == -1)
        fused_img[unknown_mask] = [128, 128, 128] # BGR = Grey

        # (可选) 机器人当前位置标记？
        # 通常不需要，因为 LLM 根据第一人称视角和 odom 坐标能推断，
        # 但如果在图上画个小箭头效果会更好。这里先保持纯地图。

        self.latest_map_img = fused_img


    def _parse_llm_to_ros(self, output_text: str):
        cmd = SimulatorCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "agent"
        
        method = ""
        method_params = ""

        # 提取 **Action** 标签后的文本
        action_match = re.search(
            r"(?:\*\*Action\*\*|Action|Navigation Goal)\s*:\s*(.*)",
            output_text,
            re.IGNORECASE | re.DOTALL
        )

        if action_match:
            action_text = action_match.group(1).strip()
            self.get_logger().info(f"Extracted Action Text: {action_text}")

            method_match = re.search(r"<(\w+)\((.*?)\)>", action_text, re.IGNORECASE | re.DOTALL)
            if method_match:
                method = method_match.group(1).strip()
                method_params = method_match.group(2).strip()
                self.get_logger().info(f"Parsed Method: {method}, Params: {method_params}")
            else:
                self.get_logger().warn("No <Method(...)> found in Action text.")
        else:
            self.get_logger().warn("Label 'Action:' not found in LLM output. No method extracted.")

        if method.lower() == "stop":
            self.get_logger().info("🏁 Stop received, exiting baseline for restart")
            self._stop_event.set()

        cmd.method = method
        cmd.method_params = method_params

        return cmd

# =====================================================
# Main Loop
# =====================================================
def main(args=None):
    rclpy.init(args=args)
    node = AgentBaseline()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)  # 持续刷新订阅数据
            node.control_once_async()               # 非阻塞推理

            if node._stop_event.is_set():
                node.get_logger().info("🔁 Baseline reseting...")
                break
    except KeyboardInterrupt:
        pass
    finally:
        if node._inference_thread is not None:
            node.get_logger().info("Waiting for inference thread to finish...")
            node._inference_thread.join(timeout=2.0)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
