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
from scipy.spatial.transform import Rotation as R
import shutil

# 引入 AgentFlow 依赖
from agentflow.agents.solver_embodied import construct_solver_embodied

# 引入 ROS 消息
from simulator_messages.msg import SimulatorCommand  # 自定义消息
from .vln_connector import VLNConnector
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32
from std_msgs.msg import String

from .events import event_manager
from .agent_costmap import AgentCostMapNode

class AgentBaseline(VLNConnector):
    def __init__(self):
        super().__init__()  # 初始化 ROS Node + RGBD Subscriber

        # 临时目录
        self._temp_dir = tempfile.TemporaryDirectory()
        self.get_logger().info(f"Temporary directory created: {self._temp_dir.name}")
        self._lock = threading.Lock()  # 推理锁
        self._inference_thread = None

        # --- 清理上次日志 ---
        self.log_path = Path("tmp/llm_raw_text.log")
        if self.log_path.exists():
            self.log_path.unlink()  # 删除文件
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f"LLM log file cleared: {self.log_path}")

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
'''
# Task Context
## Task Skills
- Understand specific task instruction, and by that information to find the target.
- There are many same store in the scene, only choose the right store described by the instruction, if you want more information, just ask way from people.
- Navigate in the physical environment to reach the target within 3 meters then stop.
- Avoid obstacles unless you would get trapped.
## Observations You Will Receive
1. Ego-centric RGB image.
2. Ego-centric Depth image (Color-coded). 
- Warm colors (yellow/red) are far.
- Cool colors (blue/purple) are close.
3. Decision Value Map:
The Decision Value Map is a cost-based navigation field that encodes obstacles, history, and goal intention into a single spatial representation.
    - Ego Icon: Circle with Arrow showing current position & heading.
    - Cold Colors (Blue/Dark): Low cost path. 
        -- Goal direction attractor: Regions aligned with the current goal yaw have lower cost.
        -- Unvisited free space: Open areas that have not been explored yet are preferred.
    - Warm Colors (Red/Yellow): High cost.
        -- History trajectory field: Areas close to where the robot has already been are assigned high cost.
        -- Directional memory: A fan-shaped penalty extends forward from past motion, discouraging the robot from moving in the same direction again (anti-loop behavior).
        -- Uncertain space: Unknown map regions have moderate cost.
    - Black Color: Obstacles. Occupied cells from the SLAM map are treated as maximum cost and are strictly forbidden.
    - Dots: Small dots mark the robot’s previous positions, providing an explicit trace of where it has been.
> Use this information to continuously adjust your navigation strategy.
by these information, adjust your strategy to navigate. 
## Task Instruction Principle
- Give step-by-step directions using landmarks and relative movements (forward, left, right).
- Be concise, clear, and follow safe paths (prefer warm, avoid cold/black regions).
# End of Task Context
'''
        )

        self.task_prompt = None
        self.data_prompt = None

        self.temp_dir = Path("tmp/agent_baseline")
        if self.temp_dir.exists():
            shutil.rmtree(self.temp_dir)
        self.temp_dir.mkdir(parents=True, exist_ok=True)

        self.step_counter = 0
        self.last_infer_step = -1  # 防止同一帧重复推理

        self.get_logger().info("AgentBaseline Initialized")

        # map 
        self.raw_map_data = None
        self.raw_map_info = None
        self.raw_map_sub = self.create_subscription(
            OccupancyGrid,
            '/rtabmap/map',
            self.raw_map_callback,
            10
        )

        self.decision_map_sub = self.create_subscription(
            OccupancyGrid,
            "/agent/decision_costmap",  # 对应你之前写的那个节点的发布话题
            self.decision_map_callback,
            10
        )
        
        self.latest_map_img = None
        self.decision_map_data = None

        self.llm_pub = self.create_publisher(String, "/agent/llm_output", 10)
        self.goal_yaw_pub = self.create_publisher(Float32, "/agent/goal_yaw", 10)

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
        depth_snapshot = obs["depth_vis"].copy() if obs["depth_vis"] is not None else None
        latest_pose_snapshot = obs["latest_pose"]  # 如果需要，也可以 deepcopy

        # 非阻塞调用 Inference
        self._inference_thread = threading.Thread(
            target=self.Inference,
            kwargs={
                "rgb": rgb_snapshot,
                "depth_vis": depth_snapshot,
                "latest_pose": latest_pose_snapshot
            }
        )
        self._inference_thread.start()

    def InputData(self, **kwargs):
        rgb = kwargs.get("rgb")
        depth_vis = kwargs.get("depth_vis")

        img_paths = []

        # RGB
        rgb_path = self.temp_dir / "rgb.jpg"
        cv2.imwrite(str(rgb_path), rgb)
        img_paths.append(str(rgb_path))

        # Depth → 8bit 可视化
        if depth_vis is not None:
            depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_TURBO)
            depth_path = self.temp_dir / "depth.jpg"
            cv2.imwrite(str(depth_path), depth_colored)
            img_paths.append(str(depth_path))
        else:
            self.get_logger().warn("No Depth input for inference, skipping", throttle_duration_sec=2.0)

        # Map
        if self.latest_map_img is not None:
            map_path = self.temp_dir / "map.jpg"
            cv2.imwrite(str(map_path), self.latest_map_img)
            img_paths.append(str(map_path))
        else:
            self.get_logger().warn("No Map input for inference, skipping", throttle_duration_sec=2.0)

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
        latest_pose = args.get("latest_pose", None)

        if image_paths is None:
            rgb = args.get("rgb")
            depth_vis = args.get("depth_vis")  # depth 可以留着以后用
            if rgb is None:
                self.get_logger().warn("No RGB input for inference, skipping", throttle_duration_sec=2.0)
                return None
            image_paths = self.InputData(rgb=rgb, depth_vis=depth_vis)

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
        self.task_prompt = f'''
# Specific task instruction
{task}
# End of Specific task instruction
'''

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
        if self.raw_map_data is None or self.decision_map_data is None:
            return
        if self.raw_map_data.shape != self.decision_map_data.shape:
            return

        # cost → heatmap (low cost = blue, high cost = red)
        cost_float = self.decision_map_data.astype(np.float32) / 100.0
        cost_float = np.clip(cost_float, 0.0, 1.0)

        heatmap_gray = (cost_float * 255).astype(np.uint8)
        fused_img = cv2.applyColorMap(heatmap_gray, cv2.COLORMAP_JET)

        # obstacles & unknowns
        obstacle_mask = (self.raw_map_data == 100)
        fused_img[obstacle_mask] = [0, 0, 0]

        unknown_mask = (self.raw_map_data == -1)
        fused_img[unknown_mask] = [128, 128, 128]

        # === 新增：垂直翻转，使地图顶部显示在图像顶部 ===
        fused_img = cv2.flip(fused_img, 0)  # 0 表示垂直翻转（上下翻转）

        self.latest_map_img = fused_img

    def _parse_llm_to_ros(self, output_text: str):
        log_path = Path("tmp/llm_raw_text.log")
        with open(log_path, "a", encoding="utf-8") as f:
            f.write(output_text + "\n" + "-"*80 + "\n")
            
        msg = String()
        msg.data = output_text
        self.llm_pub.publish(msg)

        cmd = SimulatorCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "agent"
        
        method = ""
        method_params = ""

        method, method_params = self.solver.parse_command(output_text)

        cmd.method = method
        cmd.method_params = method_params

        if method.lower() == "stop":
            self.get_logger().info("🏁 Stop received")
            self._stop_event.set()
        elif method.lower() == "move":
                try:
                    # 假设参数格式是 x, y, yaw
                    params = [p.strip() for p in method_params.split(',')]
                    if len(params) >= 3:
                        yaw_deg = float(params[2])
                        
                        # 发布到 /agent/goal_yaw
                        goal_msg = Float32()
                        goal_msg.data = yaw_deg
                        self.goal_yaw_pub.publish(goal_msg)
                        
                        self.get_logger().info(f"Published goal_yaw: {yaw_deg:.2f}")
                except Exception as e:
                    self.get_logger().error(f"Error parsing Move params for goal_yaw: {e}")

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
