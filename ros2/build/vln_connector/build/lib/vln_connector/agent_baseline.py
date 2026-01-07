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
from simulator_messages.msg import NavigationCommand  # 自定义消息
from .vln_connector import VLNConnector
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from tf_transformations import quaternion_from_euler 

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

        self.system_prompt = (
            "You will receive two images: \n"
            "1. First-person view (RGB).\n"
            "2. Top-down 2D map (White=Free, Black=Obstacle, Gray=Unknown).\n"
            "Use the map to plan a global path and the RGB view to avoid local obstacles.\n"
            "Output standard navigation actions."
        )
        self.task_prompt = None

        self.temp_dir = Path("tmp/agent_baseline")
        self.temp_dir.mkdir(parents=True, exist_ok=True)

        self.step_counter = 0
        self.last_infer_step = -1  # 防止同一帧重复推理

        self.get_logger().info("AgentBaseline Initialized")

        self._stop_event = threading.Event()

        # map 
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',  # RTAB-Map 发布的栅格地图话题
            self.map_callback,
            10
        )
        self.latest_map_img = None

        # events
        event_manager.register("task_received", self.handle_task_received)

        self.camera_extrinsics = np.array([
            [1.0,  0.0, 0.0, 0.0],
            [0.0, -1.0, 0.0, 1.6],
            [0.0,  0.0, 1.0, 0.0],
            [0.0,  0.0, 0.0, 1.0]
        ])
        
        # Manual Camera Intrinsics (3x3 Matrix)
        # 640x480 image with specific focal length
        self.camera_intrinsics = np.array([
            [415.6922, 0, 320],
            [0, 415.6922, 240],
            [0, 0, 1]
        ])

    # =====================================================
    # control logic（one step）
    # =====================================================
    def control_once_async(self):
        # 如果上一轮推理还在执行，不启动新推理
        if self._inference_thread is not None and self._inference_thread.is_alive():
            return

        # snapshot 最新 RGB/D
        rgb_snapshot = self.rgb_image.copy() if self.rgb_image is not None else None
        depth_snapshot = self.depth_image.copy() if self.depth_image is not None else None

        # 非阻塞调用 Inference
        self._inference_thread = threading.Thread(
            target=self.Inference, kwargs={"rgb": rgb_snapshot, "depth": depth_snapshot}
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
        线程安全，返回 NavigationCommand
        """
        image_paths = args.get("image_paths", None)
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
                nav_cmd = self._parse_llm_to_ros(raw_text)

                if nav_cmd is not None:
                    self.publish_navigation_command(nav_cmd)

                if nav_cmd.is_stop:
                    self.get_logger().info("🏁 Stop received, exiting baseline for restart")
                    self._stop_event.set()
                    
                self.get_logger().info(f"[LLM] Thinking... input={image_paths[-1]}")
                return nav_cmd
            
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
        print(f"🎯 任务来了: {task}")
        self.latest_task = task
        if self.latest_task is None:  # <--- 这里判断 self.latest_task 是否为空
            self.task_prompt = "[Task]:\n" + self.latest_task # <--- 报错：字符串 + None
        else:
            self.task_prompt = "[Updated Task]:\n" + self.latest_task
            
    def map_callback(self, msg: OccupancyGrid):
        # 2. 将栅格地图转换为 OpenCV 图像
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape(height, width)

        # 栅格地图值: -1 (未知), 0 (空闲), 100 (占据)
        # 转换为图像: 127 (灰), 255 (白), 0 (黑)
        img = np.zeros((height, width), dtype=np.uint8)
        img.fill(127) # 默认灰色未知
        img[data == 0] = 255   # 白色可行区域
        img[data == 100] = 0   # 黑色障碍物

        # 因为地图通常很大且原点在中心，可能需要裁剪或缩放以适应 Token 限制
        # 这里做一个简单的翻转以符合图片直观视角
        img = cv2.flip(img, 0) 
        self.latest_map_img = img

    def _parse_llm_to_ros(self, output_text: str):
        cmd = NavigationCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "agent"
        
        pos_offset = [0.0, 0.0, 0.0] 
        rot_offset = [0.0, 0.0, 0.0, 1.0]
        is_stopped = False

        # 1. 提取 Action 后的内容
        # 匹配 **Action**: 或 Action: 或 Navigation Goal:
        action_match = re.search(
            r"(?:\*\*Action\*\*|Action|Navigation Goal)\s*:\s*(.*)",
            output_text,
            re.IGNORECASE | re.DOTALL
        )

        if action_match:
            # 获取标签后的所有文本并去除空白
            action_text = action_match.group(1).strip()
            self.get_logger().info(f"Extracted Action Text: {action_text}")

            # --- Case 1: <Move(x, y, yaw)> ---
            # 注意：这里正则匹配 float, 捕获 x, y, yaw
            move_match = re.search(
                r"<Move\(\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*\)>", 
                action_text, 
                re.IGNORECASE
            )
            if move_match:
                x = float(move_match.group(1))     # Agent Forward
                y = float(move_match.group(2))     # Agent Right
                yaw_deg = float(move_match.group(3)) # Rotation in degrees
                
                self.get_logger().info(f"Parsed Move: x={x}, y={y}, yaw={yaw_deg}")

                pose = self.agent_to_ros_pose(x, y, yaw_deg)
                pos_offset = (pose.position.x, pose.position.y, pose.position.z)
                rot_offset = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

            # --- Case 2: <Stop> ---
            if "<Stop>" in action_text or "Stop()" in action_text:
                is_stopped = True
                self.get_logger().info("Action: STOP")

        else:
            # 如果根本没找到 Action: 标签
            self.get_logger().warn("Label 'Action:' not found in LLM output. Stopping for safety.")
            is_stopped = True

        # 填充 ROS 消息字段
        cmd.local_position_offset = pos_offset
        cmd.local_rotation_offset = rot_offset
        cmd.is_stop = is_stopped

        return cmd
    
    def agent_to_ros_pose(self, x_agent, y_agent, yaw_deg_agent):
        """
        Convert agent local move (x_forward, y_right, yaw_deg) 
        to ROS Pose (x, y, z, quaternion).
        """

        # 坐标轴映射
        x_ros = x_agent
        y_ros = -y_agent
        z_ros = 0.0

        # 旋转映射
        yaw_rad = math.radians(yaw_deg_agent)
        roll = 0.0
        pitch = 0.0

        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw_rad)

        # 构造 ROS Pose 消息
        pose = Pose()
        pose.position.x = x_ros
        pose.position.y = y_ros
        pose.position.z = z_ros
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        return pose

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
