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
from agentflow.agentflow.solver_embodied import construct_solver_embodied

# 引入 ROS 消息
from simulator_messages.msg import NavigationCommand  # 自定义消息
from .rgbd_connector import VLNConnector


class AgentBaseline(VLNConnector):
    """
    串行 LLM 控制 Agent（无 ROS Timer）
    每一轮：
        ROS spin -> InputData -> Inference -> Publish
    """

    def __init__(self):
        super().__init__()  # 初始化 ROS Node + RGBD Subscriber

        # 临时目录
        self._temp_dir = tempfile.TemporaryDirectory()
        self.get_logger().info(f"Temporary directory created: {self._temp_dir.name}")
        self._lock = threading.Lock()  # 推理锁
        self._inference_thread = None

        # -------------------------------------------------
        # 1. 环境 & LLM 配置
        # -------------------------------------------------
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

        # -------------------------------------------------
        # 2. Agent 状态
        # -------------------------------------------------
        self.task_prompt = (
            "Go to the <我和乔治商店>, task finish upon arrival within 2 meters."
        )

        self.temp_dir = Path("tmp/agent_baseline")
        self.temp_dir.mkdir(parents=True, exist_ok=True)

        self.step_counter = 0
        self.last_infer_step = -1  # 防止同一帧重复推理

        self.get_logger().info("AgentBaseline Initialized")

        self._stop_event = threading.Event()

    # =====================================================
    # 主控制逻辑（单步）
    # =====================================================
    def control_once_async(self):
        # 如果上一轮推理还在执行，不启动新推理
        if self._inference_thread is not None and self._inference_thread.is_alive():
            return

        # snapshot 最新 RGB/D
        rgb_snapshot = self.rgb_image.copy() if self.rgb_image is not None else None
        depth_snapshot = self.depth_image.copy() if self.depth_image is not None else None

        if rgb_snapshot is None:
            return

        # 非阻塞调用 Inference
        self._inference_thread = threading.Thread(
            target=self.Inference, kwargs={"rgb": rgb_snapshot, "depth": depth_snapshot}
        )
        self._inference_thread.start()

    # =====================================================
    # Input Adapter
    # =====================================================
    def InputData(self, **kwargs):
        """
        将 ROS 内存图像保存为文件，供 AgentFlow 使用
        """
        rgb_img = kwargs.get("rgb")

        file_name = f"step_{self.step_counter:04d}.jpg"
        file_path = self.temp_dir / file_name

        cv2.imwrite(str(file_path), rgb_img)
        self.step_counter += 1

        return [str(file_path)]

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
                self.get_logger().warn("No RGB input for inference, skipping")
                return None
            image_paths = self.InputData(rgb=rgb, depth=depth)

        self.get_logger().info(f"[LLM] Thinking... input={image_paths[-1]}")

        try:
            output = self.solver.solve(
                self.task_prompt,
                image_paths=image_paths
            )

            raw_text = output.get("direct_output", "")
            nav_cmd = self._parse_llm_to_ros(raw_text)

            if nav_cmd is not None:
                self.publish_navigation_command(nav_cmd)

            if nav_cmd.is_stop:
                self.get_logger().info("🏁 Stop received, exiting baseline for restart")
                self._stop_event.set()
                
            return nav_cmd
        
        except Exception as e:
            self.get_logger().error(f"Inference Error: {e}")
            return None
        
    def destroy_node(self):
        super().destroy_node()
        self._temp_dir.cleanup()
        self.get_logger().info("Temporary directory cleaned up.")

    def _parse_llm_to_ros(self, output_text: str):
        cmd = NavigationCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "agent"
        
        # Unity: Z=Forward, X=Right, Y=Up
        pos_offset = [0.0, 0.0, 0.0] 
        rot_offset = [0.0, 0.0, 0.0, 1.0] # Identity quaternion (x, y, z, w)
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

                # 坐标系映射逻辑:
                # Agent X (Forward) -> Sim Z (Unity Forward)
                # Agent Y (Right)   -> Sim X (Unity Right)
                pos_offset[2] = x  # Z
                pos_offset[0] = y  # X
                pos_offset[1] = 0.0 # Y (Up)
                
                # 旋转映射 (绕 Y 轴)
                yaw_rad = np.radians(yaw_deg)
                half_angle = yaw_rad / 2.0
                rot_offset[1] = np.sin(half_angle) # Y
                rot_offset[3] = np.cos(half_angle) # W

            # --- Case 2: <Rotate(yaw)> ---
            rotate_match = re.search(
                r"<Rotate\(\s*(-?\d+\.?\d*)\s*\)>", 
                action_text, 
                re.IGNORECASE
            )
            if rotate_match:
                yaw_deg = float(rotate_match.group(1))
                self.get_logger().info(f"Parsed Rotate: yaw={yaw_deg}")
                yaw_rad = np.radians(yaw_deg)
                half_angle = yaw_rad / 2.0
                rot_offset[1] = np.sin(half_angle)
                rot_offset[3] = np.cos(half_angle)

            # --- Case 3: <Stop> ---
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


# =====================================================
# Main Loop（无 Timer，串行）
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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
