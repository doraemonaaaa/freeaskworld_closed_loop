import threading
import cv2
import rclpy
import numpy as np
# 引入 ROS 消息
from simulator_messages.msg import NavigationCommand  # 自定义消息
from .vln_connector import VLNConnector

from .events import event_manager

class SimpleBaseline(VLNConnector):
    """
    Each Step:
        ROS spin -> InputData -> Inference -> Publish
    """

    def __init__(self):
        super().__init__()  # 初始化 ROS Node + RGBD Subscriber
        self._lock = threading.Lock()  # 推理锁
        self._inference_thread = None

        # events
        event_manager.register("task_received", self.handle_task_received)

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

        
    def destroy_node(self):
        super().destroy_node()

    # =====================================================
    # How to Handle task information
    # =====================================================
    def handle_task_received(self, task:str):
        if task is None:
            self.get_logger().warning(f"Task is empty")
            return
# =====================================================
# Main Loop
# =====================================================
def main(args=None):
    rclpy.init(args=args)
    node = SimpleBaseline()
    
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
