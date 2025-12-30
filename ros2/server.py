import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2

class RGBDSubscriber(Node):
    def __init__(self):
        super().__init__('rgbd_subscriber')

        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.rgb_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )

        self.rgb_image = None
        self.depth_image = None

        self.get_logger().info('RGBD Subscriber started.')

    def rgb_callback(self, msg: Image):
        """
        encoding: rgb8 或 rgba8
        """
        h, w = msg.height, msg.width

        # 将 ROS 消息数据转为 numpy array
        rgb = np.frombuffer(msg.data, dtype=np.uint8)

        # 自动判断通道数
        if rgb.size == h * w * 4:  # RGBA
            rgb = rgb.reshape((h, w, 4))
            rgb = cv2.cvtColor(rgb, cv2.COLOR_RGBA2BGR)
        elif rgb.size == h * w * 3:  # RGB
            rgb = rgb.reshape((h, w, 3))
            rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        else:
            self.get_logger().error(f"Unexpected RGB data size: {rgb.size}, expected {h*w*3} or {h*w*4}")
            return

        self.rgb_image = rgb

        # 可视化
        cv2.imshow("RGB", rgb)
        cv2.waitKey(1)


    def depth_callback(self, msg: Image):
        """
        encoding: 32FC1
        """
        h, w = msg.height, msg.width

        depth = np.frombuffer(msg.data, dtype=np.float32)
        depth = depth.reshape((h, w))

        self.depth_image = depth

        # 可视化（归一化到 0-255）
        depth_vis = np.nan_to_num(depth)
        depth_vis = np.clip(depth_vis, 0.0, 10.0)
        depth_vis = (depth_vis / 10.0 * 255).astype(np.uint8)

        cv2.imshow("Depth", depth_vis)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = RGBDSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
