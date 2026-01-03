import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
import numpy as np
import cv2

from simulator_messages.msg import NavigationCommand  # 自定义消息

class VLNConnector(Node):
    def __init__(self):
        super().__init__('vln_connector')

        # RGB / Depth 订阅
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

        self.transform_sub = self.create_subscription(
            TransformStamped,
            '/simulator_msg/robot_transform',
            self.robot_transform_callback,
            10
        )

        self.command_pub = self.create_publisher(
            NavigationCommand,
            '/simulator_msg/navigation_command',
            10
        )

        self.rgb_image = None
        self.depth_image = None

        self.get_logger().info("VLN Connector Node started")

    def rgb_callback(self, msg: Image):
        h, w = msg.height, msg.width
        rgb = np.frombuffer(msg.data, dtype=np.uint8)

        if rgb.size == h * w * 4:  # RGBA
            rgb = rgb.reshape((h, w, 4))
            rgb = cv2.cvtColor(rgb, cv2.COLOR_RGBA2BGR)
        elif rgb.size == h * w * 3:  # RGB
            rgb = rgb.reshape((h, w, 3))
            rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        else:
            self.get_logger().error(f"Unexpected RGB data size: {rgb.size}")
            return

        self.rgb_image = rgb
        cv2.imshow("RGB", rgb)
        cv2.waitKey(1)

    def depth_callback(self, msg: Image):
        h, w = msg.height, msg.width
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape((h, w))
        self.depth_image = depth

        depth_vis = np.nan_to_num(depth)
        depth_vis = np.clip(depth_vis, 0.0, 10.0)
        depth_vis = (depth_vis / 10.0 * 255).astype(np.uint8)
        cv2.imshow("Depth", depth_vis)
        cv2.waitKey(1)

    def robot_transform_callback(self, msg: TransformStamped):
        t = msg.transform.translation
        r = msg.transform.rotation

        self.robot_position = [t.x, t.y, t.z]
        self.robot_rotation = [r.x, r.y, r.z, r.w]

        # self.get_logger().info(  # ← 这里从 debug 改成 info
        #     f"Received robot pose: pos=[{t.x:.3f}, {t.y:.3f}, {t.z:.3f}], "
        #     f"rot=[{r.x:.3f}, {r.y:.3f}, {r.z:.3f}, {r.w:.3f}]"
        # )

    def publish_navigation_command(self, nav_cmd):
        # msg = NavigationCommand()
        # msg.header.stamp = self.get_clock().now().to_msg()
        # msg.header.frame_id = "nav_cmd"

        # msg.local_position_offset = [0.0, 0.0, 1.0]
        # msg.local_rotation_offset = [0.0, 0.0, 0.0, 1.0]
        # msg.is_stop = False
        self.command_pub.publish(nav_cmd)
        self.get_logger().info("Published NavigationCommand")

def main(args=None):
    rclpy.init(args=args)
    node = VLNConnector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
