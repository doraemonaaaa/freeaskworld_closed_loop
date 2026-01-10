import rclpy
from rclpy.node import Node
import numpy as np
import math
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

class AgentCostMapNode(Node):
    def __init__(self):
        super().__init__('agent_cost_map_node')

        # 内部状态
        self.cost_map = None
        self.map_info = None
        self.base_pose = None        # (x, y, yaw)
        self.goal_yaw = 0.0
        self.latest_decision_map = None

        # =========================
        # ROS Publisher
        # =========================
        self.decision_map_pub = self.create_publisher(
            OccupancyGrid,
            "/agent/decision_costmap",
            10
        )

        # =========================
        # ROS Subscribers
        # =========================
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            "/rtabmap/map",
            self.map_callback,
            10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/agent/base_pose",
            self.pose_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            Float32,
            "/agent/goal_yaw",
            self.goal_callback,
            10
        )

    # =========================
    # Callbacks
    # =========================
    def map_callback(self, msg: OccupancyGrid):
        h = msg.info.height
        w = msg.info.width
        data = np.array(msg.data, dtype=np.int8).reshape(h, w)

        # cost_map: 0 free, 0.4 unknown, 1 obstacle
        cost = np.zeros((h, w), dtype=np.float32)
        cost[data == 100] = 1.0
        cost[data == -1] = 0.4
        cost[data == 0] = 0.0

        self.cost_map = cost
        self.map_info = msg.info

        # 收到 map 就立即更新并发布决策 costmap
        self.build_decision_map()
        self.publish_decision_costmap()

    def pose_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        q = msg.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self.base_pose = (x, y, yaw)

    def goal_callback(self, msg: Float32):
        self.goal_yaw = msg.data

    # =========================
    # Map transforms
    # =========================
    def world_to_map(self, x, y):
        origin = self.map_info.origin.position
        res = self.map_info.resolution
        mx = int((x - origin.x) / res)
        my = int((y - origin.y) / res)
        return mx, my

    # =========================
    # Value maps
    # =========================
    def trajectory_value_map(self):
        h, w = self.cost_map.shape
        mt = np.zeros((h, w), dtype=np.float32)
        if self.base_pose is None:
            return mt

        bx, by, yaw = self.base_pose
        mx, my = self.world_to_map(bx, by)

        for y in range(h):
            for x in range(w):
                dx = x - mx
                dy = y - my
                dist = math.hypot(dx, dy)
                mt[y, x] = min(dist / 50.0, 1.0)
        return mt

    def instruction_value_map(self):
        h, w = self.cost_map.shape
        mi = np.zeros((h, w), dtype=np.float32)
        if self.base_pose is None:
            return mi

        bx, by, yaw = self.base_pose
        mx, my = self.world_to_map(bx, by)

        for y in range(h):
            for x in range(w):
                dx = x - mx
                dy = my - y
                angle = math.atan2(dy, dx)
                diff = abs(self.angle_diff(angle, self.goal_yaw))
                mi[y, x] = diff / math.pi
        return mi

    def semantic_value_map(self):
        ms = np.zeros_like(self.cost_map)
        ms[self.cost_map == 0.4] = 0.2
        return ms

    # =========================
    # Build and publish
    # =========================
    def build_decision_map(self):
        if self.cost_map is None or self.base_pose is None:
            return None
        mt = self.trajectory_value_map()
        mi = self.instruction_value_map()
        ms = self.semantic_value_map()

        m = (
            1.0 * self.cost_map +
            0.6 * mt +
            0.8 * mi +
            0.4 * ms
        )
        self.latest_decision_map = np.clip(m, 0.0, 1.0)
        return self.latest_decision_map

    def publish_decision_costmap(self):
        if self.latest_decision_map is None or self.map_info is None:
            return

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info = self.map_info

        cost = (self.latest_decision_map * 100).clip(0, 100).astype(np.int8)
        msg.data = cost.flatten().tolist()

        self.decision_map_pub.publish(msg)

    @staticmethod
    def angle_diff(a, b):
        d = a - b
        while d > math.pi:
            d -= 2*math.pi
        while d < -math.pi:
            d += 2*math.pi
        return d


def main(args=None):
    rclpy.init(args=args)
    node = AgentCostMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
