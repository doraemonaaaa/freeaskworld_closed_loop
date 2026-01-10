import numpy as np
import math
import cv2
from nav_msgs.msg import OccupancyGrid

class AgentCostMap:
    def __init__(self):
        self.cost_map = None
        self.map_info = None
        self.base_pose = None  # (x, y, yaw)
        self.latest_decision_map = None
        self.decision_map_pub = self.create_publisher(
            OccupancyGrid,
            "/agent/decision_costmap",
            10
        )

    # =========================
    # Update inputs
    # =========================
    def update_map(self, occ_grid):
        h, w = occ_grid.shape
        cost = np.zeros((h, w), dtype=np.float32)

        # m_a: Action feasibility
        cost[occ_grid == 100] = 1.0     # obstacle
        cost[occ_grid == -1] = 0.4      # unknown
        cost[occ_grid == 0] = 0.0       # free

        self.cost_map = cost

    def update_map_info(self, map_info):
        self.map_info = map_info

    def update_base_pose(self, x, y, yaw):
        self.base_pose = (x, y, yaw)

    # =========================
    # Coordinate transforms
    # =========================
    def world_to_map(self, x, y):
        origin = self.map_info.origin.position
        res = self.map_info.resolution
        mx = int((x - origin.x) / res)
        my = int((y - origin.y) / res)
        return mx, my

    # =========================
    # m_t: Trajectory value
    # =========================
    def trajectory_value_map(self):
        """
        Penalize sharp turns & long jumps
        """
        h, w = self.cost_map.shape
        mt = np.zeros((h, w), dtype=np.float32)

        bx, by, yaw = self.base_pose
        mx, my = self.world_to_map(bx, by)

        for y in range(h):
            for x in range(w):
                dx = x - mx
                dy = y - my
                dist = math.hypot(dx, dy)

                # 距离惩罚（鼓励近点）
                mt[y, x] = min(dist / 50.0, 1.0)

        return mt

    # =========================
    # m_i: Instruction prior
    # =========================
    def instruction_value_map(self, goal_direction_yaw):
        """
        给一个方向先验（来自 LLM）
        """
        h, w = self.cost_map.shape
        mi = np.zeros((h, w), dtype=np.float32)

        bx, by, yaw = self.base_pose
        mx, my = self.world_to_map(bx, by)

        for y in range(h):
            for x in range(w):
                dx = x - mx
                dy = my - y  # 注意图像坐标
                angle = math.atan2(dy, dx)
                diff = abs(self.angle_diff(angle, goal_direction_yaw))
                mi[y, x] = diff / math.pi  # 越偏离方向 cost 越高

        return mi

    # =========================
    # m_s: Semantic / Unknown
    # =========================
    def semantic_value_map(self):
        ms = np.zeros_like(self.cost_map)
        ms[self.cost_map == 0.4] = 0.2  # unknown 稍微惩罚
        return ms

    # =========================
    # Final map
    # =========================
    def build_decision_map(self, goal_yaw):
        mt = self.trajectory_value_map()
        mi = self.instruction_value_map(goal_yaw)
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
        if self.map_info is None:
            return

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        # 直接复用原始 map 的 info（非常重要）
        msg.info = self.cost_builder.map_info

        # 映射到 0~100
        cost = (self.latest_decision_map * 100.0).clip(0, 100).astype(np.int8)

        # ROS 要求 row-major flat list
        msg.data = cost.flatten().tolist()

        self.decision_map_pub.publish(msg)

    @staticmethod
    def angle_diff(a, b):
        d = a - b
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d
