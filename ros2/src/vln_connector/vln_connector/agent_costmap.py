import rclpy
from rclpy.node import Node
import numpy as np
import math
from collections import deque
import cv2
from nav_msgs.msg import OccupancyGrid, Odometry  # 1. 引入 Odometry
from std_msgs.msg import Float32

from .utils import world_to_map

# Test cmd: ros2 topic pub -1 /agent/goal_yaw std_msgs/msg/Float32 "{data: 0.0}"
class AgentCostMapNode(Node):
    def __init__(self):
        super().__init__('agent_cost_map_node')

        # =========================
        # 1. 内部状态缓存
        # =========================
        self.latest_map_msg = None      # 原始地图消息
        self.latest_pose = None         # (x, y, yaw)
        self.latest_goal_yaw = None     # 目标角度
        self.path_history = deque(maxlen=200)  # 存储最近的 200 个轨迹点

        # params
        self.traj_point_radius = 0.5  # meter
        self.traj_point_yaw_dif = 60  # degree
        self.roi_radius = 15.0  # region of interest for cost of history traj calculate, meter

        self.traj_cost_radius = 0.2  # meter
        self.traj_cost_fan_angle = 60  # degree
        self.traj_cost_fan_radius = 3  # meter

        self.ego_size = 0.8  # meter

        # 状态标志位
        self.map_received = False
        self.pose_received = False
        self.goal_received = False

        # 缓存计算结果
        self.map_info = None
        self.cost_map_arr = None        # 解析后的 numpy cost
        self.decision_map = None

        # =========================
        # 2. 通信接口
        # =========================
        self.decision_map_pub = self.create_publisher(
            OccupancyGrid,
            "/agent/decision_costmap",
            10
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            "/rtabmap/map",
            self.map_callback,
            10
        )

        # 2. 修改订阅：订阅 Odom 获取 Pose
        self.odom_sub = self.create_subscription(
            Odometry,
            "/simulator_msg/odom",
            self.odom_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            Float32,
            "/agent/goal_yaw",
            self.goal_callback,
            10
        )

        # 使用 Timer 解耦接收和计算
        self.process_timer = self.create_timer(0.1, self.update_decision_map_loop)

        self.get_logger().info("AgentCostMapNode Initialized. Waiting for data streams...")

    # =========================
    # Callbacks (仅负责接收数据)
    # =========================
    def map_callback(self, msg: OccupancyGrid):
        self.latest_map_msg = msg
        self.map_received = True

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        
        # 四元數轉 yaw (弧度 → 度)
        yaw_rad = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        yaw_deg = math.degrees(yaw_rad)

        # 預設：需要記錄這個點嗎？
        should_record = False

        if not self.path_history:
            should_record = True
        else:
            prev_x, prev_y, prev_yaw = self.path_history[-1]
            
            # 計算距離差
            dist_diff = math.hypot(x - prev_x, y - prev_y)
            # 計算角度差（考慮跨 ±180° 的情況）
            yaw_diff = abs(yaw_deg - prev_yaw)
            yaw_diff = min(yaw_diff, 360 - yaw_diff)   # 最短角度差
            
            # 只要「距離夠大」或「角度變化夠大」就記錄
            if dist_diff > self.traj_point_radius or \
            yaw_diff > self.traj_point_yaw_dif:
                should_record = True

        if should_record:
            self.path_history.append((x, y, yaw_deg))

        # 無論是否記錄到 path_history，都要更新 latest_pose
        self.latest_pose = (x, y, yaw_deg)
        self.pose_received = True

    def goal_callback(self, msg: Float32):
        goal_yaw = msg.data  # ROS 消息单位度

        if self.latest_pose is not None:
            _, _, robot_yaw = self.latest_pose
            relative_yaw = (robot_yaw + goal_yaw) % 360
        else:
            relative_yaw = None

        self.latest_goal_yaw = relative_yaw
        self.goal_received = True

    # =========================
    # 核心逻辑循环
    # =========================
    def update_decision_map_loop(self):
        """
        主处理循环：检查数据 -> 数据预处理 -> 计算 -> 发布
        """
        # 1. Debug & Check: 检查数据是否齐全
        if not self.check_system_ready():
            return

        # 2. 解析地图
        self.parse_raw_map()

        # 3. 构建决策图
        self.build_decision_map()

        # 4. 发布
        self.publish_decision_costmap()

    # =========================
    # Debug & Diagnostics
    # =========================
    def check_system_ready(self):
        missing_data = []
        if not self.map_received:
            missing_data.append("Map (/rtabmap/map)")
        
        # 4. 更新 Debug 文本
        if not self.pose_received:
            missing_data.append("Odom (/simulator_msg/odom)")
        
        # if not self.goal_received:
        #      missing_data.append("Goal (/agent/goal_yaw)")

        if missing_data:
            self.get_logger().warn(
                f"waiting for data: {', '.join(missing_data)}", 
                throttle_duration_sec=2.0
            )
            return False
        
        return True

    # =========================
    # Map Transforms & Logic
    # =========================
    def parse_raw_map(self):
        """将 ROS 消息转为 numpy 数组"""
        msg = self.latest_map_msg
        self.map_info = msg.info
        h = msg.info.height
        w = msg.info.width
        raw_data = np.array(msg.data, dtype=np.int8).reshape(h, w)
        # 转换为 Cost (0.0 - 1.0)
        cost = np.zeros((h, w), dtype=np.float32)
        cost[raw_data == 100] = 1.0   # obstacle
        cost[raw_data == -1]  = 0.4   # unknown
        cost[raw_data == 0]   = 0.0   # free
        
        self.cost_map_arr = cost

    def trajectory_value_map(self, h, w, mx, my):
        if not self.map_info or not self.path_history:
            return np.zeros((h, w), dtype=np.float32)

        res = self.map_info.resolution
        roi_radius = getattr(self, "roi_radius", 5.0)  # meters
        roi_pix = int(roi_radius / res)

        # ROI 边界
        x_min, x_max = max(mx - roi_pix, 0), min(mx + roi_pix, w-1)
        y_min, y_max = max(my - roi_pix, 0), min(my + roi_pix, h-1)
        roi_h = y_max - y_min
        roi_w = x_max - x_min

        # ===============================
        # 1️⃣ Trajectory distance cost
        # ===============================
        mask = np.ones((roi_h, roi_w), dtype=np.uint8)
        for hx, hy, _ in self.path_history:
            pmx, pmy = world_to_map(self.map_info, hx, hy)
            if x_min <= pmx < x_max and y_min <= pmy < y_max:
                mask[pmy - y_min, pmx - x_min] = 0
        dist_pix = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
        dist_m = dist_pix * res
        traj_cost_roi = 1.0 - np.clip(dist_m / self.traj_cost_radius, 0.0, 1.0)

        # ===============================
        # 2️⃣ Directional fan cost
        # ===============================
        fan_cost_roi = np.zeros((roi_h, roi_w), dtype=np.float32)
        Y, X = np.indices((roi_h, roi_w))
        fan_radius_pix = self.traj_cost_fan_radius / res
        half_angle = self.traj_cost_fan_angle / 2

        for hx, hy, hyaw in self.path_history:
            pmx, pmy = world_to_map(self.map_info, hx, hy)
            if not (x_min <= pmx < x_max and y_min <= pmy < y_max):
                continue
            dx = X + x_min - pmx
            dy = Y + y_min - pmy
            dist = np.sqrt(dx**2 + dy**2)
            mask = dist <= fan_radius_pix
            if not np.any(mask):
                continue
            angle = np.degrees(np.arctan2(dy, dx))
            angle_diff = (angle - hyaw + 180) % 360 - 180
            in_fan = (np.abs(angle_diff) <= half_angle) & mask
            dist_term = 1.0 - dist / fan_radius_pix
            angle_term = 1.0 - np.abs(angle_diff) / half_angle
            strength = np.clip(dist_term * angle_term * in_fan, 0.0, 1.0)
            fan_cost_roi += strength * 0.7

        fan_cost_roi = np.clip(fan_cost_roi, 0.0, 1.0)

        # ===============================
        # 3️⃣ Fuse
        # ===============================
        cost_roi = 1.0 * traj_cost_roi + 0.8 * fan_cost_roi
        cost_roi = np.clip(cost_roi, 0.0, 1.0)

        # 填回原地图大小
        cost_full = np.zeros((h, w), dtype=np.float32)
        cost_full[y_min:y_max, x_min:x_max] = cost_roi

        roi_mask = np.zeros((h, w), dtype=np.float32)
        roi_mask[y_min:y_max, x_min:x_max] = 1.0

        return cost_full, roi_mask

    def instruction_value_map(self, h, w, mx, my, target_yaw):
        y_indices, x_indices = np.indices((h, w))
        dx = x_indices - mx
        dy = y_indices - my 
        
        grid_angles = np.degrees(np.arctan2(dy, dx))
        diff = (grid_angles - target_yaw + 180) % 360 - 180
        
        alignment = 1.0 - (np.abs(diff) / 180.0)
        
        # 让对齐方向 → 0 cost，背向 → 1 cost
        mi = 1.0 - alignment
        return mi.astype(np.float32)

    def semantic_value_map(self):
        ms = np.zeros_like(self.cost_map_arr)
        ms[self.cost_map_arr == 0.4] = 0.2 
        return ms

    def draw_ego_on_map(self, cost_map, mx, my, yaw_deg):
        """
        在 cost_map 上绘制 ego 位置和朝向（增强版）
        """
        h, w = cost_map.shape
        
        # 增大半径（至少保证明显可见）
        r = int(self.ego_size / self.map_info.resolution)
        
        # 画一个低成本的深色圆（值 0.0），在蓝色背景中更突出
        cv2.circle(cost_map, (mx, my), r, 0.0, -1)
        
        # 再画一个高成本的白色细轮廓（突出边缘）
        cv2.circle(cost_map, (mx, my), r, 1.0, 2) 
        
        # 绘制更明显的朝向箭头
        length = max(10, int(1.0 / self.map_info.resolution))  # 箭头更长
        thickness = max(3, int(0.3 / self.map_info.resolution))  # 加粗
        angle_rad = math.radians(yaw_deg)
        x_end = int(mx + length * math.cos(angle_rad))
        y_end = int(my + length * math.sin(angle_rad))
        
        # 使用带箭头的线，颜色用极端值
        cv2.arrowedLine(cost_map, (mx, my), (x_end, y_end), 
                        color=0.0, thickness=thickness, tipLength=0.3)  # 深色箭头
        
        # 可选：再叠加一条白色细箭头增加对比
        cv2.arrowedLine(cost_map, (mx, my), (x_end, y_end), 
                        color=1.0, thickness=2, tipLength=0.3)
        
        return cost_map

    def draw_trajectory_on_map(self, cost_map):
        """
        在 cost_map 上绘制历史轨迹
        """
        if not self.path_history or not self.map_info:
            return cost_map
        
        for hx, hy, hyaw in self.path_history:
            pmx, pmy = world_to_map(self.map_info, hx, hy)
            r = max(1, int(0.15 / self.map_info.resolution))  # 半径小点
            cv2.circle(cost_map, (pmx, pmy), r, 1.0, -1)  # 轨迹点
        return cost_map


    def build_decision_map(self):
        if self.cost_map_arr is None or self.latest_pose is None:
            return

        h, w = self.cost_map_arr.shape
        bx, by, byaw = self.latest_pose
        target_yaw = self.latest_goal_yaw if self.latest_goal_yaw is not None else 0.0

        mx, my = world_to_map(self.map_info, bx, by)

        mt, mt_mask = self.trajectory_value_map(h, w, mx, my)
        mi = self.instruction_value_map(h, w, mx, my, target_yaw)
        ms = self.semantic_value_map()

        m = np.maximum(self.cost_map_arr, 0.0)
        m += 1.2 * mt * mt_mask         # 历史轨迹：强惩罚（红）
        m += 0.8 * mi                   # 目标方向：低 cost，不增加
        m += 0.4 * ms                   # 未知区域轻惩罚

        m = np.clip(m, 0.0, 1.0)

        # =====================
        # 绘制 ego 和轨迹
        # =====================
        m = self.draw_trajectory_on_map(m)
        m = self.draw_ego_on_map(m, mx, my, byaw)

        self.decision_map = m


    def publish_decision_costmap(self):
        if self.decision_map is None or self.map_info is None:
            return

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info = self.map_info

        cost_int = (self.decision_map * 100).clip(0, 100).astype(np.int8)
        msg.data = cost_int.flatten().tolist()

        self.decision_map_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AgentCostMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()