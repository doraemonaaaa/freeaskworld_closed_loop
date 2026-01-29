# import cv2
# import numpy as np
# import asyncio
# import math
# from .mapping_utils.projection import get_global_bounds, project_frontier_map, project_costmap_grid


# class Map2DViewer:
#     def __init__(self, mapper, update_hz=10, fov_deg=90.0):
#         self.mapper = mapper
#         self.dt = 1.0 / update_hz
#         self.fov_deg = fov_deg
#         self.fov_rad = math.radians(fov_deg)
#         self._viewer_task = None

#     async def _viewer_loop(self):
#         print(f"[Map2DViewer] Async viewer started (~{1/self.dt:.1f}Hz, FOV: {self.fov_deg}°)")
#         cv2.namedWindow("Real-time 2D Map", cv2.WINDOW_NORMAL)  # 可调整窗口大小

#         try:
#             while True:
#                 try:
#                     self.render()

#                     key = cv2.waitKey(1) & 0xFF
#                     if key == ord('q') or key == 27:
#                         print("[Map2DViewer] Quit requested")
#                         break

#                 except Exception as e:
#                     print(f"[Map2DViewer] Render error: {e}")

#                 await asyncio.sleep(self.dt)

#         finally:
#             cv2.destroyAllWindows()
#             print("[Map2DViewer] Viewer stopped")

#     def start(self):
#         if self._viewer_task is None or self._viewer_task.done():
#             self._viewer_task = asyncio.create_task(self._viewer_loop())
#             print("[Map2DViewer] start() called")

#     def stop(self):
#         if self._viewer_task is not None and not self._viewer_task.done():
#             self._viewer_task.cancel()
#             self._viewer_task = None
#             print("[Map2DViewer] stop() called")

#     def render(self):
#         """渲染当前地图（不再放大，直接原始分辨率显示）"""
#         if not hasattr(self.mapper, "navigable_pcd") or self.mapper.navigable_pcd.is_empty():
#             empty = np.zeros((400, 400, 3), dtype=np.uint8)
#             cv2.putText(empty, "Waiting for map data...", (50, 200),
#                         cv2.FONT_HERSHEY_SIMPLEX, 1.0, (128, 128, 128), 2)
#             cv2.imshow("Real-time 2D Map", empty)
#             return

#         vis_map = self.get_2d_map()
#         if vis_map is None:
#             return

#         # === 直接使用原始地图尺寸显示（不再 resize 放大）===
#         display_map = vis_map.copy()

#         # 在原始分辨率地图上绘制机器人位置和FOV
#         self.draw_robot_and_fov(display_map)

#         # 显示统计信息
#         nav_n = self.mapper.navigable_pcd.point.positions.shape[0]
#         obs_n = self.mapper.obstacle_pcd.point.positions.shape[0]
#         iter_n = getattr(self.mapper, 'update_iterations', 0)
#         info = f"Nav: {nav_n}  Obs: {obs_n}  Iter: {iter_n}"
#         cv2.putText(display_map, info, (10, 30),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

#         # 可选：添加网格坐标参考（每米一个刻度）
#         self.draw_grid_lines(display_map)

#         cv2.imshow("Real-time 2D Map", display_map)

#     def draw_grid_lines(self, img):
#         """绘制米级网格线，帮助判断距离"""
#         min_bound, max_bound = get_global_bounds([self.mapper.navigable_pcd, self.mapper.obstacle_pcd])
#         grid_res = self.mapper.grid_resolution
#         h, w = img.shape[:2]

#         # 每 1 米画一条浅灰线
#         for x in np.arange(min_bound[0], max_bound[0] + grid_res, 1.0):
#             col = int((x - min_bound[0]) / grid_res)
#             if 0 <= col < w:
#                 cv2.line(img, (col, 0), (col, h), (50, 50, 50), 1)

#         for z in np.arange(min_bound[2], max_bound[2] + grid_res, 1.0):
#             row = int((z - min_bound[2]) / grid_res)
#             if 0 <= row < h:
#                 cv2.line(img, (0, row), (w, row), (50, 50, 50), 1)

#     def draw_robot_and_fov(self, img):
#         """ego黄色方块 + FOV三角扇形 + 绿色历史轨迹"""
#         if not hasattr(self.mapper, 'current_position') or not hasattr(self.mapper, 'current_rotation'):
#             return

#         pos = self.mapper.current_position
#         rot = self.mapper.current_rotation
#         forward = rot[:, 1]
#         yaw = math.atan2(forward[0], forward[1])

#         min_bound, _ = get_global_bounds([self.mapper.navigable_pcd, self.mapper.obstacle_pcd])
#         grid_res = self.mapper.grid_resolution

#         grid_x = int((pos[0] - min_bound[0]) / grid_res)
#         grid_y = int((pos[1] - min_bound[1]) / grid_res)

#         grid_height = img.shape[0]
#         pixel_x = grid_x
#         pixel_y = grid_height - 1 - grid_y
#         robot_center = np.array([int(pixel_x), int(pixel_y)], dtype=np.int32)

#         if not (0 <= pixel_x < img.shape[1] and 0 <= pixel_y < img.shape[0]):
#             return

#         # === 黄色方块 ===
#         box_half = 8
#         tl = (robot_center[0] - box_half, robot_center[1] - box_half)
#         br = (robot_center[0] + box_half, robot_center[1] + box_half)
#         cv2.rectangle(img, tl, br, (0, 255, 255), thickness=-1)
#         cv2.rectangle(img, tl, br, (0, 0, 0), thickness=2)

#         # === FOV三角扇形 ===
#         fov_radius = self.fov_deg
#         half_fov = self.fov_rad / 2

#         left_angle = yaw - half_fov
#         right_angle = yaw + half_fov

#         pt_left = robot_center + np.array([
#             fov_radius * math.sin(left_angle),
#             -fov_radius * math.cos(left_angle)
#         ], dtype=np.int32)

#         pt_right = robot_center + np.array([
#             fov_radius * math.sin(right_angle),
#             -fov_radius * math.cos(right_angle)
#         ], dtype=np.int32)

#         triangle_pts = np.array([robot_center, pt_left, pt_right], dtype=np.int32)

#         overlay = img.copy()
#         cv2.fillPoly(overlay, [triangle_pts], color=(255, 140, 60))
#         cv2.addWeighted(overlay, 0.25, img, 0.75, 0, img)

#         cv2.line(img, tuple(robot_center), tuple(pt_left), (0, 140, 255), 3)
#         cv2.line(img, tuple(robot_center), tuple(pt_right), (0, 140, 255), 3)

#         if len(self.mapper.trajectory_position) > 1:
#             trajectory_pixels = []
#             # 相对坐标的原点在地图中心的假设（因为初始位置被减掉了）
#             # 我们把相对 (0,0) 映射到图像中心附近
#             offset_x = img.shape[1] // 2   # 假设起点在图像中心X
#             offset_y = img.shape[0] // 2   # 假设起点在图像中心Y
#             scale = 1.0 / grid_res         # 1米对应多少像素

#             for rel_pos in self.mapper.trajectory_position:
#                 # rel_pos 是 (x_rel, y_rel, z)
#                 px = int(offset_x + rel_pos[0] * scale)
#                 py = int(offset_y - rel_pos[1] * scale)  # Y向上取负
#                 if 0 <= px < img.shape[1] and 0 <= py < img.shape[0]:
#                     trajectory_pixels.append((px, py))

#             if len(trajectory_pixels) > 1:
#                 # 现在画绿色渐变轨迹
#                 for i in range(len(trajectory_pixels) - 1):
#                     alpha = i / (len(trajectory_pixels) - 1)
#                     color = (0, int(80 + 175 * alpha), int(50 + 150 * alpha))  # 深绿 → 亮绿
#                     cv2.line(img, trajectory_pixels[i], trajectory_pixels[i+1], color, 3)

#     def get_2d_map(self):
#         min_bound, max_bound = get_global_bounds([self.mapper.navigable_pcd, self.mapper.obstacle_pcd])
        
#         num_points = self.mapper.navigable_pcd.point.positions.shape[0]
#         if num_points == 0:
#             return np.zeros((200, 200, 3), dtype=np.uint8)

#         costmap_grid = project_costmap_grid(
#             self.mapper.navigable_pcd,
#             np.ones(num_points, dtype=np.float32),
#             min_bound, max_bound,
#             grid_resolution=self.mapper.grid_resolution
#         )

#         frontier_grid = project_frontier_map(
#             self.mapper.obstacle_pcd,
#             self.mapper.navigable_pcd,
#             min_bound, max_bound,
#             grid_resolution=self.mapper.grid_resolution
#         )

#         vis_map = np.zeros((*costmap_grid.shape, 3), dtype=np.uint8)
#         vis_map[costmap_grid > 0] = [255, 255, 255]      # 可通行：白色
#         vis_map[frontier_grid > 0] = [0, 0, 255]         # 前沿：红色

#         return vis_map