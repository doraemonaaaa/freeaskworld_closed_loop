import numpy as np
from typing import Optional, Tuple

class Mapper:
    def __init__(self, map_size: float = 50.0, resolution: float = 0.1):
        """
        Initialize the mapper.
        :param map_size: Size of the map in meters (square).
        :param resolution: Size of each grid cell in meters.
        """
        self.resolution = resolution
        self.map_size = map_size
        self.grid_size = int(map_size / resolution)
        # 0: unknown, 1: free, -1: occupied (or probability)
        # For simplicity, let's use a counter or probability log-odds
        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=np.float32)
        self.center_idx = self.grid_size // 2

    def update(self, depth: np.ndarray, 
               extrinsics_matrix: np.ndarray, 
               robot_pos: np.ndarray, 
               robot_rot: np.ndarray,
               intrinsics_matrix: Optional[np.ndarray] = None,
               fov: float = 90.0):
        """
        Update the map with a new depth frame.
        
        :param depth: Depth image (H, W) in meters.
        :param extrinsics_matrix: 4x4 matrix, Camera to Robot transform.
        :param robot_pos: (3,) Robot position in World (x, y, z).
        :param robot_rot: (4,) Robot rotation quaternion (x, y, z, w).
        :param intrinsics_matrix: Optional 3x3 camera intrinsic matrix. If None, computed from fov.
        :param fov: Field of view in degrees (used if intrinsics_matrix is None).
        """
        H, W = depth.shape
        
        # 1. Depth to Point Cloud (Camera Frame)
        if intrinsics_matrix is not None:
            fx = intrinsics_matrix[0, 0]
            fy = intrinsics_matrix[1, 1]
            cx = intrinsics_matrix[0, 2]
            cy = intrinsics_matrix[1, 2]
        else:
            # Intrinsic matrix assumption from FOV
            f = 0.5 * W / np.tan(np.radians(fov / 2))
            fx = fy = f
            cx, cy = W / 2, H / 2
        
        u, v = np.meshgrid(np.arange(W), np.arange(H))
        z = depth
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        
        # Points in Camera Frame: (N, 3)
        # Unity/Simulator coordinate system check needed.
        # Assuming standard Computer Vision: Z forward, X right, Y down?
        # Or Unity: Z forward, X right, Y up?
        # Usually depth is Z.
        
        # Let's assume standard CV for projection: Z is depth.
        points_cam = np.stack([x, y, z], axis=-1).reshape(-1, 3)
        
        # Filter invalid depth
        valid_mask = (z.reshape(-1) > 0.1) & (z.reshape(-1) < 10.0)
        points_cam = points_cam[valid_mask]
        
        if points_cam.shape[0] == 0:
            return

        # 2. Camera to Robot Frame
        # points_robot = (Extrinsics @ points_cam_hom).T
        points_cam_hom = np.hstack([points_cam, np.ones((points_cam.shape[0], 1))])
        points_robot = (extrinsics_matrix @ points_cam_hom.T).T[:, :3]
        
        # 3. Robot to World Frame
        # World = RobotPos + RobotRot * RobotFramePoint
        
        # Quaternion to Rotation Matrix
        rx, ry, rz, rw = robot_rot
        # Standard conversion
        R = np.array([
            [1 - 2*ry*ry - 2*rz*rz, 2*rx*ry - 2*rz*rw, 2*rx*rz + 2*ry*rw],
            [2*rx*ry + 2*rz*rw, 1 - 2*rx*rx - 2*rz*rz, 2*ry*rz - 2*rx*rw],
            [2*rx*rz - 2*ry*rw, 2*ry*rz + 2*rx*rw, 1 - 2*rx*rx - 2*ry*ry]
        ])
        
        points_world = (R @ points_robot.T).T + robot_pos
        
        # 4. Update Grid
        # Assuming World Y is up (Unity standard), map is X-Z plane.
        # Or if Z is up, map is X-Y.
        # Unity: Y is up. Map on X-Z.
        
        # Filter points based on height to determine obstacles
        # e.g., points between 0.1m and 2.0m height are obstacles.
        # Floor is likely at Y=0.
        
        obstacle_mask = (points_world[:, 1] > 0.1) & (points_world[:, 1] < 2.0)
        obs_points = points_world[obstacle_mask]
        
        # Convert to grid coordinates
        # Center of grid is (0,0) in world? Or robot start?
        # Let's assume world (0,0) is center of grid.
        
        grid_x = ((obs_points[:, 0] / self.resolution) + self.center_idx).astype(int)
        grid_y = ((obs_points[:, 2] / self.resolution) + self.center_idx).astype(int)
        
        # Boundary checks
        valid_grid = (grid_x >= 0) & (grid_x < self.grid_size) & \
                     (grid_y >= 0) & (grid_y < self.grid_size)
                     
        grid_x = grid_x[valid_grid]
        grid_y = grid_y[valid_grid]
        
        # Simple occupancy: increment
        # In a real system, we'd do ray tracing to clear free space.
        # Here just marking obstacles.
        self.grid[grid_y, grid_x] = 1.0 # Occupied

    def get_map(self):
        return self.grid
