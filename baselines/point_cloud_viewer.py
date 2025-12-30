import open3d as o3d
import asyncio
import numpy as np

class PointCloudViewer:
    def __init__(self, mapper, update_hz=10):
        self.mapper = mapper
        self.dt = 1.0 / update_hz
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name="Merged PointClouds")
        self.pcd = o3d.geometry.PointCloud()
        self._added = False
        self._task = None

    async def _viewer_loop(self):
        print(f"[RealTimePointCloudViewer] Started (~{1/self.dt:.1f} Hz)")
        try:
            while True:
                self.update_pointcloud()
                self.vis.update_geometry(self.pcd)
                self.vis.poll_events()
                self.vis.update_renderer()
                await asyncio.sleep(self.dt)
        except asyncio.CancelledError:
            print("[RealTimePointCloudViewer] Viewer loop cancelled")
        finally:
            self.vis.destroy_window()
            print("[RealTimePointCloudViewer] Viewer stopped")

    def update_pointcloud(self):
        """把 merge 后的 scene、navigable、object 点云合并"""
        scene = self.mapper.scene_pcd.to_legacy() if self.mapper.scene_pcd else None
        nav = self.mapper.navigable_pcd.to_legacy() if self.mapper.navigable_pcd else None
        obj = self.mapper.object_pcd.to_legacy() if self.mapper.object_pcd else None

        merged = o3d.geometry.PointCloud()
        if scene and len(scene.points) > 0:
            merged += scene.paint_uniform_color([1, 1, 1])  # 白色
        if nav and len(nav.points) > 0:
            merged += nav.paint_uniform_color([0, 1, 0])    # 绿色
        if obj and len(obj.points) > 0:
            merged += obj.paint_uniform_color([1, 0, 0])    # 红色

        self.pcd.points = merged.points
        self.pcd.colors = merged.colors

        if not self._added:
            self.vis.add_geometry(self.pcd)
            self._added = True

    def start(self):
        if self._task is None or self._task.done():
            self._task = asyncio.create_task(self._viewer_loop())

    def stop(self):
        if self._task and not self._task.done():
            self._task.cancel()
            self._task = None
