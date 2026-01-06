"""Agent baseline implementation using the solver from quick_start.py.

Works with WebRTC DataChannel for communication.
"""

from __future__ import annotations

import asyncio
import os
import re
import tempfile
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
import cv2

import numpy as np
import quaternion
from PIL import Image
from dotenv import load_dotenv

# from .map_viewer import Map2DViewer
import threading

# Adjust path to ensure agentflow can be imported if running from FreeAskAgent root
import sys
if str(Path(__file__).parents[2]) not in sys.path:
    sys.path.append(str(Path(__file__).parents[2]))

try:
    from agentflow.agents.solver_embodied import construct_solver_embodied
except ImportError:
    # Fallback or error handling if agentflow is not found
    print("Warning: agentflow not found. AgentBaseline will fail.")

from ..webrtc_connector.framework import BaselineResponse, BaselineSession, ClosedLoopBaseline, MessageEnvelope
from ..webrtc_connector.messages import NavigationCommand, Step, TransformData
from .mapper import Mapper
from .point_cloud_viewer import PointCloudViewer

class AgentBaseline(ClosedLoopBaseline):
    """Baseline that uses the AgentFlow solver to control the agent."""

    def __init__(self) -> None:
        self._initialized = False
        self._solver = None
        self._setup_solver()
        self._temp_dir = tempfile.TemporaryDirectory()
        self._inference_lock = asyncio.Lock()
        
        # Manual Camera Extrinsics (Camera to Robot)
        # Camera at (0, 1.6, 0) in Unity coords (X right, Y up, Z forward).
        # Mapper produces points in CV coords (X right, Y down, Z forward).
        # Transform: Flip Y, then translate by (0, 1.6, 0).
        self.camera_extrinsics = np.array([
            [1.0,  0.0, 0.0, 0.0],
            [0.0, -1.0, 0.0, 1.6],
            [0.0,  0.0, 1.0, 0.0],
            [0.0,  0.0, 0.0, 1.0]
        ])
        
        # Manual Camera Intrinsics (3x3 Matrix)
        # 640x480 image with specific focal length
        self.camera_intrinsics = np.array([
            [415.6922, 0, 320],
            [0, 415.6922, 240],
            [0, 0, 1]
        ])
        
        # Import Unity coordinate transformation functions
        from .mapping_utils.geometry import unity_translation, unity_rotation
        
        # Initialize Mapper with Unity coordinate system settings
        # Unity: X-right, Y-up, Z-forward
        self.mapper = Mapper(
            camera_intrinsic=self.camera_intrinsics,
            pcd_resolution=0.05,
            grid_resolution=0.1,
            grid_size=5,
            floor_height=-0.2,      # Adjusted for Unity's Y-up coordinate
            ceiling_height=2.5,     # Typical ceiling height in meters
            translation_func=unity_translation,
            rotation_func=unity_rotation,
            rotate_axis=[0, 1, 0],  # Y-axis rotation for Unity
            device='cuda:0',
            enable_object_detection=False  # Disable GLEE object detection for now
        )
        
        # Flag to track if mapper has been reset for current session
        self._mapper_initialized = False

        # self.map2d = Map2DViewer(
        #                 self.mapper,
        #                 update_hz=15,      # 根据你的处理速度调整，10~20 都可以
        #                 fov_deg=60.0       # 你的相机FOV（常见RGB相机约80-110，可实测调整）
        #             )

    def _setup_solver(self):
        # Load environment variables
        env_path = Path(__file__).parents[2] / "agentflow" / ".env"
        load_dotenv(dotenv_path=env_path)
        
        print("Proxy_API_BASE:" + os.environ.get("Proxy_API_BASE", "Not Set"))
        print("OPENAI_API_KEY:" + os.environ.get("OPENAI_API_KEY", "Not Set"))

        llm_engine_name = os.environ.get("LLM_ENGINE_NAME", "gpt-4o")

        self._solver = construct_solver_embodied(
            llm_engine_name=llm_engine_name,
            enabled_tools=["Base_Generator_Tool", "GroundedSAM2_Tool"],
            tool_engine=["gpt-4o"],
            model_engine=["gpt-4o", "gpt-4o", "gpt-4o"],
            output_types="direct",
            max_time=300,
            max_steps=1,
            enable_multimodal=True
        )

    async def on_session_start(self, session: BaselineSession) -> None:
        session.metadata.clear()
        session.metadata["frame_history"] = []
        # Reset mapper initialization flag for new session
        self._mapper_initialized = False

        # self.map2d.start()

    async def on_session_end(self, session: BaselineSession) -> None:
        
        # Cleanup temp files for this session
        if "frame_history" in session.metadata:
            for path_str in session.metadata["frame_history"]:
                try:
                    Path(path_str).unlink(missing_ok=True)
                except Exception:
                    pass
        
        session.state.clear()
        # Reset mapper state
        self._mapper_initialized = False
        # Cleanup temp dir if needed, but we keep it for the lifetime of the baseline instance
        # self.map2d.stop()

    async def handle_envelope(
        self, session: BaselineSession, envelope: MessageEnvelope
    ) -> BaselineResponse | None:
        if envelope.message_type == "json":
            self._handle_json_packet(session, envelope)
        elif envelope.message_type == "rgbd":
            session.metadata["has_rgbd"] = True
            session.state.update_rgbd(envelope.payload)

        if not self._ready_to_respond(session):
            return None

        # Only run one inference at a time; drop intermediate triggers
        if self._inference_lock.locked():
            return None

        async with self._inference_lock:
            return await self._run_inference(session)

    # async def _run_inference(self, session: BaselineSession) -> BaselineResponse | None:
    #     """Execute one inference pass using the latest state."""
    #     # Get the latest RGB and depth frames
    #     rgb_frame = self._get_rgb_frame(session.state.latest_rgbd)
    #     if rgb_frame is None:
    #         print("[AgentBaseline] WARNING: rgb_frame is None!")
    #         return None
        
    #     depth_frame = self._get_depth_frame(session.state.latest_rgbd)
        
    #     # Get robot pose from session metadata
    #     robot_pose = session.metadata.get("robot_pose")
        
    #     # Debug: Print what data we have
    #     print(f"[AgentBaseline] DEBUG: rgb_frame shape={rgb_frame.shape if rgb_frame is not None else None}")
    #     print(f"[AgentBaseline] DEBUG: depth_frame={depth_frame is not None}, shape={depth_frame.shape if depth_frame is not None else None}")
    #     print(f"[AgentBaseline] DEBUG: robot_pose={robot_pose}")
        
    #     # Update mapper with current observation
    #     if robot_pose is not None and depth_frame is not None:
    #         position = np.array(robot_pose.position, dtype=np.float64)
    #         rotation = np.array(robot_pose.rotation, dtype=np.float64)  # (x, y, z, w)
            
    #         print(f"[AgentBaseline] DEBUG: position={position}, rotation={rotation}")
            
    #         # Initialize mapper on first valid pose
    #         if not self._mapper_initialized:
    #             self.mapper.reset(position, rotation)
    #             self._mapper_initialized = True
    #             print(f"[AgentBaseline] Mapper initialized at position: {position}")
            
    #         # Update mapper with current RGBD observation
    #         try:
    #             await asyncio.to_thread(
    #                 self.mapper.update,
    #                 rgb_frame, depth_frame, position, rotation
    #             )
    #             print(f"[AgentBaseline] Mapper updated (iter {self.mapper.update_iterations}) - Objects: {self.mapper.get_appeared_objects()}")
                    
    #         except Exception as e:
    #             import traceback
    #             print(f"[AgentBaseline] Mapper update failed: {e}")
    #             traceback.print_exc()
    #     else:
    #         print(f"[AgentBaseline] WARNING: Skipping mapper update - robot_pose={robot_pose is not None}, depth_frame={depth_frame is not None}")

    #     # Save image in thread pool
    #     timestamp = time.time()
    #     timestamp_str = f"{timestamp:.3f}".replace(".", "_")
    #     img_path = Path(self._temp_dir.name) / (
    #         f"frame_{session.session_id}_{timestamp_str}_"
    #         f"{len(session.metadata.get('frame_history', []))}.jpg"
    #     )

    #     await asyncio.to_thread(self._save_image, rgb_frame, img_path)

    #     # Update history
    #     history = session.metadata.setdefault("frame_history", [])
    #     history.append(str(img_path))
    #     if len(history) > 5:
    #         old_file = history.pop(0)
    #         try:
    #             Path(old_file).unlink(missing_ok=True)
    #         except Exception:
    #             pass

    #     # Run solver in thread pool
    #     # Use only the latest image
    #     output = await asyncio.to_thread(self._run_solver, [str(img_path)])

    #     # Parse output
    #     direct_output = output.get("direct_output", "")
    #     print(f"[AgentBaseline] Agent Output: {direct_output}")

    #     navigation, text_response = self._parse_output(direct_output)

    #     step = Step()

    #     messages = [
    #         {
    #             "type": "NavigationCommand",
    #             "payload": navigation.to_dict(),
    #         },
    #         {
    #             "type": "Step",
    #             "payload": step.to_dict(),
    #         }
    #     ]

    #     if text_response:
    #         messages.append({
    #             "type": "AgentText",
    #             "payload": {"text": text_response}
    #         })

    #     return BaselineResponse(
    #         messages=messages,
    #         reset_state=True,
    #     )

    async def _run_inference(self, session: BaselineSession) -> BaselineResponse | None:
        """纯地图模式：只更新地图，不进行任何推理或控制"""
        rgb_frame = self._get_rgb_frame(session.state.latest_rgbd)
        depth_frame = self._get_depth_frame(session.state.latest_rgbd)
        robot_pose = session.metadata.get("robot_pose")

        if rgb_frame is None or depth_frame is None or robot_pose is None:
            return None

        position = np.array(robot_pose.position, dtype=np.float64)
        rotation = np.array(robot_pose.rotation, dtype=np.float64)

        # 初始化 mapper
        if not self._mapper_initialized:
            self.mapper.reset(position, rotation)
            self._mapper_initialized = True
            print(f"[Mapper] Initialized at {position}")

        # === 关键：测量更新时间 ===
        start_time = time.time()

        await asyncio.to_thread(
            self.mapper.update,
            rgb_frame, depth_frame, position, rotation
        )

        duration = time.time() - start_time
        # 获取各点云数量
        scene_points = self.mapper.scene_pcd.point.positions.shape[0] if hasattr(self.mapper, 'scene_pcd') else 0
        nav_points   = self.mapper.navigable_pcd.point.positions.shape[0] if hasattr(self.mapper, 'navigable_pcd') else 0
        # obj_points   = self.mapper.object_pcd.point.positions.shape[0] if hasattr(self.mapper, 'object_pcd') else 0
        obs_points   = self.mapper.obstacle_pcd.point.positions.shape[0] if hasattr(self.mapper, 'obstacle_pcd') else 0
        traj_points  = len(self.mapper.trajectory_position) if hasattr(self.mapper, 'trajectory_position') else 0
        # frontier_points = self.mapper.frontier_pcd.shape[0] if hasattr(self.mapper, 'frontier_pcd') else 0
        object_entities = len(self.mapper.object_entities) if hasattr(self.mapper, 'object_entities') else 0
        update_iter = getattr(self.mapper, 'update_iterations', 0)

        print(
            f"[Mapper] Update #{update_iter} | Time: {duration:.3f}s\n"
            f"Scene points: {scene_points}\n"
            f"Navigable points: {nav_points}\n"
            f"Obstacle points: {obs_points}\n"
            # f"Object points: {obj_points}\n"
            f"Trajectory points: {traj_points}\n"
            # f"Frontier points: {frontier_points}\n"
            f"Detected object entities: {object_entities}\n"
        )

        update_iter = getattr(self.mapper, 'update_iterations', 0)
        if update_iter > 0 and update_iter % 10 == 0:  # 每100帧保存一次
            self.mapper.save_scene_ply()
        # === 不返回任何动作，让机器人静止或由外部控制 ===
        # 返回 empty response，但保持连接活跃
        return BaselineResponse(messages=[], reset_state=False)

    def _run_solver(self, image_paths: List[str]) -> Dict[str, Any]:
        # Use a generic prompt that aligns with vln.py's expectations
        # vln.py expects a query/task description.
        navigation_task_prompt = "Go to the <我和乔治商店>, task finish upon arrival within 2 meter."
        if self._solver:
            return self._solver.solve(
                navigation_task_prompt,
                image_paths=image_paths,
            )
        return {}

    def _save_image(self, rgb_frame: np.ndarray, img_path: Path) -> None:
        """Save image to disk (runs in thread pool)."""
        img = Image.fromarray(rgb_frame)
        if img.mode != "RGB":
            img = img.convert("RGB")
        img.save(img_path, format="JPEG")

    def _parse_output(self, output_text: str) -> Tuple[NavigationCommand, str]:
        # Default to stop
        pos_offset = np.zeros(3, dtype=float)
        rot_offset = np.array([0.0, 0.0, 0.0, 1.0], dtype=float) # Identity quaternion (x, y, z, w)
        is_stopped = False
        
        # Extract Action from vln.py format
        # Look for "Action:" or "Navigation Goal:" (legacy)
        # The output might contain "**Action**:" or just "Action:"
        # We use a flexible regex to capture the content after the label
        action_match = re.search(
            r"(?:\*\*Action\*\*|Action|Navigation Goal)\s*:\s*(.*)",
            output_text,
            re.IGNORECASE | re.DOTALL
        )
        if action_match:
            action_text = action_match.group(1).strip()
            print(f"[AgentBaseline] Extracted Action Text: {action_text}")
            
            # Case 1: <Move(x, y, yaw)>
            move_match = re.search(r"<Move\(\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*\)>", action_text, re.IGNORECASE)
            if move_match:
                x = float(move_match.group(1))
                y = float(move_match.group(2))
                yaw_deg = float(move_match.group(3))
                print(f"[AgentBaseline] Parsed Move: x={x}, y={y}, yaw={yaw_deg}")
                
                # Mapping vln (X=forward, Y=right) to simulator (Unity: Z=forward, Y=up, X=right).
                # Agent X (Forward) -> Sim Z
                # Agent Y (Right) -> Sim X
                pos_offset[2] = x
                pos_offset[1] = 0.0
                pos_offset[0] = y
                
                # Rotation mapping
                yaw_rad = np.radians(yaw_deg)
                half_angle = yaw_rad / 2
                rot_offset[1] = np.sin(half_angle)
                rot_offset[3] = np.cos(half_angle)
                
            # Case 2: <Rotate(yaw)> (Legacy support, though removed from prompt)
            rotate_match = re.search(r"<Rotate\(\s*(-?\d+\.?\d*)\s*\)>", action_text, re.IGNORECASE)
            if rotate_match:
                yaw_deg = float(rotate_match.group(1))
                print(f"[AgentBaseline] Parsed Rotate: yaw={yaw_deg}")
                yaw_rad = np.radians(yaw_deg)
                # Positive yaw is Left. 
                # Assuming standard quaternion rotation around Y axis.
                half_angle = yaw_rad / 2
                rot_offset[1] = np.sin(half_angle)
                rot_offset[3] = np.cos(half_angle)
                
            # Case 3: <Stop>
            if "<Stop>" in action_text:
                is_stopped = True
                
            # Case 4: <Ask>
            if "<Ask>" in action_text:
                pass 
                
            # Case 5: <Wait(t)>
            if "<Wait" in action_text:
                pass

        return NavigationCommand(
            LocalPositionOffset=pos_offset,
            LocalRotationOffset=rot_offset,
            IsStop=is_stopped  # whether to stop the agent and end task
        ), output_text

    def _get_rgb_frame(self, rgbd) -> np.ndarray:
        frame = rgbd.color
        source = getattr(rgbd, "metadata", {}).get("source")
        if source == "webrtc" and frame.ndim == 3 and frame.shape[-1] == 3:
            # aiortc gives BGR; convert to RGB for downstream tools
            return frame[..., ::-1].copy()
        return frame
    
    def _get_depth_frame(self, rgbd) -> np.ndarray | None:
        """Extract depth frame from RGBD data.
        
        Returns:
            Depth image as numpy array (H, W) or (H, W, 1), or None if not available.
        """
        if rgbd is None:
            return None
        depth = getattr(rgbd, "depth", None)
        if depth is None:
            return None
        # Ensure depth is numpy array
        if not isinstance(depth, np.ndarray):
            depth = np.array(depth)


        # 如果是 HxW，保持；如果是 HxW x 1，去掉最后一维
        if len(depth.shape) == 3 and depth.shape[2] == 1:
            depth = depth[:, :, 0]

        # 归一化到 0~255
        depth_vis = depth.copy()
        depth_vis = np.nan_to_num(depth_vis)  # 避免 nan
        min_val, max_val = np.min(depth_vis), np.max(depth_vis)
        if max_val > min_val:
            depth_vis = (depth_vis - min_val) / (max_val - min_val) * 255.0
        else:
            depth_vis = np.zeros_like(depth_vis)
        depth_vis = depth_vis.astype(np.uint8)

        # 可选伪彩色
        depth_vis_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

        cv2.imshow("Depth Frame", depth_vis_color)
        
        return depth

    def _show_depth_frame(self, rgbd):
        depth = self._get_depth_frame(rgbd)
        if depth is None:
            return

    def _translation_from_agent(self, position: np.ndarray | Tuple[float, float, float]) -> np.ndarray:
        """Pass-through translation that works with TransformData inputs."""
        return np.asarray(position, dtype=float)

    def _rotation_from_agent(self, rotation: np.ndarray | Tuple[float, float, float, float]) -> np.ndarray:
        """Convert simulator quaternion (x, y, z, w) to rotation matrix."""
        try:
            quat = quaternion.quaternion(rotation[3], rotation[0], rotation[1], rotation[2])
            return quaternion.as_rotation_matrix(quat)
        except Exception:
            return np.eye(3, dtype=float)

    def _handle_json_packet(self, session: BaselineSession, envelope: MessageEnvelope) -> None:
        packet = envelope.payload
        session.metadata.setdefault("json_types", set()).add(packet.json_type)

        if packet.json_type == "Init":
            self._initialized = True
        elif packet.json_type == "TransformData" and isinstance(packet.content, dict):
            try:
                transform = TransformData.from_dict(packet.content)
                if transform.ObjectName == "Baseline":
                    session.metadata["robot_pose"] = transform
            except (KeyError, TypeError, ValueError) as e:
                print("Error TransformData")
        elif packet.json_type == "SimulationTime":
            # 期望 content = {"time": float}
            try:
                sim_time = packet.content.get("time")
                if sim_time is not None:
                    session.metadata["simulation_time"] = float(sim_time)
            except (AttributeError, TypeError, ValueError):
                pass

    def _ready_to_respond(self, session: BaselineSession) -> bool:
        if not self._initialized:
            return False

        if session.state.latest_rgbd is None:
            return False

        packets = session.state.json_packets
        # Require Init + TransformData (Instruction optional for this baseline)
        return "TransformData" in packets


def create_baseline() -> ClosedLoopBaseline:
    """Factory required by the server to instantiate the baseline."""
    return AgentBaseline()
