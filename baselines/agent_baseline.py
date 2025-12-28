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

import numpy as np
import quaternion
from PIL import Image
from dotenv import load_dotenv

# Adjust path to ensure agentflow can be imported if running from FreeAskAgent root
import sys
if str(Path(__file__).parents[2]) not in sys.path:
    sys.path.append(str(Path(__file__).parents[2]))

try:
    from agentflow.agentflow.solver_embodied import construct_solver_embodied
except ImportError:
    # Fallback or error handling if agentflow is not found
    print("Warning: agentflow not found. AgentBaseline will fail.")

from ..freeaskworld_connector.framework import BaselineResponse, BaselineSession, ClosedLoopBaseline, MessageEnvelope
from ..freeaskworld_connector.messages import NavigationCommand, Step, TransformData
from .mapper import Mapper

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
        # If None, mapper will estimate from FOV
        self.camera_intrinsics = np.array([[415.6922, 0, 320],
                                    [0, 415.6922, 240],
                                    [0, 0, 1]])
        # Example for 256x256 image with 90 deg FOV:
        # self.camera_intrinsics = np.array([
        #     [128.0, 0.0, 128.0],
        #     [0.0, 128.0, 128.0],
        #     [0.0, 0.0, 1.0]
        # ])
        self.mapper = Mapper(
            camera_intrinsic=self.camera_intrinsics,
            translation_func=self._translation_from_agent,
            rotation_func=self._rotation_from_agent,
        )
        self._mapper_initialized = False

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
        self._mapper_initialized = False

    async def on_session_end(self, session: BaselineSession) -> None:
        
        # Cleanup temp files for this session
        if "frame_history" in session.metadata:
            for path_str in session.metadata["frame_history"]:
                try:
                    Path(path_str).unlink(missing_ok=True)
                except Exception:
                    pass
        session.state.clear()
        # Cleanup temp dir if needed, but we keep it for the lifetime of the baseline instance
        self._mapper_initialized = False

    async def handle_envelope(
        self, session: BaselineSession, envelope: MessageEnvelope
    ) -> BaselineResponse | None:
        if envelope.message_type == "json":
            self._handle_json_packet(session, envelope)
        elif envelope.message_type == "rgbd":
            session.metadata["has_rgbd"] = True
            session.state.update_rgbd(envelope.payload)

        self._try_update_map(session)

        if not self._ready_to_respond(session):
            return None

        # Only run one inference at a time; drop intermediate triggers
        if self._inference_lock.locked():
            return None

        async with self._inference_lock:
            return await self._run_inference(session)

    def _try_update_map(self, session: BaselineSession) -> None:
        rgbd = session.state.latest_rgbd
        if rgbd is None:
            return

        rgb_frame = self._get_rgb_frame(rgbd)
        if rgb_frame is None:
            return

        # Skip mapping until a valid depth frame arrives
        if getattr(rgbd, "depth", None) is None:
            return
        try:
            if np.sum(rgbd.depth) == 0:
                return
        except Exception:
            return

        robot_pose = session.metadata.get("robot_pose")
        
        # If RobotPose is not explicitly sent, try to use TransformData
        if robot_pose is None and "transform" in session.metadata:
            t = session.metadata["transform"]
            # TransformData has position (tuple) and rotation (tuple)
            robot_pos_arr = np.array(t.position)
            robot_rot_arr = np.array(t.rotation)
        elif robot_pose:
            # robot_pose is now a TransformData object if coming from "Baseline" check
            # or could be the old RobotPose if we hadn't removed it, but we did.
            # In _handle_json_packet we set session.metadata["robot_pose"] = transform (TransformData)
            robot_pos_arr = np.array(robot_pose.position)
            robot_rot_arr = np.array(robot_pose.rotation)
        else:
            return # No pose data

        if not self._mapper_initialized:
            self.mapper.reset(robot_pos_arr, robot_rot_arr)
            self._mapper_initialized = True

        self.mapper.update(
            rgb=rgb_frame,
            depth=rgbd.depth,
            position=robot_pos_arr,
            rotation=robot_rot_arr,
        )

    async def _run_inference(self, session: BaselineSession) -> BaselineResponse | None:
        """Execute one inference pass using the latest state."""
        # Get the latest RGB image
        rgb_frame = self._get_rgb_frame(session.state.latest_rgbd)
        if rgb_frame is None:
            return None

        # Save image in thread pool
        timestamp = time.time()
        timestamp_str = f"{timestamp:.3f}".replace(".", "_")
        img_path = Path(self._temp_dir.name) / (
            f"frame_{session.session_id}_{timestamp_str}_"
            f"{len(session.metadata.get('frame_history', []))}.jpg"
        )

        await asyncio.to_thread(self._save_image, rgb_frame, img_path)

        # Update history
        history = session.metadata.setdefault("frame_history", [])
        history.append(str(img_path))
        if len(history) > 5:
            old_file = history.pop(0)
            try:
                Path(old_file).unlink(missing_ok=True)
            except Exception:
                pass

        # Run solver in thread pool
        # Use only the latest image
        output = await asyncio.to_thread(self._run_solver, [str(img_path)])

        # Parse output
        direct_output = output.get("direct_output", "")
        print(f"[AgentBaseline] Agent Output: {direct_output}")

        navigation, text_response = self._parse_output(direct_output)

        step = Step()

        messages = [
            {
                "type": "NavigationCommand",
                "payload": navigation.to_dict(),
            },
            {
                "type": "Step",
                "payload": step.to_dict(),
            }
        ]

        if text_response:
            messages.append({
                "type": "AgentText",
                "payload": {"text": text_response}
            })

        return BaselineResponse(
            messages=messages,
            reset_state=True,
        )

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
                session.metadata["transform"] = transform
                if transform.ObjectName == "Baseline":
                    session.metadata["robot_pose"] = transform
            except (KeyError, TypeError, ValueError):
                session.metadata["transform"] = packet.content
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
