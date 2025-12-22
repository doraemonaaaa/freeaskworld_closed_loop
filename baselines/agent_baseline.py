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
from PIL import Image
from dotenv import load_dotenv

# Adjust path to ensure agentflow can be imported if running from FreeAskAgent root
import sys
if str(Path(__file__).parents[2]) not in sys.path:
    sys.path.append(str(Path(__file__).parents[2]))

try:
    from agentflow.agentflow.solver import construct_solver
    from agentflow.agentflow.solver_fast import construct_fast_solver
except ImportError:
    # Fallback or error handling if agentflow is not found
    print("Warning: agentflow not found. AgentBaseline will fail.")
    construct_solver = None
    construct_fast_solver = None

from ..freeaskworld_connector.framework import BaselineResponse, BaselineSession, ClosedLoopBaseline, MessageEnvelope
from ..freeaskworld_connector.messages import NavigationCommand, Step, TransformData

class AgentBaseline(ClosedLoopBaseline):
    """Baseline that uses the AgentFlow solver to control the agent."""

    def __init__(self) -> None:
        self._initialized = False
        self._solver = None
        self._setup_solver()
        self._temp_dir = tempfile.TemporaryDirectory()
        self._inference_lock = asyncio.Lock()

    def _setup_solver(self):
        # Load environment variables
        env_path = Path(__file__).parents[2] / "agentflow" / ".env"
        load_dotenv(dotenv_path=env_path)
        
        print("Proxy_API_BASE:" + os.environ.get("Proxy_API_BASE", "Not Set"))
        print("OPENAI_API_KEY:" + os.environ.get("OPENAI_API_KEY", "Not Set"))

        llm_engine_name = os.environ.get("LLM_ENGINE_NAME", "gpt-4o")
        # fast_mode = os.environ.get("FAST_MODE", "false").lower() == "true"
        fast_mode = True  # Always use fast mode for this baseline

        if fast_mode and construct_fast_solver:
            # Fast planner-only solver; lightweight but less capable.
            self._solver = construct_fast_solver(
                llm_engine_name=llm_engine_name,
                enabled_tools=["Base_Generator_Tool", "GroundedSAM2_Tool"],
                tool_engine=["gpt-4o"],
                output_types="direct",
                max_steps=1,
                max_time=10,
                max_tokens=1024,
                fast_max_tokens=256,
                enable_multimodal=True,
                verbose=True,
            )
        elif construct_solver:
            # Full solver for higher-quality navigation planning.
            self._solver = construct_solver(
                llm_engine_name=llm_engine_name,
                enabled_tools=["Base_Generator_Tool", "GroundedSAM2_Tool"],
                tool_engine=["gpt-4o"],
                model_engine=["gpt-4o", "gpt-4o", "gpt-4o", "gpt-4o"],
                output_types="direct",
                max_time=300,
                max_steps=1,
                enable_multimodal=True,
            )
        else:
            print("Warning: solver constructors unavailable; baseline will not respond.")
            self._solver = None

    async def on_session_start(self, session: BaselineSession) -> None:
        session.metadata.clear()
        session.metadata["frame_history"] = []

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
        output = await asyncio.to_thread(self._run_solver, history)

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
        
        # Get the latest RGB image
        rgb_frame = self._get_rgb_frame(session.state.latest_rgbd)
        
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
            # Remove old file
            old_file = history.pop(0)
            try:
                Path(old_file).unlink(missing_ok=True)
            except Exception:
                pass

        # Run solver in thread pool
        output = await asyncio.to_thread(self._run_solver, history)
        
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
        navigation_task_prompt = """
[Rules]
要躲避物体不要撞上
当你离人2m内的时候就可以触发问路
[Policy]
使用最快获取信息的策略，你可选择自己不断探索地点，也可以问人来快速获取信息，尽管可能不精准
[Action Space]
动作空间是[前进，左转，右转，后转, 后退, 停止, 问路][1m, 2m, 3m]
每次动作只能选择一个动作和一个距离, 比如'前进2m'
[Output Format]
请给出后续5步的导航指令序列。
[Tools]
你可以使用GroundedSAM2_Tool来识别图像中的物体，自己设置prompt比如obst，获取物体的位置和类别信息，辅助你做出导航决策。你可以获取obstacle,street,building等信息
[Image Sequence]
这里有一系列按时间顺序排列的图像帧，展示了你当前的视野。请根据这些图像帧来理解环境。
"""
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
        rot_offset = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        is_stopped = False
        
        # Simple parsing logic
        # Look for the first valid action in the text
        # Regex for actions
        action_pattern = r"(前进|后退|左转|右转|后转|停止|问路)\s*(\d+m)?"
        
        # We assume the output contains a list, we take the first one
        # Example: "1. 前进2m"
        
        match = re.search(action_pattern, output_text)
        if match:
            action = match.group(1)
            distance_str = match.group(2)
            distance = float(distance_str.replace('m', '')) if distance_str else 0.0
            
            if action == "前进":
                # Forward is +Z in local frame (assuming)
                pos_offset[2] = distance
            elif action == "后退":
                pos_offset[2] = -distance
            elif action == "左转":
                # Rotate left (around Y axis)
                # Assuming 90 degrees for turn? Or maybe distance implies angle?
                # Usually "Turn Left" is a discrete action. Let's assume 90 degrees.
                # Quaternion for 90 degrees around Y (0, 1, 0)
                # q = [x, y, z, w] = [0, sin(45), 0, cos(45)] = [0, 0.707, 0, 0.707]
                # Note: Check coordinate system. If Y is up.
                # Left turn -> -90 degrees?
                # Let's use -90 degrees (Right Hand Rule around Y points up -> CCW is positive. Left turn is usually CCW).
                # Wait, if I face Z, Left is -X. Turning left means rotating towards +X? No, rotating towards -X.
                # That is +90 degrees around Y?
                # Let's assume +90 degrees around Y.
                # sin(45) = 0.7071, cos(45) = 0.7071
                rot_offset = np.array([0.0, 0.70710678, 0.0, 0.70710678]) 
            elif action == "右转":
                # -90 degrees around Y
                # sin(-45) = -0.7071
                rot_offset = np.array([0.0, -0.70710678, 0.0, 0.70710678])
            elif action == "后转":
                # 180 degrees around Y
                # sin(90) = 1, cos(90) = 0
                rot_offset = np.array([0.0, 1.0, 0.0, 0.0])
            elif action == "停止":
                is_stopped = True
            elif action == "问路":
                pass # No movement
                
        return NavigationCommand(
            LocalPositionOffset=pos_offset,
            LocalRotationOffset=rot_offset,
            IsStop=is_stopped
        ), output_text

    def _get_rgb_frame(self, rgbd) -> np.ndarray:
        frame = rgbd.color
        source = getattr(rgbd, "metadata", {}).get("source")
        if source == "webrtc" and frame.ndim == 3 and frame.shape[-1] == 3:
            # aiortc gives BGR; convert to RGB for downstream tools
            return frame[..., ::-1].copy()
        return frame

    def _handle_json_packet(self, session: BaselineSession, envelope: MessageEnvelope) -> None:
        packet = envelope.payload
        session.metadata.setdefault("json_types", set()).add(packet.json_type)

        if packet.json_type == "Init":
            self._initialized = True
        elif packet.json_type == "TransformData" and isinstance(packet.content, dict):
            try:
                session.metadata["transform"] = TransformData.from_dict(packet.content)
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
