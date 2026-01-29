"""Default baseline implementation used for smoke testing the server."""

from __future__ import annotations

import asyncio
import time
from typing import Optional

import numpy as np

from .framework import BaselineResponse, BaselineSession, ClosedLoopBaseline, MessageEnvelope
from .messages import NavigationCommand, Step, TransformData


class SimpleStepBaseline(ClosedLoopBaseline):
    """A minimal baseline that keeps the agent stationary but steps the simulator.
    
    Uses the new architecture:
    - handle_envelope() only updates the frame buffer
    - Separate inference loop triggers responses at controlled intervals
    """

    def __init__(self) -> None:
        self._initialized = False
        self._inference_lock = asyncio.Lock()

    async def on_session_start(self, session: BaselineSession) -> None:
        session.metadata.clear()

    async def on_session_end(self, session: BaselineSession) -> None:
        session.state.clear()

    async def handle_envelope(
        self, session: BaselineSession, envelope: MessageEnvelope
    ) -> BaselineResponse | None:
        if envelope.message_type == "json":
            self._handle_json_packet(session, envelope)
        elif envelope.message_type == "rgbd":
            session.metadata["has_rgbd"] = True

        if not self._ready_to_respond(session):
            return None

        if self._inference_lock.locked():
            return None

        async with self._inference_lock:
            return await self._run_inference(session)

    async def _run_inference(self, session: BaselineSession) -> BaselineResponse | None:
        navigation = NavigationCommand(
            LocalPositionOffset=np.zeros(3, dtype=float),
            LocalRotationOffset=np.array([0.0, 0.0, 0.0, 1.0], dtype=float),
            IsStop=False,
        )
        step = Step()

        return BaselineResponse(
            messages=[
                {
                    "type": "NavigationCommand",
                    "payload": navigation.to_dict(),
                },
                {
                    "type": "Step",
                    "payload": step.to_dict(),
                },
            ],
            reset_state=True,
        )

    def _handle_json_packet(self, session: BaselineSession, envelope: MessageEnvelope) -> None:
        packet = envelope.payload
        session.metadata.setdefault("json_types", set()).add(packet.json_type)

        if packet.json_type == "Init":
            self._initialized = True
        elif packet.json_type == "TransformData" and isinstance(packet.content, dict):
            try:
                session.metadata["transform"] = TransformData.from_dict(packet.content)
            except (KeyError, TypeError, ValueError):  # pragma: no cover - defensive parse
                session.metadata["transform"] = packet.content

    def _ready_to_respond(self, session: BaselineSession) -> bool:
        if not self._initialized:
            return False

        if session.state.latest_rgbd is None:
            return False

        packets = session.state.json_packets
        return "Instruction" in packets and "TransformData" in packets


def create_baseline() -> ClosedLoopBaseline:
    """Factory required by the server to instantiate the baseline."""

    return SimpleStepBaseline()


__all__ = ["SimpleStepBaseline", "create_baseline"]
