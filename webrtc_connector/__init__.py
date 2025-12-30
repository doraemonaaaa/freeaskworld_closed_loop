"""FreeAskWorld connector package for closed-loop baselines.

This package provides:
- WebRTC-based server (webrtc_server.py) - Recommended for production
- Legacy WebSocket server (server.py) - For backward compatibility
"""

from .framework import (
    BaselineFactory,
    BaselineResponse,
    BaselineSession,
    ClosedLoopBaseline,
    MessageEnvelope,
    SessionState,
    load_baseline_factory,
)
from .messages import (
    JsonPacket,
    NavigationCommand,
    RGBDFrame,
    Step,
    TransformData,
)

__all__ = [
    # Framework
    "BaselineFactory",
    "BaselineResponse",
    "BaselineSession",
    "ClosedLoopBaseline",
    "MessageEnvelope",
    "SessionState",
    "load_baseline_factory",
    # Messages
    "JsonPacket",
    "NavigationCommand",
    "RGBDFrame",
    "Step",
    "TransformData",
]
