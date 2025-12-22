"""WebRTC signaling and media bridge for the closed-loop server."""

from __future__ import annotations

import asyncio
import json
import logging
import os
from typing import Any, Awaitable, Callable, Dict, Optional

import numpy as np
from aiortc import (
    MediaStreamTrack,
    RTCPeerConnection,
    RTCIceCandidate,
    RTCConfiguration,
    RTCIceServer,
    RTCSessionDescription,
)
from aiortc.sdp import candidate_from_sdp

from .messages import RGBDFrame

logger = logging.getLogger(__name__)

SignalingSender = Callable[[Dict[str, Any]], Awaitable[None]]
FrameCallback = Callable[[RGBDFrame], Awaitable[None]]


class WebRTCBridge:
    """Handles WebRTC signaling over the existing websocket connection.

    The bridge responds to Unity's ``WebRTCSignalingMessage`` packets by
    performing the SDP exchange and forwarding ICE candidates. Incoming video
    tracks are mapped to RGB/Depth channels and forwarded as ``RGBDFrame``
    objects to the server/baseline pipeline.
    """

    def __init__(
        self,
        *,
        send_signaling: SignalingSender,
        on_frame: FrameCallback,
    ) -> None:
        self._send_signaling = send_signaling
        self._on_frame = on_frame

        ice_servers = self._load_ice_servers()
        ice_transport_policy = os.environ.get("ICE_TRANSPORT_POLICY", "all").lower()
        config = RTCConfiguration(iceServers=ice_servers)
        if hasattr(config, "iceTransportPolicy"):
            config.iceTransportPolicy = ice_transport_policy
        else:  # pragma: no cover - compatibility with older aiortc
            logger.warning("iceTransportPolicy not supported by this aiortc; using default")
        
        # Add ICE gathering timeout and bundle policy for better connectivity
        if hasattr(config, "bundlePolicy"):
            config.bundlePolicy = "max-bundle"
        if hasattr(config, "rtcpMuxPolicy"):
            config.rtcpMuxPolicy = "require"
            
        logger.info("WebRTC config: servers=%s, transport_policy=%s", ice_servers, ice_transport_policy)

        self._pc = RTCPeerConnection(configuration=config)
        self._tracks: list[MediaStreamTrack] = []
        self._color_frame: Optional[np.ndarray] = None
        self._depth_frame: Optional[np.ndarray] = None
        self._frame_lock = asyncio.Lock()
        self._latest_jpeg: Optional[bytes] = None
        self._pending_candidates: list[Dict[str, Any]] = []  # buffer for early candidates
        self._has_remote_description = False
        self._frame_count = [0, 0]  # Per-track frame counters
        self._last_frame_log_time = [0.0, 0.0]  # Per-track timing
        self._total_frames_received = 0
        self._frames_sent_to_baseline = 0
        self._last_jpeg_update = 0.0  # Last time JPEG was encoded
        
        # Load JPEG update interval from config
        try:
            from .config import config
            self._jpeg_update_interval = config.JPEG_UPDATE_INTERVAL
        except ImportError:
            self._jpeg_update_interval = float(os.environ.get("JPEG_UPDATE_INTERVAL", "0.1"))

        @self._pc.on("track")
        def on_track(track) -> None:
            track_id = len(self._tracks)
            self._tracks.append(track)
            logger.info("✓ WebRTC track %d received: kind=%s, id=%s, readyState=%s", 
                        track_id, track.kind, track.id, track.readyState)
            asyncio.create_task(self._consume_track(track_id, track))

        @self._pc.on("connectionstatechange")
        async def on_connection_state_change() -> None:
            logger.info("WebRTC Connection state: %s", self._pc.connectionState)
            if self._pc.connectionState == "failed":
                logger.error("WebRTC connection failed! Check Unity and network logs.")
            elif self._pc.connectionState == "disconnected":
                logger.warning("WebRTC connection disconnected")

        @self._pc.on("iceconnectionstatechange")
        async def on_ice_connection_change() -> None:
            logger.info("WebRTC ICE state: %s", self._pc.iceConnectionState)
            if self._pc.iceConnectionState == "checking":
                logger.info("ICE candidates are being checked...")
            elif self._pc.iceConnectionState == "connected":
                logger.info("✓ ICE connection established")
            elif self._pc.iceConnectionState == "completed":
                logger.info("✓ ICE connection completed")
            elif self._pc.iceConnectionState == "failed":
                logger.error("✗ ICE connection failed - this shouldn't happen on localhost!")
                logger.error("Possible causes: firewall blocking UDP, Unity WebRTC config issue")
            elif self._pc.iceConnectionState == "disconnected":
                logger.warning("ICE connection disconnected - may be temporary")

        @self._pc.on("icecandidate")
        async def on_icecandidate(candidate) -> None:
            if candidate is None:  # end-of-candidates
                logger.info("ICE gathering complete")
                return
            try:
                candidate_str = getattr(candidate, "candidate", None) or candidate.to_sdp()
                # Log candidate type for debugging
                if "typ host" in candidate_str:
                    logger.debug("ICE candidate (host/local): %s", candidate_str[:80])
                elif "typ srflx" in candidate_str:
                    logger.debug("ICE candidate (srflx/STUN): %s", candidate_str[:80])
                elif "typ relay" in candidate_str:
                    logger.debug("ICE candidate (relay/TURN): %s", candidate_str[:80])
                
                payload = {
                    "type": "candidate",
                    "candidate": candidate_str,
                    "sdpMid": candidate.sdpMid,
                    "sdpMLineIndex": candidate.sdpMLineIndex,
                }
                await self._send_signaling(payload)
            except Exception as exc:  # pragma: no cover - best-effort logging
                logger.warning("Failed to forward ICE candidate: %s", exc)

    async def handle_signaling(self, message: Dict[str, Any]) -> None:
        msg_type = message.get("type")
        if msg_type == "offer":
            await self._handle_offer(message)
        elif msg_type == "answer":
            await self._handle_answer(message)
        elif msg_type == "candidate":
            await self._handle_candidate(message)
        else:
            logger.warning("Unsupported WebRTC signaling message: %s", msg_type)

    async def _handle_offer(self, offer: Dict[str, Any]) -> None:
        logger.info("Received WebRTC offer")
        await self._pc.setRemoteDescription(
            RTCSessionDescription(sdp=offer["sdp"], type="offer")
        )
        self._has_remote_description = True
        # Process any buffered candidates
        if self._pending_candidates:
            logger.info("Processing %d buffered ICE candidates", len(self._pending_candidates))
            for cand in self._pending_candidates:
                await self._add_ice_candidate(cand)
            self._pending_candidates.clear()
        answer = await self._pc.createAnswer()
        await self._pc.setLocalDescription(answer)
        await self._send_signaling(
            {"type": "answer", "sdp": self._pc.localDescription.sdp}
        )
        logger.info("✓ Sent WebRTC answer - waiting for tracks...")

    async def _handle_answer(self, answer: Dict[str, Any]) -> None:
        logger.info("Received WebRTC answer")
        await self._pc.setRemoteDescription(
            RTCSessionDescription(sdp=answer["sdp"], type="answer")
        )
        self._has_remote_description = True
        # Process any buffered candidates
        if self._pending_candidates:
            logger.info("Processing %d buffered ICE candidates", len(self._pending_candidates))
            for cand in self._pending_candidates:
                await self._add_ice_candidate(cand)
            self._pending_candidates.clear()

    async def _handle_candidate(self, cand: Dict[str, Any]) -> None:
        if not self._has_remote_description:
            # Buffer candidates that arrive before remote description
            self._pending_candidates.append(cand)
            logger.debug("Buffered early ICE candidate (total: %d)", len(self._pending_candidates))
            return
        await self._add_ice_candidate(cand)

    async def _add_ice_candidate(self, cand: Dict[str, Any]) -> None:
        candidate_str = cand.get("candidate")
        sdp_mid = cand.get("sdpMid")
        mline_index = cand.get("sdpMLineIndex")
        if candidate_str is None or sdp_mid is None or mline_index is None:
            logger.warning("Incomplete ICE candidate payload: %s", cand)
            return
        ice = candidate_from_sdp(candidate_str)
        ice.sdpMid = sdp_mid
        ice.sdpMLineIndex = int(mline_index)
        ice.candidate = candidate_str
        await self._pc.addIceCandidate(ice)

    async def _consume_track(self, track_id: int, track) -> None:
        logger.info("Starting to consume track %d (%s)...", track_id, track.kind)
        frame_count = 0
        last_log_time = 0
        import time
        
        while True:
            try:
                logger.debug("Track %d: Waiting for frame...", track_id)
                frame = await track.recv()
                frame_count += 1
                
                now = time.time()
                if frame_count == 1:
                    logger.info("✓ First frame received from track %d!", track_id)
                
                # Log every second for debugging
                if now - last_log_time >= 1.0:
                    logger.info("Track %d: recv() returned frame #%d (%.1f fps)", 
                                track_id, frame_count, frame_count / max(now - last_log_time, 1))
                    last_log_time = now
                    
            except Exception as exc:  # pragma: no cover - runtime guard
                logger.warning("WebRTC track %s ended: %s (received %d frames)", track_id, exc, frame_count)
                break

            try:
                array = frame.to_ndarray(format="bgr24")
                logger.debug("Track %d: Frame decoded to array shape=%s", track_id, array.shape)
            except Exception as exc:  # pragma: no cover - decode guard
                logger.warning("Failed to decode WebRTC frame: %s", exc)
                continue

            await self._handle_frame(track_id, array)

    async def _handle_frame(self, track_id: int, array: np.ndarray) -> None:
        import time
        
        self._total_frames_received += 1
        logger.debug("_handle_frame: track=%d, total_received=%d", track_id, self._total_frames_received)
        
        now = time.time()
        
        # Update frame storage (non-blocking)
        if track_id == 0:
            self._color_frame = array
            # Update JPEG preview at regular intervals (async)
            if now - self._last_jpeg_update >= self._jpeg_update_interval:
                self._last_jpeg_update = now
                # Encode in background to avoid blocking
                asyncio.create_task(self._update_jpeg_preview(array.copy()))
        elif track_id == 1:
            self._depth_frame = array
        else:
            return

        # Log frame rate per track
        self._frame_count[track_id] += 1
        if now - self._last_frame_log_time[track_id] >= 2.0:  # every 2 seconds per track
            elapsed = now - self._last_frame_log_time[track_id] if self._last_frame_log_time[track_id] > 0 else 1.0
            fps = self._frame_count[track_id] / elapsed
            logger.info("Track %d: %.1f fps (%d frames in %.1fs) | Total: %d received, %d sent to baseline",
                        track_id, fps, self._frame_count[track_id], elapsed,
                        self._total_frames_received, self._frames_sent_to_baseline)
            self._frame_count[track_id] = 0
            self._last_frame_log_time[track_id] = now

        # Send to baseline only when both frames available (but don't block reception)
        if self._color_frame is not None and self._depth_frame is not None:
            logger.debug("Both frames available, creating RGBD and sending to baseline")
            rgbd = RGBDFrame(
                color=self._color_frame.copy(),  # Copy to avoid race conditions
                depth=self._depth_frame.copy(),
                metadata={"source": "webrtc", "track_id": track_id},
            )
            self._frames_sent_to_baseline += 1
            # Fire and forget - don't block frame reception
            asyncio.create_task(self._on_frame(rgbd))
        else:
            logger.debug("Waiting for both frames: color=%s, depth=%s", 
                        self._color_frame is not None, self._depth_frame is not None)
    
    async def _update_jpeg_preview(self, frame: np.ndarray) -> None:
        """Update JPEG preview in background thread."""
        def encode():
            return self._encode_jpeg(frame)
        
        jpeg = await asyncio.to_thread(encode)
        if jpeg is not None:
            self._latest_jpeg = jpeg

    def get_latest_jpeg(self) -> Optional[bytes]:
        return self._latest_jpeg

    async def close(self) -> None:
        await self._pc.close()

    @staticmethod
    def _encode_jpeg(frame: np.ndarray) -> Optional[bytes]:
        try:
            import cv2  # Local import to avoid hard dependency on import time

            ok, buf = cv2.imencode(".jpg", frame)
            if not ok:
                return None
            return buf.tobytes()
        except Exception:
            return None

    @staticmethod
    def _load_ice_servers() -> list[RTCIceServer]:
        env_value = os.environ.get("ICE_SERVERS")
        if env_value:
            try:
                servers_raw = json.loads(env_value)
                servers: list[RTCIceServer] = []
                if isinstance(servers_raw, dict):
                    servers_raw = [servers_raw]
                if isinstance(servers_raw, list):
                    for item in servers_raw:
                        if isinstance(item, str):
                            servers.append(RTCIceServer(urls=item))
                        elif isinstance(item, dict):
                            servers.append(
                                RTCIceServer(
                                    urls=item.get("urls"),
                                    username=item.get("username"),
                                    credential=item.get("credential"),
                                )
                            )
                if servers:
                    return servers
            except Exception as exc:  # pragma: no cover - best effort logging
                logger.warning("Failed to parse ICE_SERVERS env: %s", exc)
        # For localhost: minimal STUN config (host candidates should work directly)
        return [RTCIceServer(urls="stun:stun.l.google.com:19302")]


__all__ = ["WebRTCBridge"]
