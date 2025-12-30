"""Pure WebRTC server for closed-loop baselines.

This module replaces the WebSocket-based server with a pure WebRTC implementation.
Communication happens through:
- HTTP endpoints for WebRTC signaling (SDP/ICE exchange)
- WebRTC DataChannel for bidirectional JSON messages
- WebRTC MediaTracks for RGBD streaming (optional)
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import io
import json
import logging
import os
import socket
import sys
import uuid
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Optional

import numpy as np
from aiohttp import web
from aiortc import (
    RTCConfiguration,
    RTCDataChannel,
    RTCIceCandidate,
    RTCIceServer,
    RTCPeerConnection,
    RTCSessionDescription,
)
from aiortc.sdp import candidate_from_sdp
from PIL import Image

from .framework import (
    BaselineFactory,
    BaselineResponse,
    BaselineSession,
    ClosedLoopBaseline,
    MessageEnvelope,
    SessionState,
    load_baseline_factory,
)
from .messages import JsonPacket, RGBDFrame
from .preview_http import start_preview_server

logger = logging.getLogger(__name__)


@dataclass(slots=True)
class ServerConfig:
    """Configuration for the WebRTC server."""

    host: str = "0.0.0.0"
    port: int = 8766
    persist_rgbd_dir: Optional[str] = None
    verbose: bool = False


@dataclass
class PeerSession:
    """Represents a connected WebRTC peer."""

    session_id: str
    pc: RTCPeerConnection
    data_channel: Optional[RTCDataChannel] = None
    baseline: Optional[ClosedLoopBaseline] = None
    baseline_session: Optional[BaselineSession] = None
    latest_jpeg: Optional[bytes] = None
    is_ready: bool = False


class WebRTCServer:
    """Pure WebRTC server using HTTP for signaling and DataChannel for messages."""

    def __init__(self, baseline_factory: BaselineFactory, config: ServerConfig) -> None:
        self._baseline_factory = baseline_factory
        self._config = config
        self._peers: Dict[str, PeerSession] = {}
        self._app = web.Application()
        self._setup_routes()
        self._preview_server = None

    def _setup_routes(self) -> None:
        """Setup HTTP routes for WebRTC signaling."""
        self._app.router.add_post("/offer", self._handle_offer)
        self._app.router.add_post("/answer/{session_id}", self._handle_answer)
        self._app.router.add_post("/ice/{session_id}", self._handle_ice_candidate)
        self._app.router.add_get("/status", self._handle_status)
        self._app.router.add_get("/status/{session_id}", self._handle_session_status)
        self._app.router.add_delete("/session/{session_id}", self._handle_close_session)
        self._app.router.add_options("/{path:.*}", self._handle_cors_preflight)

    def _add_cors_headers(self, response: web.Response) -> web.Response:
        """Add CORS headers to allow cross-origin requests."""
        response.headers["Access-Control-Allow-Origin"] = "*"
        response.headers["Access-Control-Allow-Methods"] = "GET, POST, DELETE, OPTIONS"
        response.headers["Access-Control-Allow-Headers"] = "Content-Type"
        return response

    async def _handle_cors_preflight(self, request: web.Request) -> web.Response:
        """Handle CORS preflight requests."""
        return self._add_cors_headers(web.Response())

    async def _handle_offer(self, request: web.Request) -> web.Response:
        """Handle incoming WebRTC offer from Unity client."""
        try:
            data = await request.json()
        except json.JSONDecodeError:
            return self._add_cors_headers(
                web.json_response({"error": "Invalid JSON"}, status=400)
            )

        sdp = data.get("sdp")
        if not sdp:
            return self._add_cors_headers(
                web.json_response({"error": "Missing SDP"}, status=400)
            )

        session_id = str(uuid.uuid4())
        logger.info("🔗 New WebRTC offer received, session: %s", session_id)

        # Create peer connection with ICE servers
        ice_servers = self._load_ice_servers()
        logger.info("Using ICE servers: %s", ice_servers)
        config = RTCConfiguration(iceServers=ice_servers)
        pc = RTCPeerConnection(configuration=config)

        # Create baseline and session
        logger.info("Creating baseline...")
        baseline = self._baseline_factory()
        logger.info("Baseline created")
        baseline_session = BaselineSession(session_id=session_id)

        peer = PeerSession(
            session_id=session_id,
            pc=pc,
            baseline=baseline,
            baseline_session=baseline_session,
        )
        self._peers[session_id] = peer

        # Setup event handlers
        self._setup_peer_handlers(peer)

        # Process the offer
        logger.info("Setting remote description...")
        logger.info("SDP offer contains datachannel: %s", "m=application" in sdp)
        await pc.setRemoteDescription(
            RTCSessionDescription(sdp=sdp, type="offer")
        )
        logger.info("Remote description set")
        logger.info("Current data channels after setRemoteDescription: %s", 
                    [dc.label for dc in peer.pc.getTransceivers() if hasattr(dc, 'label')])
        
        # Check if datachannel was created from offer
        if peer.data_channel:
            logger.info("✅ DataChannel already received from offer: %s", peer.data_channel.label)
        else:
            logger.info("⚠️ No DataChannel received from offer yet")

        # Create answer
        logger.info("Creating answer...")
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)
        logger.info("Answer created, SDP length: %d", len(pc.localDescription.sdp))

        # Start baseline session
        logger.info("Starting baseline session...")
        await baseline.on_session_start(baseline_session)
        logger.info("✅ Baseline session started for %s", session_id)

        response_data = {
            "session_id": session_id,
            "sdp": pc.localDescription.sdp,
            "type": "answer",
        }

        logger.info("Sending answer response to client")
        return self._add_cors_headers(web.json_response(response_data))

    async def _handle_answer(self, request: web.Request) -> web.Response:
        """Handle WebRTC answer (if server creates offer)."""
        session_id = request.match_info["session_id"]
        peer = self._peers.get(session_id)

        if not peer:
            return self._add_cors_headers(
                web.json_response({"error": "Session not found"}, status=404)
            )

        try:
            data = await request.json()
        except json.JSONDecodeError:
            return self._add_cors_headers(
                web.json_response({"error": "Invalid JSON"}, status=400)
            )

        sdp = data.get("sdp")
        if not sdp:
            return self._add_cors_headers(
                web.json_response({"error": "Missing SDP"}, status=400)
            )

        await peer.pc.setRemoteDescription(
            RTCSessionDescription(sdp=sdp, type="answer")
        )

        return self._add_cors_headers(web.json_response({"status": "ok"}))

    async def _handle_ice_candidate(self, request: web.Request) -> web.Response:
        """Handle ICE candidate from client."""
        session_id = request.match_info["session_id"]
        peer = self._peers.get(session_id)

        if not peer:
            return self._add_cors_headers(
                web.json_response({"error": "Session not found"}, status=404)
            )

        try:
            data = await request.json()
        except json.JSONDecodeError:
            return self._add_cors_headers(
                web.json_response({"error": "Invalid JSON"}, status=400)
            )

        candidate_str = data.get("candidate")
        sdp_mid = data.get("sdpMid")
        sdp_mline_index = data.get("sdpMLineIndex")

        if candidate_str and sdp_mid is not None and sdp_mline_index is not None:
            try:
                ice = candidate_from_sdp(candidate_str)
                ice.sdpMid = sdp_mid
                ice.sdpMLineIndex = int(sdp_mline_index)
                await peer.pc.addIceCandidate(ice)
                logger.info("Added ICE candidate for session %s: %s", session_id, candidate_str[:80])
            except Exception as exc:
                logger.warning("Failed to add ICE candidate: %s", exc)

        return self._add_cors_headers(web.json_response({"status": "ok"}))

    async def _handle_status(self, request: web.Request) -> web.Response:
        """Return server status and active sessions."""
        sessions = []
        for sid, peer in self._peers.items():
            sessions.append({
                "session_id": sid,
                "connection_state": peer.pc.connectionState if peer.pc else "unknown",
                "ice_state": peer.pc.iceConnectionState if peer.pc else "unknown",
                "data_channel_ready": peer.is_ready,
            })

        return self._add_cors_headers(web.json_response({
            "status": "running",
            "active_sessions": len(self._peers),
            "sessions": sessions,
        }))

    async def _handle_session_status(self, request: web.Request) -> web.Response:
        """Return status for a specific session."""
        session_id = request.match_info["session_id"]
        peer = self._peers.get(session_id)

        if not peer:
            return self._add_cors_headers(
                web.json_response({"error": "Session not found"}, status=404)
            )

        return self._add_cors_headers(web.json_response({
            "session_id": session_id,
            "connection_state": peer.pc.connectionState if peer.pc else "unknown",
            "ice_state": peer.pc.iceConnectionState if peer.pc else "unknown",
            "data_channel_ready": peer.is_ready,
        }))

    async def _handle_close_session(self, request: web.Request) -> web.Response:
        """Close a specific session."""
        session_id = request.match_info["session_id"]
        await self._cleanup_peer(session_id)
        return self._add_cors_headers(web.json_response({"status": "closed"}))

    def _setup_peer_handlers(self, peer: PeerSession) -> None:
        """Setup WebRTC event handlers for a peer connection."""

        @peer.pc.on("connectionstatechange")
        async def on_connection_state_change():
            state = peer.pc.connectionState
            logger.info("Session %s: Connection state -> %s", peer.session_id, state)
            if state in ("failed", "closed"):
                await self._cleanup_peer(peer.session_id)

        @peer.pc.on("iceconnectionstatechange")
        async def on_ice_state_change():
            state = peer.pc.iceConnectionState
            logger.info("Session %s: ICE state -> %s", peer.session_id, state)
            if state == "connected":
                logger.info("✅ ICE connected for session %s", peer.session_id)
            elif state == "failed":
                logger.error("❌ ICE failed for session %s", peer.session_id)
            elif state == "checking":
                logger.info("🔍 ICE checking candidates for session %s", peer.session_id)

        @peer.pc.on("icegatheringstatechange")
        async def on_ice_gathering_state_change():
            state = peer.pc.iceGatheringState
            logger.info("Session %s: ICE gathering state -> %s", peer.session_id, state)
            if state == "complete":
                # Log local candidates
                for transceiver in peer.pc.getTransceivers():
                    if transceiver.sender and transceiver.sender.transport:
                        ice = transceiver.sender.transport.transport
                        if hasattr(ice, '_local_candidates'):
                            for c in ice._local_candidates:
                                logger.info("  Local candidate: %s", c)

        @peer.pc.on("datachannel")
        def on_datachannel(channel: RTCDataChannel):
            logger.info(
                "✅ DataChannel '%s' received for session %s, readyState=%s",
                channel.label,
                peer.session_id,
                channel.readyState,
            )
            peer.data_channel = channel
            peer.is_ready = True
            self._setup_datachannel_handlers(peer, channel)
            logger.info("DataChannel handlers setup complete")

        @peer.pc.on("track")
        def on_track(track):
            logger.info(
                "🎥 Media track received: kind=%s, id=%s",
                track.kind,
                track.id,
            )
            # Handle video tracks for RGBD streaming if needed
            asyncio.create_task(self._consume_video_track(peer, track))

    def _setup_datachannel_handlers(
        self, peer: PeerSession, channel: RTCDataChannel
    ) -> None:
        """Setup handlers for DataChannel messages."""
        logger.info("Setting up DataChannel handlers for channel '%s'", channel.label)

        @channel.on("open")
        def on_open():
            logger.info("🟢 DataChannel '%s' is now OPEN for session %s", channel.label, peer.session_id)

        @channel.on("message")
        def on_message(message):
            # Note: aiortc event handlers must be synchronous; use create_task for async work
            if self._config.verbose:
                logger.info("📨 DataChannel message received, type=%s, len=%d", 
                        type(message).__name__, len(message) if message else 0)
            try:
                if isinstance(message, bytes):
                    data = json.loads(message.decode("utf-8"))
                else:
                    data = json.loads(message)
                if self._config.verbose:
                    logger.info("Parsed message keys: %s; type=%s; json_type=%s", 
                                list(data.keys()) if isinstance(data, dict) else "not a dict",
                                data.get("type"), data.get("json_type") if isinstance(data, dict) else None)
                # Schedule async handler on the event loop
                asyncio.create_task(self._handle_datachannel_message(peer, data))
            except json.JSONDecodeError as exc:
                logger.warning("Invalid JSON from DataChannel: %s", exc)
            except Exception as exc:
                logger.exception("Error handling DataChannel message: %s", exc)

        @channel.on("close")
        def on_close():
            logger.info("DataChannel closed for session %s", peer.session_id)
            peer.is_ready = False

        @channel.on("error")
        def on_error(error):
            logger.error("DataChannel error for session %s: %s", peer.session_id, error)

    async def _handle_datachannel_message(
        self, peer: PeerSession, data: Dict[str, Any]
    ) -> None:
        """Process a message received from DataChannel.
        
        Message format: {"type": "xxx", "json_type": "...", "content": {...}}
        - type: rgbd_stream, json, metadata, or direct types (Init, TransformData, etc)
        - json_type: message subtype (for type="json")
        - content: message payload
        """
        msg_type = data.get("type", "")
        msg_type_lower = msg_type.lower() if msg_type else ""
        
        logger.debug("DataChannel message: type=%s", msg_type)

        if msg_type_lower in ("rgbd_stream", "rgbd"):
            # Unity RGBDStreamSender uses 'payload', fallback to 'content' for compatibility
            content = data.get("payload") or data.get("content") or data
            await self._handle_rgbd_payload(peer, content)
        elif msg_type_lower == "json":
            await self._handle_json_payload(peer, data)
        elif msg_type_lower == "metadata":
            content = data.get("payload") or data.get("content") or {}
            peer.baseline_session.metadata.update(content)
        elif msg_type_lower in ("init", "transformdata", "instruction", "simulationtime", "step", "navigationcommand"):
            # Direct Unity message types -> convert to standard format
            converted = {
                "json_type": msg_type,
                "content": data.get("content", data),
            }
            logger.info("Received Unity message: %s", msg_type)
            await self._handle_json_payload(peer, converted)
        else:
            json_type = data.get("json_type")
            if json_type:
                await self._handle_json_payload(peer, data)
            else:
                logger.warning("Unknown message type: %s", msg_type)

    async def _handle_rgbd_payload(
        self, peer: PeerSession, payload: Dict[str, Any]
    ) -> None:
        """Process RGBD frame received via DataChannel."""
        try:
            color_b64 = payload.get("color")
            depth_b64 = payload.get("depth")
            depth_format = (payload.get("depth_format") or "").lower()
            depth_scale = float(payload.get("depth_scale") or 1.0)

            if not color_b64 or not depth_b64:
                logger.warning("RGBD payload missing color or depth")
                return

            color_bytes = base64.b64decode(color_b64)
            depth_bytes = base64.b64decode(depth_b64)

            with Image.open(io.BytesIO(color_bytes)) as color_img:
                color_array_rgb = np.asarray(color_img)

            if depth_format == "float32_m":
                # Raw float32 depth in meters from Unity sender
                depth_array = np.frombuffer(depth_bytes, dtype=np.float32)
                expected_size = color_array_rgb.shape[0] * color_array_rgb.shape[1]
                if depth_array.size != expected_size:
                    raise ValueError(
                        f"Depth size mismatch: got {depth_array.size}, expected {expected_size}"
                    )
                depth_array = depth_array.reshape((color_array_rgb.shape[0], color_array_rgb.shape[1]))
                if depth_scale != 1.0:
                    depth_array = depth_array * depth_scale
            else:
                # Fallback: treat as image-encoded depth
                with Image.open(io.BytesIO(depth_bytes)) as depth_img:
                    depth_array = np.asarray(depth_img)

            if self._config.verbose:
                logger.info(
                    "🖼️ RGBD frame received: %sx%s, depth dtype=%s, shape=%s, min=%.4f, max=%.4f",
                    color_array_rgb.shape[1],
                    color_array_rgb.shape[0],
                    str(depth_array.dtype),
                    depth_array.shape,
                    float(np.nanmin(depth_array)),
                    float(np.nanmax(depth_array)),
                )

            # Update preview JPEG (serve real JPEG bytes instead of PNG)
            try:
                buf = io.BytesIO()
                Image.fromarray(color_array_rgb).save(buf, format="JPEG", quality=80)
                peer.latest_jpeg = buf.getvalue()
            except Exception:
                peer.latest_jpeg = color_bytes

            # Convert RGB to BGR for agent (OpenCV compatibility)
            color_array_bgr = color_array_rgb[..., ::-1]

            frame = RGBDFrame(
                color=color_array_bgr,
                depth=depth_array,
                metadata={
                    "width": payload.get("width"),
                    "height": payload.get("height"),
                    "timestamp": payload.get("timestamp"),
                    "source": "webrtc",
                },
            )

            peer.baseline_session.state.update_rgbd(frame)

            envelope = MessageEnvelope(
                message_type="rgbd",
                payload=frame,
                raw=payload,
            )

            await self._dispatch_envelope(peer, envelope)

        except Exception as exc:
            logger.exception("Error processing RGBD payload: %s", exc)

    async def _handle_json_payload(
        self, peer: PeerSession, data: Dict[str, Any]
    ) -> None:
        """Process JSON packet received via DataChannel."""
        json_type = data.get("json_type")
        if not json_type:
            logger.warning("JSON payload missing json_type")
            return
        
        # Normalize json_type
        content = data.get("content")
        
        packet = JsonPacket(json_type=json_type, content=content)
        peer.baseline_session.state.update_json(packet)

        envelope = MessageEnvelope(
            message_type="json",
            payload=packet,
            raw=data,
        )

        # Send ACK
        self._send_datachannel_message(peer, {
            "type": "ack",
            "json_type": json_type,
        })

        await self._dispatch_envelope(peer, envelope)

    async def _dispatch_envelope(
        self, peer: PeerSession, envelope: MessageEnvelope
    ) -> None:
        """Dispatch envelope to baseline and send response."""
        try:
            response = await peer.baseline.handle_envelope(
                peer.baseline_session, envelope
            )
        except Exception as exc:
            logger.exception("Baseline error: %s", exc)
            self._send_datachannel_message(peer, {
                "type": "error",
                "message": f"Baseline error: {exc}",
            })
            return

        if response is None:
            return

        if not isinstance(response, BaselineResponse):
            logger.warning("Invalid baseline response type: %s", type(response))
            return

        # Send response messages
        for packet in response.messages:
            self._send_datachannel_message(peer, packet)

        if response.reset_state:
            peer.baseline_session.state.clear()

    def _send_datachannel_message(
        self, peer: PeerSession, message: Dict[str, Any]
    ) -> bool:
        """Send a message through the DataChannel."""
        if not peer.data_channel or peer.data_channel.readyState != "open":
            logger.warning("DataChannel not ready for session %s", peer.session_id)
            return False

        try:
            json_str = json.dumps(message)
            peer.data_channel.send(json_str)
            return True
        except Exception as exc:
            logger.warning("Failed to send DataChannel message: %s", exc)
            return False

    async def _consume_video_track(self, peer: PeerSession, track) -> None:
        """Consume video track frames (for MediaTrack-based RGBD)."""
        frame_count = 0
        try:
            while True:
                frame = await track.recv()
                frame_count += 1

                if frame_count % 30 == 0:
                    logger.debug(
                        "Session %s: Video track received %d frames",
                        peer.session_id,
                        frame_count,
                    )

                # Convert frame to numpy array
                array = frame.to_ndarray(format="bgr24")

                # Update preview
                peer.latest_jpeg = self._encode_jpeg(array)

        except Exception as exc:
            logger.info("Video track ended: %s (%d frames)", exc, frame_count)

    async def _cleanup_peer(self, session_id: str) -> None:
        """Clean up a peer session."""
        peer = self._peers.pop(session_id, None)
        if not peer:
            return

        logger.info("🧹 Cleaning up session %s", session_id)

        if peer.baseline and peer.baseline_session:
            try:
                await peer.baseline.on_session_end(peer.baseline_session)
            except Exception as exc:
                logger.warning("Error ending baseline session: %s", exc)

        if peer.pc:
            try:
                await peer.pc.close()
            except Exception as exc:
                logger.warning("Error closing peer connection: %s", exc)

    def _get_latest_jpeg(self) -> Optional[bytes]:
        """Get the latest JPEG frame from any active session."""
        for peer in self._peers.values():
            if peer.latest_jpeg:
                return peer.latest_jpeg
        return None

    @staticmethod
    def _encode_jpeg(frame: np.ndarray) -> Optional[bytes]:
        """Encode numpy array to JPEG bytes."""
        try:
            import cv2
            ok, buf = cv2.imencode(".jpg", frame)
            if ok:
                return buf.tobytes()
        except Exception:
            pass
        return None

    @staticmethod
    def _load_ice_servers() -> list[RTCIceServer]:
        """Load ICE servers from environment or use defaults."""
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
            except Exception as exc:
                logger.warning("Failed to parse ICE_SERVERS: %s", exc)

        # Defaults
        return [
            RTCIceServer(urls="stun:stun.l.google.com:19302"),
            RTCIceServer(urls="stun:stun1.l.google.com:19302"),
        ]

    async def start(self) -> None:
        """Start the WebRTC signaling server."""
        # Start preview server
        try:
            self._preview_server = start_preview_server(self._get_latest_jpeg)
            preview_url = "http://localhost:8080/viewer"
            print("\n" + "=" * 60)
            print("\033[1;36m" + "  📺 VIDEO PREVIEW AVAILABLE" + "\033[0m")
            print("=" * 60)
            print(f"\033[1;33m  ➜  {preview_url}\033[0m")
            print("=" * 60 + "\n")
        except Exception as exc:
            logger.warning("Preview server failed to start: %s", exc)

        runner = web.AppRunner(self._app)
        await runner.setup()
        site = web.TCPSite(runner, self._config.host, self._config.port)
        await site.start()

        host_ip = socket.gethostbyname(socket.gethostname())
        logger.info("✅ WebRTC Signaling Server running:")
        logger.info("  - Local: http://localhost:%s", self._config.port)
        logger.info("  - Network: http://%s:%s", host_ip, self._config.port)
        logger.info("  - All interfaces: http://%s:%s", self._config.host, self._config.port)
        logger.info("")
        logger.info("📡 Endpoints:")
        logger.info("  - POST /offer          - Submit WebRTC offer")
        logger.info("  - POST /ice/{session}  - Add ICE candidate")
        logger.info("  - GET  /status         - Server status")

        # Keep running
        await asyncio.Future()

    async def stop(self) -> None:
        """Stop the server and clean up."""
        if self._preview_server:
            httpd, _ = self._preview_server
            httpd.shutdown()

        # Close all peer connections
        for session_id in list(self._peers.keys()):
            await self._cleanup_peer(session_id)


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Run the FreeAskWorld WebRTC connector"
    )
    parser.add_argument(
        "--host", default="0.0.0.0", help="HTTP server bind host"
    )
    parser.add_argument(
        "--port", type=int, default=8766, help="HTTP server bind port"
    )
    parser.add_argument(
        "--baseline",
        default="closed_loop.freeaskworld_connector.simple_baseline:create_baseline",
        help="Baseline factory in the form 'pkg.module:function'",
    )
    parser.add_argument(
        "--persist-rgbd",
        default=None,
        help="Optional directory to persist RGBD frames for debugging",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable verbose logging for high-frequency events",
    )
    return parser.parse_args(argv)


async def async_main(argv: Optional[list[str]] = None) -> None:
    """Async entry point."""
    args = parse_args(argv)
    config = ServerConfig(
        host=args.host,
        port=args.port,
        persist_rgbd_dir=args.persist_rgbd,
        verbose=args.verbose,
    )

    baseline_factory = load_baseline_factory(args.baseline)
    server = WebRTCServer(baseline_factory, config)

    try:
        await server.start()
    except asyncio.CancelledError:
        pass
    finally:
        await server.stop()


def main(argv: Optional[list[str]] = None) -> None:
    """CLI entry point."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    )
    asyncio.run(async_main(argv))


if __name__ == "__main__":
    main()
