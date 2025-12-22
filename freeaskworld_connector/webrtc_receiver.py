import argparse
import asyncio
import json
import logging
import cv2
import numpy as np
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCIceCandidate
from aiortc.contrib.media import MediaBlackhole, MediaPlayer, MediaRecorder
import websockets

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("webrtc_receiver")

class WebRTCClient:
    def __init__(self, signaling_url):
        self.signaling_url = signaling_url
        self.pc = RTCPeerConnection()
        self.websocket = None
        self.tracks = []

        @self.pc.on("track")
        def on_track(track):
            logger.info(f"Track received: {track.kind}")
            if track.kind == "video":
                self.tracks.append(track)
                asyncio.ensure_future(self.consume_track(track))

        @self.pc.on("iceconnectionstatechange")
        async def on_iceconnectionstatechange():
            logger.info(f"ICE connection state is {self.pc.iceConnectionState}")
            if self.pc.iceConnectionState == "failed":
                await self.pc.close()

    async def consume_track(self, track):
        """
        Consume frames from a video track and display them using OpenCV.
        Assumes Track 0 is RGB and Track 1 is Depth (based on Unity sender order).
        """
        track_id = len(self.tracks) - 1
        window_name = f"Track {track_id} ({track.kind})"
        
        while True:
            try:
                frame = await track.recv()
                
                # Convert frame to numpy array (YUV420p to BGR)
                img = frame.to_ndarray(format="bgr24")
                
                # Display
                cv2.imshow(window_name, img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            except Exception as e:
                logger.warning(f"Track {track_id} ended: {e}")
                break

    async def connect(self):
        async with websockets.connect(self.signaling_url) as ws:
            self.websocket = ws
            logger.info(f"Connected to signaling server at {self.signaling_url}")

            async for message in ws:
                data = json.loads(message)
                
                # Unity sends wrapped JSON: { "type": "json", "json_type": "...", "content": { ... } }
                if data.get("type") == "json" and "content" in data:
                    content = data["content"]
                    msg_type = content.get("type")

                    if msg_type == "offer":
                        await self.handle_offer(content)
                    elif msg_type == "answer":
                        await self.handle_answer(content)
                    elif msg_type == "candidate":
                        await self.handle_candidate(content)

    async def handle_offer(self, offer):
        logger.info("Received Offer")
        await self.pc.setRemoteDescription(RTCSessionDescription(sdp=offer["sdp"], type=offer["type"]))
        
        answer = await self.pc.createAnswer()
        await self.pc.setLocalDescription(answer)
        
        response = {
            "type": "answer",
            "sdp": self.pc.localDescription.sdp
        }
        await self.send_signaling_message(response)
        logger.info("Sent Answer")

    async def handle_answer(self, answer):
        logger.info("Received Answer")
        await self.pc.setRemoteDescription(RTCSessionDescription(sdp=answer["sdp"], type=answer["type"]))

    async def handle_candidate(self, candidate):
        logger.info(f"Received Candidate: {candidate['candidate']}")
        ice_candidate = RTCIceCandidate(
            candidate=candidate["candidate"],
            sdpMid=candidate["sdpMid"],
            sdpMLineIndex=candidate["sdpMLineIndex"]
        )
        await self.pc.addIceCandidate(ice_candidate)

    async def send_signaling_message(self, message):
        # Wrap message to match Unity's expected format
        payload = {
            "type": "json",
            "json_type": "Simulator.ClosedLoop.WebRTCSignalingMessage", # Adjust namespace if needed
            "content": message
        }
        await self.websocket.send(json.dumps(payload))

async def main():
    parser = argparse.ArgumentParser(description="WebRTC Receiver for Unity RGBD Stream")
    parser.add_argument("--url", default="ws://127.0.0.1:8765", help="Signaling WebSocket URL")
    args = parser.parse_args()

    client = WebRTCClient(args.url)
    await client.connect()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
