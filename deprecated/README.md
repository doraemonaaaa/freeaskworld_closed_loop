# FreeAskWorld Connector

WebRTC bridge between the Unity simulator and Python baselines. It receives Unity payloads (JSON + RGBD), feeds them into a baseline, and streams navigation commands back to Unity over a DataChannel. A lightweight HTTP signaling server handles SDP/ICE exchange.

## Quick start

1) Create env and install deps (from repo root):
```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

2) Run signaling server (no tunnel, listen on all interfaces):
```bash
ENABLE_TUNNEL=0 PORT=8766 HOST=0.0.0.0 \
python -m closed_loop.freeaskworld_connector.webrtc_server \
    --host $HOST --port $PORT \
    --baseline closed_loop.freeaskworld_connector.simple_baseline:create_baseline
```

Or use the helper script (defaults ENABLE_TUNNEL=1; set to 0 unless you need Cloudflare):
```bash
ENABLE_TUNNEL=0 PORT=8766 HOST=0.0.0.0 \
BASELINE_SPEC="closed_loop.baselines.agent_baseline:create_baseline" \
bash closed_loop/baselines/start.bash
```

3) Unity side: point the client to `http://<server-ip>:8766` and ensure DataChannel label matches `control`. The preview page (JPEG) is at `http://<server-ip>:8080/viewer` once frames arrive.

> Note: run scripts from the repo root so `closed_loop` can be imported. If using the helper script and you don't need Cloudflare, set `ENABLE_TUNNEL=0`.

## Modes

- Pure WebRTC (recommended): HTTP signaling + DataChannel JSON + optional RGBD stream over DataChannel (no TURN baked in; set `ICE_SERVERS` env for TURN).
- Legacy WebSocket: `python -m closed_loop.freeaskworld_connector.server ...` (kept for compatibility).

## Baselines in this repo

- `simple_baseline`: smoke test; emits a `NavigationCommand` + `Step` once `Init`, `TransformData`, and RGBD are present. Now serialized with an internal asyncio lock so only one response is produced at a time.
- `agent_baseline`: uses AgentFlow solver; also serialized (one inference at a time) and runs on the latest state.

Implement your own by conforming to `ClosedLoopBaseline` and exporting `create_baseline()` in your module; start the server with `--baseline your.module:create_baseline`.

## Signaling endpoints

- `POST /offer` — submit SDP offer, receive `{session_id, sdp}` answer
- `POST /ice/{session_id}` — add ICE candidate
- `GET /status` — server + sessions
- `GET /status/{session_id}` — session detail
- `DELETE /session/{session_id}` — close a session

## DataChannel message shapes (Unity → Python)

- RGBD: `{ "type": "rgbd_stream", "payload": { color, depth, width, height, timestamp } }`
    - Server accepts `payload` (Unity sender default) or `content` fallback.
- JSON control: `{ "type": "json", "json_type": "Init|TransformData|Instruction|SimulationTime|Step|NavigationCommand", "content": {...} }`
    - Use `content` (not `payload`) for JSON bodies; direct types (`{ "type": "Init", ... }`) are also accepted and normalized.

Server ACKs JSON with `{ "type": "ack", "json_type": "..." }`.

## Unity integration (current scripts)

- `WebRTCManager` (signaling + peer + DataChannel)
    - Default URL `http://localhost:8766`, DataChannel label `control`, ICE `stun:stun.l.google.com:19302`, auto-connect on start.
    - Sends control via `SendControlMessage(json_type, payload)` → `{type:"json", json_type, content:payload}`.
    - Receives UTF-8 JSON on `OnTextMessage` (wired in `BenchmarkPlayer`).
- `RGBDJPEGSender` (RGBD push over DataChannel)
    - Hooks channel open/close; sends when open.
    - Downscales frames to `maxWidth/maxHeight` (default 320x240) if `limitResolution` true.
    - JPEG-encodes color/depth (default quality 70/50) to keep payload under SCTP limits.
    - Envelope: `{type:"rgbd_stream", payload:{width,height,color,depth,timestamp}}`.
- `BenchmarkPlayer` (VLN loop)
    - On start: subscribes to DataChannel messages; RGBD camera `onSensorUpdated` triggers `SendOnce()`.
    - Per step: sends `Init` (first step only), then `TransformData`, `Instruction`, `SimulationTime {time}`, `Step {IsStep:true}`.
    - Waits for server messages: `navigationcommand` (apply offsets, `IsStop`), `step` (to advance), optional `stop`.
    - Auto-step coroutine: wait channel open → send inputs → wait for `step` from server → sleep `stepSimTime` → `BenchmarkManager.Step()` → repeat.

## TURN / NAT traversal

Cloudflared (optional) only forwards HTTP signaling; media/DataChannel is P2P. For strict NAT/firewalls set:
```bash
export ICE_SERVERS='[{"urls":"turn:your-turn-server:3478","username":"user","credential":"pass"}]'
```
And configure the same TURN URL in Unity.

## Troubleshooting

- 502/Bad Gateway via Cloudflare: likely `ENABLE_TUNNEL=1` but the server failed to start or bound to a different host; try `ENABLE_TUNNEL=0` locally, or fix imports by running from repo root.
- `ModuleNotFoundError: closed_loop`: run commands from repo root, ensure venv is active.
- DataChannel open but no RGBD: check Unity `RGBDStreamSender` is enabled, channel is open, and payload size is small (use the compressed/downscaled sender).
- No baseline response: baseline waits for `Init` + `TransformData` + latest RGBD; only one inference at a time.

## Reference

- `closed_loop/freeaskworld_connector/webrtc_server.py` — HTTP signaling + DataChannel handler
- `closed_loop/freeaskworld_connector/simple_baseline.py` — minimal baseline (serialized)
- `closed_loop/baselines/agent_baseline.py` — AgentFlow-powered baseline (serialized)
- `closed_loop/baselines/start.bash` — launcher (optionally Cloudflare)
