# FreeAskWorld Connector

The FreeAskWorld connector provides the communication bridge between the Unity
simulator (client) and Python baselines hosted inside this repository. The
package translates incoming simulator payloads into structured objects, feeds
them into a pluggable baseline, and streams the resulting navigation commands
back to Unity.

## Architecture

### Pure WebRTC Mode (Recommended)
- **HTTP Signaling Server**: Handles SDP/ICE exchange via REST endpoints
- **WebRTC DataChannel**: Bidirectional JSON messaging for control commands
- **WebRTC MediaTracks**: (Optional) Real-time RGBD video streaming

### Legacy WebSocket Mode
- WebSocket server with WebRTC overlay for media tracks
- Maintained for backward compatibility

## Highlights
- Unified `ClosedLoopBaseline` protocol for implementing baseline policies.
- Session-scoped state containers that cache RGB-D frames and JSON packets.
- Drop-in smoke-test baseline so you can validate the communication pipeline in isolation.
- Optional Cloudflare tunnel helper for quick NAT/firewall traversal.

## Requirements
- Python 3.10+
- Repository dependencies: `pip install -r requirements.txt`
- Optional: `cloudflared` binary if you plan to expose the server publicly

You can create a dedicated environment first:

```bash
conda create -n freeaskworld python=3.10
conda activate freeaskworld
pip install -r requirements.txt
```

## Running the server

### Pure WebRTC Server (Recommended)

Launch the HTTP-based WebRTC signaling server:

```bash
python -m closed_loop.freeaskworld_connector.webrtc_server \
  --host 0.0.0.0 \
  --port 8766 \
  --baseline closed_loop.freeaskworld_connector.simple_baseline:create_baseline
```

From Unity, use `HTTPSignalingClient` to connect to `http://<server-ip>:8766`.

#### HTTP Endpoints:
- `POST /offer` - Submit WebRTC offer, receive answer
- `POST /ice/{session_id}` - Add ICE candidate
- `GET /status` - Server status and active sessions
- `DELETE /session/{session_id}` - Close a session

### Legacy WebSocket Server

For backward compatibility:

```bash
USE_WEBRTC=0 bash start.bash
# or directly:
python -m closed_loop.freeaskworld_connector.server \
  --host 0.0.0.0 \
  --port 8766 \
  --baseline closed_loop.freeaskworld_connector.simple_baseline:create_baseline
```

From Unity, connect to `ws://<server-ip>:8766`.

### Helper script

`start.bash` wraps both the server and an optional Cloudflare tunnel. Override
defaults through environment variables:

```bash
PORT=8766 \
HOST=0.0.0.0 \
USE_WEBRTC=1 \
BASELINE_SPEC="closed_loop.freeaskworld_connector.simple_baseline:create_baseline" \
bash start.bash
```

The script prints the generated `*.trycloudflare.com` URL once the tunnel is
ready.

## NAT穿透说明

### Cloudflared的作用
Cloudflared只转发**HTTP信令流量**（SDP/ICE交换），不处理WebRTC数据流。

```
连接建立流程:
1. Unity -> Cloudflared -> Python: HTTP POST /offer (SDP)
2. Python -> Cloudflared -> Unity: HTTP Response (SDP Answer)
3. Unity <-> Cloudflared <-> Python: HTTP POST /ice (ICE candidates)
4. Unity <======= WebRTC P2P =======> Python: DataChannel直连
```

### 网络场景

| 场景 | 信令 | 数据传输 | 是否可行 |
|------|------|----------|----------|
| 同一局域网 | LAN HTTP | WebRTC P2P | ✅ |
| Unity在NAT后，Python有公网IP | Cloudflared | WebRTC P2P | ✅ |
| 双方都在NAT后（同运营商） | Cloudflared | WebRTC via STUN | ✅ |
| 双方都在严格NAT后 | Cloudflared | 需要TURN服务器 | ⚠️ |

### TURN服务器配置
如果P2P连接失败（严格NAT/防火墙），需要配置TURN服务器：

```bash
# Python端 (环境变量)
export ICE_SERVERS='[{"urls":"turn:your-turn-server:3478","username":"user","credential":"pass"}]'

# Unity端 (Inspector)
ICE Servers: turn:your-turn-server:3478
```

免费TURN服务器（测试用）：
- `turn:openrelay.metered.ca:80` (用户名/密码: openrelayproject)

## Unity Integration

### 单一组件设计
现在只需要一个组件：**WebRTCManager**

它内置了：
- HTTP信令（用于SDP/ICE交换）
- WebRTC PeerConnection管理
- DataChannel双向通信
- RGBD数据流发送
- 自动重连功能

### 快速设置
1. 在GameObject上添加 `WebRTCManager` 组件
2. 设置服务器URL（例如 `http://localhost:8766`）
3. 启用 `Auto Connect` 或手动调用 `Connect()`

### Inspector配置
```
[Signaling Server]
- Server URL: http://localhost:8766
- Auto Connect: ✓
- Reconnect Delay: 5s
- Max Reconnect Attempts: 3

[WebRTC Configuration]
- ICE Servers: stun:stun.l.google.com:19302

[Data Channel]
- Data Channel Label: control
- Log Debug: ✓

[RGBD Stream]
- RGBD Camera: (your RGBDCamera reference)
- Auto Send RGBD: ✓
- RGBD Send Interval: 0.1 (10 FPS)
```

## Optional Cloudflare setup

```bash
wget -q https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-amd64.deb
sudo dpkg -i cloudflared-linux-amd64.deb
cloudflared tunnel --url http://localhost:8766
```

## Message flow (WebRTC Mode)
1. Unity sends HTTP POST to `/offer` with SDP offer
2. Server responds with SDP answer and session ID
3. Both sides exchange ICE candidates via `/ice/{session_id}`
4. WebRTC DataChannel opens for bidirectional communication
5. Unity streams RGBD frames and JSON packets via DataChannel
6. Server deserializes messages and updates `SessionState`
7. Baseline processes envelopes and emits navigation commands
8. Commands sent back through DataChannel
9. Unity applies commands and advances simulation

## Implementing a baseline

Create a class that satisfies the `ClosedLoopBaseline` protocol and expose a factory named `create_baseline`:

```python
from closed_loop.freeaskworld_connector.framework import (
    BaselineResponse, BaselineSession, ClosedLoopBaseline, MessageEnvelope,
)


class MyBaseline(ClosedLoopBaseline):
    async def on_session_start(self, session: BaselineSession) -> None:
        ...

    async def on_session_end(self, session: BaselineSession) -> None:
        ...

    async def handle_envelope(
        self, session: BaselineSession, envelope: MessageEnvelope
    ) -> BaselineResponse | None:
        ...


def create_baseline() -> ClosedLoopBaseline:
    return MyBaseline()
```

Run the server with `--baseline my_package.module:create_baseline` (or set the `BASELINE_SPEC` environment variable) to activate your implementation.

## Troubleshooting
- **Connection refused**: ensure Unity targets the host/port configured on the server.
- **No responses**: verify that the baseline only returns a `BaselineResponse` once all necessary inputs (RGB-D, instruction, transform) are available.
- **Cloudflare errors**: make sure the `cloudflared` binary is on the `PATH` and the requested port is free.

## Reference files
- `server.py`: websocket event loop and baseline dispatcher
- `framework.py`: core data structures and interfaces
- `handlers.py`: payload decoding utilities
- `simple_baseline.py`: minimal baseline example
- `start_cloudflared.py`: Cloudflare tunnel launcher
