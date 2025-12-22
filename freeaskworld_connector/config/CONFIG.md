# Closed-Loop Configuration Guide

## Quick Start

1. **复制示例配置**：
   ```bash
   cp .env.example .env
   ```

2. **编辑 `.env` 文件**，根据你的需求调整参数

3. **启动服务**：
   ```bash
   cd /home/pengyh/workspace/FreeAskAgent
   python -m closed_loop.freeaskworld_connector.server --baseline closed_loop.baselines.agent_baseline:create_baseline
   ```

## 关键配置说明

### 🎯 推理控制

```bash
# 推理间隔（秒）- 控制 API 调用频率
MIN_RESPONSE_INTERVAL=2.0
```

**影响**：
- **值越大** → API 调用越少，成本越低，但响应越慢
- **值越小** → 响应越快，但 API 成本越高

**推荐设置**：
- GPT-4o 智能体：`2.0` 秒（每秒 0.5 次调用）
- 简单 baseline：`0.5` 秒（每秒 2 次调用）
- 测试/演示：`5.0` 秒（降低成本）

### 📹 视频流配置

```bash
# 帧率（fps）
WEBRTC_FRAMERATE=30

# 码率（bps）
WEBRTC_BITRATE=2000000
```

**影响**：
- **FRAMERATE** 控制视频流畅度
  - `30`: 流畅（推荐本地网络）
  - `15-20`: 节省带宽
  - `10`: 最低可用帧率
  
- **BITRATE** 控制视频质量
  - `2000000` (2 Mbps): 高质量（推荐本地）
  - `1000000` (1 Mbps): 中等质量
  - `500000` (0.5 Mbps): 低质量但省带宽

### 🖼️ Web 预览配置

```bash
# JPEG 编码间隔（秒）
JPEG_UPDATE_INTERVAL=0.1

# 预览端口
PREVIEW_PORT=8080

# 自动打开浏览器
PREVIEW_AUTO_OPEN=0
```

**影响**：
- **JPEG_UPDATE_INTERVAL**：
  - `0.1` 秒 = 10 fps 预览（推荐，平衡质量和 CPU）
  - `0.05` 秒 = 20 fps（更流畅但占用更多 CPU）
  - `0.2` 秒 = 5 fps（节省 CPU）

- **PREVIEW_AUTO_OPEN**：
  - `1`: 启动时自动在浏览器打开预览
  - `0`: 手动打开（推荐）

### 🌐 WebRTC ICE 配置

```bash
# ICE 传输策略
ICE_TRANSPORT_POLICY=all

# STUN/TURN 服务器
ICE_SERVERS='[{"urls":"stun:stun.l.google.com:19302"}]'
```

**本地测试**（推荐）：
```bash
ICE_TRANSPORT_POLICY=all
ICE_SERVERS='[{"urls":"stun:stun.l.google.com:19302"}]'
```

**远程连接**（需要 TURN 服务器）：
```bash
ICE_TRANSPORT_POLICY=all
ICE_SERVERS='[
  {"urls":"stun:stun.l.google.com:19302"},
  {"urls":"turn:your-turn-server.com:3478","username":"user","credential":"pass"}
]'
```

## 性能调优

### 场景 1：降低 API 成本

```bash
MIN_RESPONSE_INTERVAL=5.0        # 每 5 秒推理一次
WEBRTC_FRAMERATE=15              # 降低帧率节省带宽
JPEG_UPDATE_INTERVAL=0.2         # 降低预览更新率
```

**效果**：API 调用从 30 次/秒 → 0.2 次/秒（降低 150 倍）

### 场景 2：最佳体验（本地测试）

```bash
MIN_RESPONSE_INTERVAL=1.0        # 快速响应
WEBRTC_FRAMERATE=30              # 流畅视频
WEBRTC_BITRATE=2000000           # 高质量
JPEG_UPDATE_INTERVAL=0.1         # 流畅预览
```

**效果**：流畅视频 + 快速响应，适合本地开发

### 场景 3：远程网络

```bash
MIN_RESPONSE_INTERVAL=2.0        # 平衡响应
WEBRTC_FRAMERATE=20              # 适中帧率
WEBRTC_BITRATE=1000000           # 降低带宽
JPEG_UPDATE_INTERVAL=0.2         # 降低网络负载
```

**效果**：适应较慢网络连接

## 架构说明

### 新架构（已实现）

```
帧到达 (30fps) → 更新缓存 → [不触发推理]
                          ↓
推理循环 → 检查间隔 → 取最新帧 → 运行推理 → 发送结果 → 等待
```

**关键优势**：
1. ✅ 帧接收和推理解耦
2. ✅ 推理频率独立控制
3. ✅ 不会因推理阻塞帧接收
4. ✅ API 成本可控

### 对比旧架构

**旧架构问题**：
```
帧到达 → 立即触发推理 → 阻塞等待 → 下一帧
```
- ❌ 30 fps = 30 次推理/秒
- ❌ API 成本极高
- ❌ 推理阻塞帧接收

**新架构解决方案**：
```
帧到达 (持续) → 缓存
推理 (独立)    → 按间隔执行
```
- ✅ 帧率和推理频率独立
- ✅ 成本可控（2 秒间隔 = 0.5 次/秒）
- ✅ 不阻塞

## 监控和调试

### 查看日志

启动时会显示关键信息：
```
[AgentBaseline] Started inference loop (min_interval=2.0s)
Track 0: 30.2 fps (61 frames in 2.0s)
[AgentBaseline] Starting inference (last inference: 2.15s ago)
```

### 重要指标

- **Track fps**：视频流帧率（应接近 WEBRTC_FRAMERATE）
- **last inference**：距离上次推理的时间（应 ≥ MIN_RESPONSE_INTERVAL）
- **frames received vs sent**：接收帧数 vs 发送给 baseline 的帧数

### 调试选项

```bash
# 启用详细日志
LOG_LEVEL=DEBUG

# 保存 RGBD 帧到文件
PERSIST_RGBD_DIR=/tmp/rgbd_frames

# 启动时自动打开预览
PREVIEW_AUTO_OPEN=1
```

## 常见问题

### Q: 推理太慢/成本太高？

**A:** 增加 `MIN_RESPONSE_INTERVAL`：
```bash
MIN_RESPONSE_INTERVAL=5.0  # 或更高
```

### Q: 视频卡顿？

**A:** 检查这些配置：
```bash
WEBRTC_FRAMERATE=30          # 确保足够高
WEBRTC_BITRATE=2000000       # 确保带宽足够
JPEG_UPDATE_INTERVAL=0.1     # 预览更新间隔
```

查看日志中的 "Track fps" 是否达到目标值。

### Q: 预览占用 CPU 太高？

**A:** 降低预览更新频率：
```bash
JPEG_UPDATE_INTERVAL=0.2     # 从 10fps 降到 5fps
```

### Q: WebRTC 连接失败？

**A:** 检查 ICE 配置：
```bash
# 本地测试使用
ICE_TRANSPORT_POLICY=all

# 查看日志中的 ICE 状态
# 应该看到 "ICE connection established"
```

## 文件位置

- **配置模板**：`closed_loop/.env.example`
- **实际配置**：`closed_loop/.env`（需要自己创建）
- **Agent baseline**：`closed_loop/baselines/agent_baseline.py`
- **Simple baseline**：`closed_loop/freeaskworld_connector/simple_baseline.py`
- **WebRTC bridge**：`closed_loop/freeaskworld_connector/webrtc_bridge.py`

## 更新配置后

修改 `.env` 文件后，**重启服务**即可生效：
```bash
# Ctrl+C 停止
# 重新启动
python -m closed_loop.freeaskworld_connector.server --baseline closed_loop.baselines.agent_baseline:create_baseline
```
