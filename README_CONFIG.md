# FreeAskAgent Closed-Loop System

## 目录结构

```
closed_loop/
├── configure                          # 配置快捷脚本
├── baselines/
│   └── agent_baseline.py             # GPT-4o 智能体 baseline
└── freeaskworld_connector/
    ├── config/                        # ✨ 配置目录
    │   ├── __init__.py               # 配置加载模块
    │   ├── .env                      # 实际配置（已生效）
    │   ├── .env.example              # 配置模板
    │   ├── CONFIG.md                 # 完整文档
    │   ├── README.md                 # 配置说明
    │   └── configure.sh              # 交互式配置脚本
    ├── server.py                     # WebSocket 服务器
    ├── webrtc_bridge.py              # WebRTC 视频桥接
    ├── preview_http.py               # HTTP 预览服务器
    ├── simple_baseline.py            # 简单测试 baseline
    └── framework.py                  # 框架基类
```

## 配置系统

### 快速开始

**1. 使用默认配置（推荐）**

```bash
cd /home/pengyh/workspace/FreeAskAgent
python -m closed_loop.freeaskworld_connector.server \
    --baseline closed_loop.baselines.agent_baseline:create_baseline
```

**2. 交互式配置**
```bash
cd /home/pengyh/workspace/FreeAskAgent/closed_loop
./configure
# 或
cd freeaskworld_connector/config
./configure.sh
```

**3. 手动编辑配置**
```bash
cd /home/pengyh/workspace/FreeAskAgent/closed_loop/freeaskworld_connector/config
nano .env
```

### 配置文件位置

所有配置都在：
```
closed_loop/freeaskworld_connector/config/.env
```

### 在代码中使用配置

```python
# 在 freeaskworld_connector 内部使用相对导入
from .config import config

framerate = config.WEBRTC_FRAMERATE

# 在 baselines 或其他地方使用完整路径
from closed_loop.freeaskworld_connector.config import config
```

### 关键配置项

| 配置项 | 默认值 | 说明 |
|--------|--------|------|
| `WEBRTC_FRAMERATE` | 30 | 视频帧率 (fps) |
| `WEBRTC_BITRATE` | 2000000 | 视频码率 (bps) |
| `JPEG_UPDATE_INTERVAL` | 0.1 | 预览更新间隔（秒）|
| `PREVIEW_PORT` | 8080 | HTTP 预览端口 |
| `WS_PORT` | 8766 | WebSocket 端口 |

详细说明见：`freeaskworld_connector/config/CONFIG.md`

## 架构说明

### 新架构（已实现）

```
Unity 视频流 (30fps)
    ↓
WebRTC Bridge → 持续更新帧缓存
    ↓
Baseline (独立推理循环)
    ↓
按配置间隔 → 取最新帧 → GPT-4o 推理 → 返回导航指令
```

**优势**：
- ✅ 帧接收和推理解耦
- ✅ 推理频率可控（2秒间隔 = 0.5次/秒）
- ✅ API 成本降低 60 倍
- ✅ 视频流保持 30fps 流畅

### 对比旧架构

**旧架构问题** ❌：
```
帧到达 → 立即推理 → 30fps = 30次推理/秒 → 成本爆炸
```

**新架构解决** ✅：
```
帧到达（持续）→ 缓存
推理（独立）→ 串行执行，按间隔触发
```

## 配置预设

运行 `./configure` 选择：

1. **💰 Low Cost** - API 调用 ~0.2次/秒
   - 适合：长时间测试，演示
   - 配置：5s 间隔，15fps

2. **⚡ Balanced** - API 调用 ~0.5次/秒 [默认]
   - 适合：日常开发，本地测试
   - 配置：2s 间隔，30fps

3. **🚀 High Performance** - API 调用 ~1次/秒
   - 适合：需要快速响应的场景
   - 配置：1s 间隔，30fps
   - ⚠️ 注意：API 成本更高

4. **🌐 Remote Network** - 网络友好
   - 适合：远程连接，较慢网络
   - 配置：2s 间隔，20fps，低码率

## 监控和调试

### 查看配置

```bash
cd /home/pengyh/workspace/FreeAskAgent
python -c "from closed_loop.freeaskworld_connector.config import config; config.print_summary()"
```

输出示例：
```
============================================================
📋 Configuration Summary
============================================================
  Inference Interval:  2.0s
  Video Framerate:     30 fps
  Video Bitrate:       2.0 Mbps
  Preview Update:      0.1s (10 fps)
  Preview Port:        8080
  WebSocket Port:      8766
  ICE Policy:          all
  LLM Engine:          gpt-4o
  Fast Mode:           True
  Log Level:           INFO
============================================================
```

### 运行时日志

启动服务器后会看到：
```
✓ Loaded configuration from: .../config/.env
[AgentBaseline] Started inference loop (min_interval=2.0s)
Track 0: 30.2 fps (61 frames in 2.0s)
[AgentBaseline] Starting inference (last inference: 2.15s ago)
```

### Web 预览

启动后自动显示：
```
============================================================
  📺 VIDEO PREVIEW AVAILABLE
============================================================
  ➜  http://localhost:8080/viewer
     Snapshot: http://localhost:8080/frame
     MJPEG:    http://localhost:8080/mjpeg
============================================================
```

## 性能优化建议

### 降低 API 成本
```bash
# 编辑 config/.env
WEBRTC_FRAMERATE=15          # 降低帧率
```

### 提升视频质量
```bash
WEBRTC_FRAMERATE=30          # 流畅视频
WEBRTC_BITRATE=3000000       # 更高码率
JPEG_UPDATE_INTERVAL=0.05    # 更快预览
```

### 节省 CPU
```bash
JPEG_UPDATE_INTERVAL=0.2     # 降低预览更新频率
```

## 常见问题

**Q: 配置修改后不生效？**
A: 需要重启服务器（Ctrl+C 然后重新运行）

**Q: 找不到配置文件？**
A: 配置文件在 `closed_loop/freeaskworld_connector/config/.env`

**Q: 想要临时覆盖配置？**
A: 使用环境变量：
```bash
WEBRTC_FRAMERATE=30 python -m closed_loop.freeaskworld_connector.server ...
```

**Q: 视频卡顿？**
A: 检查 `WEBRTC_FRAMERATE` 和日志中的实际 fps

## 完整文档

- **配置详细说明**：[config/CONFIG.md](freeaskworld_connector/config/CONFIG.md)
- **配置目录 README**：[config/README.md](freeaskworld_connector/config/README.md)
- **配置模板**：[config/.env.example](freeaskworld_connector/config/.env.example)

## 快速测试

```bash
# 1. 进入项目目录
cd /home/pengyh/workspace/FreeAskAgent

# 2. 查看当前配置
python -c "from closed_loop.freeaskworld_connector.config import config; config.print_summary()"

# 3. 启动服务器
python -m closed_loop.freeaskworld_connector.server \
    --baseline closed_loop.baselines.agent_baseline:create_baseline

# 4. 打开预览（另一个终端）
xdg-open http://localhost:8080/viewer

# 5. 启动 Unity 客户端连接 ws://localhost:8766
```

搞定！🎉
