# Quick Reference - Configuration System

## 📁 配置文件位置
```
closed_loop/freeaskworld_connector/config/.env
```

## 🚀 快速命令

### 查看当前配置
```bash
cd /home/pengyh/workspace/FreeAskAgent
python -c "from closed_loop.freeaskworld_connector.config import config; config.print_summary()"
```

### 交互式配置
```bash
cd /home/pengyh/workspace/FreeAskAgent/closed_loop
./configure
```

### 编辑配置
```bash
cd /home/pengyh/workspace/FreeAskAgent/closed_loop/freeaskworld_connector/config
nano .env
```

### 启动服务器
```bash
cd /home/pengyh/workspace/FreeAskAgent
python -m closed_loop.freeaskworld_connector.server \
    --baseline closed_loop.baselines.agent_baseline:create_baseline
```

## 🎯 关键配置（最常修改的）

| 配置项 | 默认值 | 调整方式 |
|--------|--------|----------|
| **推理间隔** | 2.0s | 改 `MIN_RESPONSE_INTERVAL` |
| **视频帧率** | 30fps | 改 `WEBRTC_FRAMERATE` |
| **预览端口** | 8080 | 改 `PREVIEW_PORT` |

## 💡 常见场景

### 节省带宽
```bash
WEBRTC_FRAMERATE=15        # 从 30fps 降到 15fps
WEBRTC_BITRATE=1000000     # 从 2Mbps 降到 1Mbps
```

## 📊 性能对比

| 配置 | API 调用频率 | 成本倍数 |
|------|--------------|----------|
| 旧架构（每帧推理）| 30 次/秒 | 60x |
| High Performance | 1 次/秒 | 2x |
| **Balanced（默认）** | **0.5 次/秒** | **1x** |
| Low Cost | 0.2 次/秒 | 0.4x |

## 🔧 代码中使用配置

```python
# 在 freeaskworld_connector 内部
from .config import config

# 在其他地方
from closed_loop.freeaskworld_connector.config import config

# 访问配置
framerate = config.WEBRTC_FRAMERATE
port = config.PREVIEW_PORT
```

## 📚 完整文档

- **详细说明**: `freeaskworld_connector/config/CONFIG.md`
- **配置模板**: `freeaskworld_connector/config/.env.example`
- **配置 README**: `freeaskworld_connector/config/README.md`
- **系统概览**: `closed_loop/README_CONFIG.md`

## ✅ 验证配置

运行这个命令确认配置正常加载：
```bash
cd /home/pengyh/workspace/FreeAskAgent
python -c "from closed_loop.freeaskworld_connector.config import config; config.print_summary()"
```

应该看到：
```
✓ Loaded configuration from: .../config/.env
============================================================
📋 Configuration Summary
============================================================
  Inference Interval:  2.0s
  Video Framerate:     30 fps
  ...
============================================================
```
