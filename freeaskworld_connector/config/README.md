# FreeAskWorld Connector Configuration

This directory contains all configuration files for the closed-loop connector.

## Files

- **`.env`** - Active configuration (loaded at runtime)
- **`.env.example`** - Configuration template with all available options
- **`CONFIG.md`** - Detailed documentation for all settings
- **`configure.sh`** - Interactive configuration script
- **`__init__.py`** - Python module for loading configuration

## Quick Start

### Option 1: Use defaults (recommended)
The `.env` file already contains sensible defaults. Just start the server:

```bash
cd /home/pengyh/workspace/FreeAskAgent
python -m closed_loop.freeaskworld_connector.server \
    --baseline closed_loop.baselines.agent_baseline:create_baseline
```

### Option 2: Interactive configuration
Run the configuration script to choose a preset:

```bash
cd /home/pengyh/workspace/FreeAskAgent/closed_loop/freeaskworld_connector/config
./configure.sh
```

Or from the closed_loop directory:
```bash
cd /home/pengyh/workspace/FreeAskAgent/closed_loop
./configure
```

### Option 3: Manual configuration
Edit `.env` directly:

```bash
cd /home/pengyh/workspace/FreeAskAgent/closed_loop/freeaskworld_connector/config
cp .env.example .env  # if needed
nano .env             # or your favorite editor
```

## Using Config in Code

The configuration is automatically loaded when you import the config module:

```python
from closed_loop.freeaskworld_connector.config import config

# Access settings
framerate = config.WEBRTC_FRAMERATE          # 30
interval = config.MIN_RESPONSE_INTERVAL      # 2.0
port = config.PREVIEW_PORT                   # 8080

# Print summary
config.print_summary()
```

All code in `freeaskworld_connector/` uses relative imports:
```python
from .config import config
```

## Key Settings

### Inference Control
- `MIN_RESPONSE_INTERVAL` - Seconds between inference calls (default: 2.0)
  - Controls API cost (higher = fewer calls)
  - Recommended: 2.0 for GPT-4o, 0.5 for simple baselines

### Video Streaming
- `WEBRTC_FRAMERATE` - Video fps (default: 30)
- `WEBRTC_BITRATE` - Video bitrate in bps (default: 2000000 = 2 Mbps)

### Preview Server
- `JPEG_UPDATE_INTERVAL` - Preview update interval in seconds (default: 0.1 = 10fps)
- `PREVIEW_PORT` - HTTP server port (default: 8080)
- `PREVIEW_AUTO_OPEN` - Auto-open browser (default: 0 = no)

### WebSocket
- `WS_HOST` - Server bind address (default: 0.0.0.0)
- `WS_PORT` - WebSocket port (default: 8766)

See `CONFIG.md` for complete documentation.

## Configuration Presets

The interactive script offers these presets:

1. **💰 Low Cost** - Minimize API costs
   - 5s inference interval (~0.2 calls/sec)
   - 15fps video
   
2. **⚡ Balanced** - Recommended for most use cases
   - 2s inference interval (~0.5 calls/sec)
   - 30fps video
   
3. **🚀 High Performance** - Fast response
   - 1s inference interval (~1 call/sec)
   - 30fps video
   - ⚠️ Higher API costs
   
4. **🌐 Remote Network** - Network-friendly
   - 2s inference interval
   - 20fps, lower bitrate

## Troubleshooting

### Config not loading?
Check that you're in the right directory and `.env` exists:
```bash
ls -la /home/pengyh/workspace/FreeAskAgent/closed_loop/freeaskworld_connector/config/.env
```

### Want to reset to defaults?
```bash
cd /home/pengyh/workspace/FreeAskAgent/closed_loop/freeaskworld_connector/config
cp .env.example .env
```

### Need environment-specific settings?
You can override any setting via environment variables:
```bash
export MIN_RESPONSE_INTERVAL=5.0
export WEBRTC_FRAMERATE=15
python -m closed_loop.freeaskworld_connector.server ...
```

Environment variables take precedence over `.env` file.

## Architecture

The new configuration system:
- ✅ Centralized: All settings in one place
- ✅ Type-safe: Config module provides typed access
- ✅ Relative imports: Easy to use within freeaskworld_connector
- ✅ Fallback values: Works even without .env file
- ✅ Documentation: CONFIG.md has detailed explanations

## See Also

- **CONFIG.md** - Complete configuration documentation
- **../** - Go back to freeaskworld_connector directory
- **../../** - Go back to closed_loop directory
