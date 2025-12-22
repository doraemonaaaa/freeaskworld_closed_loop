#!/bin/bash
# Quick configuration script for FreeAskAgent closed-loop

set -e

# This script is in freeaskworld_connector/config/
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "🔧 FreeAskAgent Closed-Loop Configuration"
echo "=========================================="
echo "Config directory: $SCRIPT_DIR"
echo ""

# Check if .env exists
if [ -f ".env" ]; then
    echo "⚠️  .env file already exists!"
    read -p "Overwrite? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Keeping existing .env file"
        exit 0
    fi
fi

echo "Select configuration preset:"
echo ""
echo "1) 💰 Low Cost (5s inference interval, 15fps)"
echo "2) ⚡ Balanced (2s inference interval, 30fps) [Default]"
echo "3) 🚀 High Performance (1s inference interval, 30fps)"
echo "4) 🌐 Remote Network (2s interval, 20fps, lower bitrate)"
echo "5) 📝 Custom (edit manually)"
echo ""
read -p "Choice [1-5] (default: 2): " choice
choice=${choice:-2}

case $choice in
    1)
        echo "Setting up Low Cost configuration..."
        cat > .env << EOF
# Low Cost Configuration
MIN_RESPONSE_INTERVAL=5.0
WEBRTC_FRAMERATE=15
WEBRTC_BITRATE=1000000
JPEG_UPDATE_INTERVAL=0.2
PREVIEW_PORT=8080
PREVIEW_AUTO_OPEN=0
WS_HOST=0.0.0.0
WS_PORT=8766
ICE_TRANSPORT_POLICY=all
ICE_SERVERS=[{"urls":"stun:stun.l.google.com:19302"}]
LLM_ENGINE_NAME=gpt-4o
FAST_MODE=true
LOG_LEVEL=INFO
EOF
        echo "✅ Low Cost preset applied (API calls: ~0.2/sec)"
        ;;
    2)
        echo "Setting up Balanced configuration..."
        cat > .env << EOF
# Balanced Configuration (Default)
MIN_RESPONSE_INTERVAL=2.0
WEBRTC_FRAMERATE=30
WEBRTC_BITRATE=2000000
JPEG_UPDATE_INTERVAL=0.1
PREVIEW_PORT=8080
PREVIEW_AUTO_OPEN=0
WS_HOST=0.0.0.0
WS_PORT=8766
ICE_TRANSPORT_POLICY=all
ICE_SERVERS=[{"urls":"stun:stun.l.google.com:19302"}]
LLM_ENGINE_NAME=gpt-4o
FAST_MODE=true
LOG_LEVEL=INFO
EOF
        echo "✅ Balanced preset applied (API calls: ~0.5/sec)"
        ;;
    3)
        echo "Setting up High Performance configuration..."
        cat > .env << EOF
# High Performance Configuration
MIN_RESPONSE_INTERVAL=1.0
WEBRTC_FRAMERATE=30
WEBRTC_BITRATE=2000000
JPEG_UPDATE_INTERVAL=0.05
PREVIEW_PORT=8080
PREVIEW_AUTO_OPEN=0
WS_HOST=0.0.0.0
WS_PORT=8766
ICE_TRANSPORT_POLICY=all
ICE_SERVERS=[{"urls":"stun:stun.l.google.com:19302"}]
LLM_ENGINE_NAME=gpt-4o
FAST_MODE=true
LOG_LEVEL=INFO
EOF
        echo "✅ High Performance preset applied (API calls: ~1/sec)"
        echo "⚠️  Warning: Higher API costs!"
        ;;
    4)
        echo "Setting up Remote Network configuration..."
        cat > .env << EOF
# Remote Network Configuration
MIN_RESPONSE_INTERVAL=2.0
WEBRTC_FRAMERATE=20
WEBRTC_BITRATE=1000000
JPEG_UPDATE_INTERVAL=0.2
PREVIEW_PORT=8080
PREVIEW_AUTO_OPEN=0
WS_HOST=0.0.0.0
WS_PORT=8766
ICE_TRANSPORT_POLICY=all
ICE_SERVERS=[{"urls":"stun:stun.l.google.com:19302"}]
LLM_ENGINE_NAME=gpt-4o
FAST_MODE=true
LOG_LEVEL=INFO
EOF
        echo "✅ Remote Network preset applied"
        echo "ℹ️  Note: You may need to add TURN servers for remote access"
        ;;
    5)
        echo "Copying example configuration..."
        cp .env.example .env
        echo "✅ Created .env from template"
        echo "📝 Edit .env to customize settings"
        exit 0
        ;;
    *)
        echo "❌ Invalid choice, using default (Balanced)"
        choice=2
        ;;
esac

echo ""
echo "Configuration applied!"
echo ""
echo "📋 Current settings:"
grep -E "^[A-Z_]+=" .env | while read line; do
    key=$(echo "$line" | cut -d= -f1)
    value=$(echo "$line" | cut -d= -f2-)
    printf "  %-25s = %s\n" "$key" "$value"
done

echo ""
echo "Next steps:"
echo "  1. Review CONFIG.md for detailed documentation"
echo "  2. Start the server:"
echo "     cd /home/pengyh/workspace/FreeAskAgent"
echo "     python -m closed_loop.freeaskworld_connector.server --baseline closed_loop.baselines.agent_baseline:create_baseline"
echo ""
echo "🎉 Configuration complete!"
