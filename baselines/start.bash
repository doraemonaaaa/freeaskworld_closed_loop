#!/usr/bin/env bash
set -euo pipefail

PORT=${PORT:-8766}
HOST=${HOST:-0.0.0.0}
ENABLE_TUNNEL=${ENABLE_TUNNEL:-0}
VERBOSE=${VERBOSE:-0}

BASELINENAME=${BASELINENAME:-agent_baseline}
BASELINE_SPEC=${BASELINE_SPEC:-closed_loop.baselines.${BASELINENAME}:create_baseline}

CLOUDFLARED_PID=""
if [[ "$ENABLE_TUNNEL" == "1" ]]; then
    echo "🌐 Starting cloudflared tunnel for HTTP signaling..."
    python closed_loop/freeaskworld_connector/start_cloudflared.py "$PORT" &
    CLOUDFLARED_PID=$!
else
    echo "📡 No tunnel; using LAN at http://${HOST}:${PORT}"
fi

echo "🚀 Starting WebRTC server with ${BASELINENAME}..."
if [[ "$VERBOSE" == "1" ]]; then
    VERBOSE_FLAG="--verbose"
else
    VERBOSE_FLAG=""
fi
python -m closed_loop.freeaskworld_connector.webrtc_server \
    --host "$HOST" \
    --port "$PORT" \
    --baseline "$BASELINE_SPEC" \
    $VERBOSE_FLAG

if [[ -n "$CLOUDFLARED_PID" ]]; then
    wait "$CLOUDFLARED_PID"
fi
