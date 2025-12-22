#!/usr/bin/env bash
set -euo pipefail

PORT=${PORT:-8766}
HOST=${HOST:-0.0.0.0}
ENABLE_TUNNEL=${ENABLE_TUNNEL:-1}

BASELINENAME=${BASELINENAME:-simple_baseline}
BASELINE_SPEC=${BASELINE_SPEC:-closed_loop.baselines.${BASELINENAME}:create_baseline}

CLOUDFLARED_PID=""
if [[ "$ENABLE_TUNNEL" == "1" ]]; then
    python start_cloudflared.py "$PORT" &
    CLOUDFLARED_PID=$!
else
    echo "Skipping cloudflared tunnel; using LAN at ws://${HOST}:${PORT}"
fi

# The server.py path
python -m closed_loop.freeaskworld_connector.server \
    --host "$HOST" \
    --port "$PORT" \
    --baseline "$BASELINE_SPEC"

if [[ -n "$CLOUDFLARED_PID" ]]; then
    wait "$CLOUDFLARED_PID"
fi
