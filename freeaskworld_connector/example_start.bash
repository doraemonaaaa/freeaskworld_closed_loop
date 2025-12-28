#!/usr/bin/env bash
set -euo pipefail

MODE=${MODE:-shared} # local | public | shared
PORT=${PORT:-8766}
HOST=${HOST:-0.0.0.0}
ENABLE_TUNNEL=${ENABLE_TUNNEL:-0}
VERBOSE=${VERBOSE:-1}

BASELINENAME=${BASELINENAME:-agent_baseline}
BASELINE_SPEC=${BASELINE_SPEC:-closed_loop.baselines.${BASELINENAME}:create_baseline}

if [[ "$MODE" == "public" ]]; then
    ENABLE_TUNNEL=1
fi

if [[ "$VERBOSE" == "1" ]]; then
    VERBOSE_FLAG="--verbose"
else
    VERBOSE_FLAG=""
fi

case "$MODE" in
    shared|shm)
        SHM_NAME=${SHM_NAME:-unity_rgbd_shm}
        SHM_SIZE=${SHM_SIZE:-67108864}
        SHM_POLL_INTERVAL=${SHM_POLL_INTERVAL:-0.02}
        PERSIST_RGBD=${PERSIST_RGBD:-}
        CTRL_IN_NAME=${CTRL_IN_NAME:-unity_ctrl_shm}
        CTRL_OUT_NAME=${CTRL_OUT_NAME:-agent_ctrl_shm}
        CTRL_SIZE=${CTRL_SIZE:-4194304}
        SHARED_ARGS=()
        if [[ -n "$PERSIST_RGBD" ]]; then
            SHARED_ARGS+=(--persist-rgbd "$PERSIST_RGBD")
        fi

        echo "🔗 Starting shared memory connector (name=${SHM_NAME}, size=${SHM_SIZE} bytes)..."
        python -m closed_loop.freeaskworld_connector.shared_memory_server \
            --memory-name "$SHM_NAME" \
            --memory-size "$SHM_SIZE" \
            --control-in-name "$CTRL_IN_NAME" \
            --control-out-name "$CTRL_OUT_NAME" \
            --control-size "$CTRL_SIZE" \
            --poll-interval "$SHM_POLL_INTERVAL" \
            --baseline "$BASELINE_SPEC" \
            $VERBOSE_FLAG \
            "${SHARED_ARGS[@]}"
        ;;

    local|public)
        CLOUDFLARED_PID=""
        if [[ "$ENABLE_TUNNEL" == "1" ]]; then
            echo "🌐 Starting cloudflared tunnel for HTTP signaling..."
            python start_cloudflared.py "$PORT" &
            CLOUDFLARED_PID=$!
        else
            echo "📡 No tunnel; using LAN at http://${HOST}:${PORT}"
        fi

        echo "🚀 Starting WebRTC server (mode=${MODE})..."
        python -m closed_loop.freeaskworld_connector.webrtc_server \
            --host "$HOST" \
            --port "$PORT" \
            --baseline "$BASELINE_SPEC" \
            $VERBOSE_FLAG

        if [[ -n "$CLOUDFLARED_PID" ]]; then
            wait "$CLOUDFLARED_PID"
        fi
        ;;

    *)
        echo "❌ Unknown MODE: $MODE (use local, public, or shared)" >&2
        exit 1
        ;;
esac
