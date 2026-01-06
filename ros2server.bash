#!/bin/bash

# Attention: start this firstly, then unity

# 获取并激活 uv 创建的虚拟环境
PROJECT_ROOT="/home/pengyh/workspace/FreeAskAgent"
VENV_PATH="/home/pengyh/workspace/FreeAskAgent/.venv"
BASELINE_NAME="agent_baseline"
EPISODES=46
TCP_PORT=10000

# ource ROS 2 系统环境
source /opt/ros/jazzy/setup.bash
source /home/pengyh/workspace/FreeAskAgent/closed_loop/ros2/install/setup.bash

# 指定 PYTHONPATH 
export PYTHONPATH=$VENV_PATH/lib/python3.12/site-packages:$PYTHONPATH
export PYTHONPATH=$PROJECT_ROOT:$PYTHONPATH


if [ -d "$VENV_PATH" ]; then
    source "$VENV_PATH/bin/activate"
    echo "Activated uv virtualenv at $VENV_PATH"
else
    echo "Warning: Virtualenv not found at $VENV_PATH"
fi


# 检查端口是否被占用
if lsof -i tcp:$TCP_PORT >/dev/null 2>&1; then
    echo "Port $TCP_PORT is in use. Killing process..."
    sudo lsof -t -i tcp:$TCP_PORT | xargs sudo kill -9
fi
# 启动 TCP Endpoint
ros2 run ros_tcp_endpoint default_server_endpoint &
echo "Start ros2 server"
sleep 0.5

## Additional plugins
RTABMAP_PROC=$(pgrep -f "ros2 launch rtabmap_launch rtabmap.launch.py")
if [ -n "$RTABMAP_PROC" ]; then
    echo "rtabmap_launch is already running (PID: $RTABMAP_PROC). Killing it..."
    kill -9 $RTABMAP_PROC
    sleep 0.5
fi
ros2 launch rtabmap_launch rtabmap.launch.py \
    rgb_topic:=/camera/color/image_raw \
    depth_topic:=/camera/depth/image_raw \
    camera_info_topic:=/camera/color/camera_info \
    odom_topic:=/simulator_msg/odom2baselink\
    frame_id:=base_link \
    rgb_frame_id:=camera_link \
    depth_frame_id:=camera_link \
    camera_frame_id:=camera_link \
    approx_sync:=true\
    use_sim_time:=true\
    approx_sync_max_interval:=0.3

# 运行baseliine节点
# for ((i=1;i<=EPISODES;i++)); do
#   echo "===== Episode $i ====="
#   ros2 run vln_connector $BASELINE_NAME
#   sleep 1
# done