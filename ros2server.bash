#!/bin/bash

# Attention: start this firstly, then unity

# 获取并激活 uv 创建的虚拟环境
PROJECT_ROOT="/home/pengyh/workspace/FreeAskAgent"
VENV_PATH="/home/pengyh/workspace/FreeAskAgent/.venv"
BASELINE_NAME="agent_baseline"
EPISODES=46
TCP_PORT=10000

# source ROS2 系统环境
source /opt/ros/jazzy/setup.bash
source /home/pengyh/workspace/FreeAskAgent/closed_loop/ros2/install/setup.bash

# ROS2 Setting
# Must use this, default can't run this program, not smooth
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Cyclone prefers multicast by default, if your router got too much spammed, 
# disable multicast with (https://github.com/ros2/rmw_cyclonedds/issues/489):
export CYCLONEDDS_URI="<Disc><DefaultMulticastAddress>0.0.0.0</></>"

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

# Additional plugins
# bash mapping.bash &

# run baseliine
for ((i=1;i<=EPISODES;i++)); do
  echo "===== Episode $i ====="
  ros2 run vln_connector $BASELINE_NAME
  sleep 1
done

# Debug Utils
# ros2 run tf2_tools view_frames
