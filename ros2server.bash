#!/bin/bash
source ros2/install/setup.bash

TCP_PORT=10000

# 检查端口是否被占用，如果占用则杀掉进程
if lsof -i tcp:$TCP_PORT >/dev/null 2>&1; then
    echo "Port $TCP_PORT is in use. Killing process..."
    sudo lsof -t -i tcp:$TCP_PORT | xargs sudo kill -9
fi

# 启动 TCP Endpoint
ros2 run ros_tcp_endpoint default_server_endpoint &

# 等待几秒确保 TCP Endpoint 启动完成
sleep 2

# 启动 Python 订阅节点
python ros2/server.py
