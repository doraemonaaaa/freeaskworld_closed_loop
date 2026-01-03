#!/bin/bash

# 1. 自动获取并激活 uv 创建的虚拟环境
PROJECT_ROOT="/home/pengyh/workspace/FreeAskAgent"
VENV_PATH="/home/pengyh/workspace/FreeAskAgent/.venv"
BASELINE_NAME="agent_baseline"
EPISODES=46

if [ -d "$VENV_PATH" ]; then
    source "$VENV_PATH/bin/activate"
    echo "Activated uv virtualenv at $VENV_PATH"
else
    echo "Warning: Virtualenv not found at $VENV_PATH"
fi

# 2. Source ROS 2 系统环境
source /opt/ros/jazzy/setup.bash

# 3. Source 你的工作空间环境 (必须用绝对路径，修复之前的报错)
source /home/pengyh/workspace/FreeAskAgent/closed_loop/ros2/install/setup.bash

# 4. 强制指定 PYTHONPATH (这是解决问题的双保险)
# 将虚拟环境的 site-packages 放到最前面
export PYTHONPATH=$VENV_PATH/lib/python3.12/site-packages:$PYTHONPATH
export PYTHONPATH=$PROJECT_ROOT:$PYTHONPATH

# 5. 运行节点
for ((i=1;i<=EPISODES;i++)); do
  echo "===== Episode $i ====="
  ros2 run vln_connector $BASELINE_NAME
  sleep 1
done
