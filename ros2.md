# Install

## Install Ros2 (choose one)
### Install Ros2 Humble (ubuntu22.04)
```
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl gnupg lsb-release -y
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2-latest.list
sudo apt update && sudo apt install ros-humble-desktop -y

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Install Ros2 Jazzy (ubuntu24.04)
```
sudo apt update
sudo apt install curl gnupg lsb-release -y

sudo mkdir -p /etc/apt/keyrings
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | sudo gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list

echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Install Others
sudo apt install colcon  # ros2
uv pip install "empy<4"

## Build the packages, get connection pack in python
git clone -b main-ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
colcon build --symlink-install
source ./install/setup.bash

## If python3.12 have some build problem, just use this system python to build and used by uv python3.12
sudo apt update
sudo apt install -y python3-catkin-pkg python3-empy python3-colcon-common-extensions

## How to use
```
ip addr show eth0  # Get wsl2 ip address, set inet to unity
ros2 run ros_tcp_endpoint default_server_endpoint  # Connect to Unity
```
```
bash ros2server.bash  # this is a direct api script to execute your baseline, no need for other code
```

## Create your own baseline
Please refer to ros2/src/vln_connector/vln_connector/simple_baseline.py, using this format