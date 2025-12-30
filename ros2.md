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
uv pip install em

## Build the packages, get connection pack in python
git clone -b main-ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
colcon build --symlink-install
source ./install/setup.bash

### Build Python Env
Sometimes in python3.12, have some build error such as empy have no interpreter, you just need construct a conda environment 3.10 to build it specially.
```
conda create -n ros2_jazzy python=3.10 -y
conda activate ros2_jazzy
conda install -c conda-forge cmake git wget curl pkg-config -y
conda install -c conda-forge empy -
conda install -c conda-forge empy lxml setuptools colcon-common-extensions -y
conda install numpy
pip install lark-parser
```

## How to use
ip addr show eth0  # Get wsl2 ip address, set inet to unity
ros2 run ros_tcp_endpoint default_server_endpoint  # Connect to Unity
 

