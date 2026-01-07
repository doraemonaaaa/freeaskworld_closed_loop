## Installation
```
# download map plugin
sudo apt install ros-$ROS_DISTRO-rtabmap-ros

sudo apt install \
  ros-jazzy-rtabmap \
  ros-jazzy-rtabmap-ros \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-compressed-image-transport   

# if you want to local source code
git clone --branch ros2 https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros
```

```
# Recommend DDS(Must use this, default can't run this program, not smooth)
sudo apt update
sudo apt install ros-jazzy-rmw-cyclonedds-cpp

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Cyclone prefers multicast by default, if your router got too much spammed, 
# disable multicast with (https://github.com/ros2/rmw_cyclonedds/issues/489):
export CYCLONEDDS_URI="<Disc><DefaultMulticastAddress>0.0.0.0</></>"
```

### Sensor fusion
```
# Download robot localization
sudo apt install ros-jazzy-robot-localization
```