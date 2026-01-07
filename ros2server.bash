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

## Additional plugins
# 1. 杀掉旧的 rtabmap_launch 进程（如果存在）
RTABMAP_PROC=$(pgrep -f "ros2 launch rtabmap_launch rtabmap.launch.py")
if [ -n "$RTABMAP_PROC" ]; then
    echo "rtabmap_launch is already running (PID: $RTABMAP_PROC). Killing it..."
    kill -9 $RTABMAP_PROC
    sleep 1
fi

# 2. 启动 robot_localization EKF（后台运行）
echo "Starting robot_localization EKF with fused odometry..."
ros2 launch robot_localization ekf.launch.py \
    params_file:=/home/pengyh/workspace/FreeAskAgent/closed_loop/ros2/ekf.yaml &

# 等待 EKF 启动并开始发布 /odom_fused（很重要！）
sleep 3

# 检查 /odom_fused 是否已发布
if ros2 topic list | grep -q "/odom_fused"; then
    echo "/odom_fused is publishing. Starting RTAB-Map..."
else
    echo "Warning: /odom_fused not detected yet, proceeding anyway..."
fi

# 3. 启动 RTAB-Map，使用融合后的 odom
ros2 launch rtabmap_launch rtabmap.launch.py \
    rgb_topic:=/simulator_msg/camera/color/image_raw \
    depth_topic:=/simulator_msg/camera/depth/image_raw \
    camera_info_topic:=/simulator_msg/camera/color/camera_info \
    imu_topic:=/simulator_msg/imu \
    subscribe_odom_info:=True \
    odom_topic:=/odom_fused \
    visual_odometry:=false \
    odom_frame_id:=odom \
    frame_id:=base_link \
    subscribe_scan:=false \
    subscribe_scan_cloud:=false \
    use_sim_time:=true \
    approx_sync:=true \
    approx_sync_max_interval:=0.02 \
    qos_image:=2 \
    qos_imu:=2 \
    qos_odom:=2 \
    rviz:=true \
    wait_for_transform:=0.5 \
    rtabmap_args:="--delete_db_on_start \
                    --Rtabmap/DetectionRate 15 \             # 降低关键帧率到5Hz（原来可能10+），减少错误节点插入
                    --RGBD/CreateOccupancyGrid true \
                    --RGBD/NeighborLinkRefining true \
                    --RGBD/AngularUpdate 0.2 \          # 室外运动慢时可放宽，减少频繁更新
                    --RGBD/LinearUpdate 0.2 \           # 同上，避免小抖动触发过多节点
                    --RGBD/ProximityBySpace false \     # 强烈推荐关闭！室外地图大，开启会极慢或崩溃
                    --RGBD/ProximityByTime true \       # 推荐改用时间邻近检测，更适合大尺度
                    --RGBD/ProximityPathMaxNeighbors 0 \ # 关闭空间邻近后此参数无效
                    --Mem/UseScanMatching true \
                    --Mem/STMSize 15 \
                    --Optimizer/Slam2D true\
                    --Grid/3D true \
                    --Grid/Sensor 1 \
                    --Reg/Strategy 1 \                  # 0=Visual，1=ICP，2=Visual+ICP
                    --Reg/Force3DoF false \             # 关闭！室外地面不平，需要6DoF优化
                    --Icp/VoxelSize 0.2 \                  # 更大体素，强力降采样，减少噪声影响（从0.1改到0.15~0.2）
                    --Icp/PointToPlaneMinComplexity 0.1 \   # 增加平面复杂度阈值，避免在噪声上拟合假平面
                    --Icp/CorrespondenceRatio 0.2 \        # 更严格，只用20%以上的匹配点（从0.3降到0.2）
                    --Icp/MaxCorrespondenceDistance 0.2 \  # 限制最大对应距离，防止远距离误匹配
                    --Icp/Epsilon 0.001 \                   # 收敛阈值
                    --Icp/Iterations 50 \                   # 增加迭代次数，提高配准精度
                    --Grid/DepthDecimation 8 \          # 如果仍用depth，增大抽稀
                    --Grid/RangeMin 0.2 \               # 最小范围稍增大，避免近距离噪声
                    --Grid/RangeMax 8.0 \               # 增大到8-10m（根据你的RGBD相机实际有效深度）
                    --Grid/CellSize 0.1 \              # 可保持，或增大到0.1降低分辨率节省内存
                    --Grid/ClusterRadius 0.15 \          # 增大聚类半径，过滤阳光噪声
                    --Grid/MinClusterSize 80 \         # 减小最小簇大小，保留更多有效点
                    --Grid/NoiseFilteringRadius 0.0 \   # 关闭或减小噪声过滤，室外噪声模式不同
                    --Grid/NoiseFilteringMinNeighbors 0 \
                    --Grid/MaxGroundHeight 0.1 \        # 根据机器人高度调整地面阈值
                    --Grid/MaxObstacleHeight 2.5 \      # 增大障碍高度（室外可能有树、人等）
                    --Grid/RayTracing true \
                    --Grid/FlatObstacleDetected true \  # 新增：检测平坦障碍（如地面凹凸）
                    --Grid/NormalForFlatObstacles true \ # 新增：提高地面检测准确性"

# 运行baseliine节点
# for ((i=1;i<=EPISODES;i++)); do
#   echo "===== Episode $i ====="
#   ros2 run vln_connector $BASELINE_NAME
#   sleep 1
# done

# Debug Utils
# ros2 run tf2_tools view_frames
