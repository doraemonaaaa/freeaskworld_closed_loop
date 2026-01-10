

pkill -9 -f ekf_filter_node
pkill -9 -f robot_localization
pkill -9 -f rtabmap
pkill -9 -f rtabmapviz
pkill -9 -f rgbd_odometry
pkill -9 -f icp_odometry
pkill -9 -f rtabmap.launch.py


## Additional plugins
# 1. 杀掉旧的 rtabmap_launch 进程（如果存在）
RTABMAP_PROC=$(pgrep -f "ros2 launch rtabmap_launch rtabmap.launch.py")
if [ -n "$RTABMAP_PROC" ]; then
    echo "rtabmap_launch is already running (PID: $RTABMAP_PROC). Killing it..."
    kill -9 $RTABMAP_PROC
    sleep 1
fi

# 3. 启动 RTAB-Map，使用融合后的 odom
# 启动 RTAB-Map，使用融合后的 odom
# 参数说明：
# --Rtabmap/DetectionRate 15                  降低关键帧率到 5Hz，减少错误节点插入
# --RGBD/AngularUpdate 0.2                     室外运动慢时可放宽，减少频繁更新
# --RGBD/LinearUpdate 0.2                      避免小抖动触发过多节点
# --RGBD/ProximityBySpace false                室外大地图强烈建议关闭
# --RGBD/ProximityByTime true                  推荐改用时间邻近检测
# --Rtabmap/MaxRepublished 0                   不重复 republish 节点，避免刷 WARN
# --LogLevel Warning                            只显示 Warning+ 日志
# 其余参数根据需要保留
ros2 launch rtabmap_launch rtabmap.launch.py \
    rgb_topic:=/simulator_msg/camera/color/image_raw \
    depth_topic:=/simulator_msg/camera/depth/image_raw \
    camera_info_topic:=/simulator_msg/camera/color/camera_info \
    imu_topic:=/simulator_msg/imu \
    subscribe_odom_info:=True \
    odom_topic:=/simulator_msg/odom \
    visual_odometry:=false \
    odom_frame_id:=odom \
    frame_id:=base_link \
    use_sim_time:=true \
    approx_sync:=true \
    approx_sync_max_interval:=0.05 \
    qos_image:=2 \
    qos_imu:=2 \
    qos_odom:=2 \
    rviz:=true \
    wait_for_transform:=0.5 \
    rtabmap_args:="--delete_db_on_start \
                --Rtabmap/DetectionRate 5 \
                --RGBD/CreateOccupancyGrid true \
                --RGBD/NeighborLinkRefining true \
                --RGBD/AngularUpdate 0.2 \
                --RGBD/LinearUpdate 0.2 \
                --RGBD/ProximityBySpace false \
                --RGBD/ProximityByTime true \
                --RGBD/ProximityPathMaxNeighbors 0 \
                --Mem/UseScanMatching true \
                --Mem/STMSize 10 \
                --Optimizer/Slam2D true \
                --Reg/Strategy 1 \
                --Reg/Force3DoF true \
                --Icp/VoxelSize 0.1 \
                --Icp/PointToPlaneMinComplexity 0.1 \
                --Icp/CorrespondenceRatio 0.2 \
                --Icp/MaxCorrespondenceDistance 0.2 \
                --Icp/Epsilon 0.001 \
                --Icp/Iterations 20 \
                --Vis/MaxDepth 5 \
                --Grid/3D true \
                --Grid/Sensor 1 \
                --Grid/DepthDecimation 8 \
                --Grid/RangeMin 0.1 \
                --Grid/RangeMax 5.0 \
                --Grid/CellSize 0.05 \
                --Grid/ClusterRadius 0.15 \
                --Grid/MinClusterSize 80 \
                --Grid/NoiseFilteringRadius 0.0 \
                --Grid/NoiseFilteringMinNeighbors 0 \
                --Grid/MaxGroundHeight 0.1 \
                --Grid/MaxObstacleHeight 2.5 \
                --Grid/RayTracing true \
                --Grid/FlatObstacleDetected true \
                --Grid/NormalForFlatObstacles true \
                --Rtabmap/MaxRepublished 0" \
    log_level:=WARN \
    &
