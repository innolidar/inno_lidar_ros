#!/bin/bash
set -e

echo "=== Switch to ROS2 environment ==="

# 切换 package.xml
cp -f ./inno_lidar_msg/package_ros2.xml ./inno_lidar_msg/package.xml
cp -f ./inno_lidar_ros/package_ros2.xml ./inno_lidar_ros/package.xml


# 修改 CMakeLists 的 COMPILE_METHOD 为 COLCON
sed -i 's/set(COMPILE_METHOD .*)/set(COMPILE_METHOD COLCON)/' ./inno_lidar_msg/CMakeLists.txt
sed -i 's/set(COMPILE_METHOD .*)/set(COMPILE_METHOD COLCON)/' ./inno_lidar_ros/CMakeLists.txt


# 编译
colcon build


