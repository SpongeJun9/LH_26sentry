#!/bin/bash

# 清理环境变量，移除MVS路径
export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | tr ':' '\n' | grep -v MVS | tr '\n' ':' | sed 's/:$//')

# 确保ROS2核心库路径优先
export LD_LIBRARY_PATH=/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/opt/rviz_ogre_vendor/lib:/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

# 修复libusb符号问题
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libusb-1.0.so.0:$LD_PRELOAD

# 加载ROS2环境
source /home/lionheart/new_dev/install/setup.bash

# 启动导航
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py world:=map_61 slam:=False use_robot_state_pub:=True
