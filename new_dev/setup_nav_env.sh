#!/bin/bash
# 添加到 ~/.bashrc 以永久修复导航启动问题

# 清理MVS路径
export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | tr ':' '\n' | grep -v MVS | tr '\n' ':' | sed 's/:$//')

# ROS2库路径优先
export LD_LIBRARY_PATH=/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/opt/rviz_ogre_vendor/lib:/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

# 修复libusb
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libusb-1.0.so.0:$LD_PRELOAD
