#全局启动脚本，有可视化界面
#!/bin/bash 
source /opt/ros/humble/setup.bash
source ~/dev_ws/install/local_setup.bash
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
echo ' '| sudo -S chmod 777 /dev/ttyUSB0
cd ~/dev_ws/
colcon build --symlink-install
gnome-terminal -- bash -c "ros2 launch core_serial core_serial_bringup.launch.py;exec bash;"
gnome-terminal -- bash -c "ros2 launch rm_vision_bringup vision_bringup.launch.py;exec bash;"	
sleep 3s
gnome-terminal -- bash -c "ros2 launch nav_bringup bringup_real_rviz.launch.py \
	world:=555 \
	mode:=nav \
	localization:=slam_toolbox;exec bash;"
sleep 5s 
gnome-terminal -- bash -c "ros2 launch dec_run dec_run_bringup.launch.py;exec bash;"
#./foxglove_bridge.sh
#sleep 1s 
#gnome-terminal -- bash -c "ros2 run core_watchdog core_watchdog_node;exec bash;"

