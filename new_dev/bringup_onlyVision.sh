#仅视觉启动脚本，无可视化界面，无决策、无导航，比赛现场导航寄了使用
#!/bin/bash 
source /opt/ros/humble/setup.bash
source ~/dev_ws/install/local_setup.bash
echo ' '| sudo -S chmod 777 /dev/ttyUSB0
cd ~/dev_ws/
colcon build --symlink-install
gnome-terminal -- bash -c "ros2 launch core_serial core_serial_bringup.launch.py;exec bash;"
sleep 1s 	
gnome-terminal -- bash -c "ros2 launch rm_vision_bringup vision_bringup.launch.py;exec bash;"
./foxglove_bridge.sh
#sleep 1s 
#gnome-terminal -- bash -c "ros2 run core_watchdog core_watchdog_node;exec bash;"

