#仅导航启动脚本，有可视化界面，无看门狗节点，日常调试使用
#!/bin/bash 
source /opt/ros/humble/setup.bash
source ~/dev_ws/install/local_setup.bash
echo ' '| sudo -S chmod 777 /dev/ttyUSB0
cd ~/dev_ws/
colcon build --symlink-install
gnome-terminal -- bash -c "ros2 launch core_serial core_serial_bringup.launch.py;exec bash;"
sleep 1s
gnome-terminal -- bash -c "ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py world:=rmul2026 slam:=False use_robot_state_pub:=True;exec bash;"
#sleep 4s 
#gnome-terminal -- bash -c "ros2 launch dec_run dec_run_bringup.launch.py;exec bash;"

