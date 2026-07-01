echo "winwindut" | sudo -S chmod 777 /dev/ttyUSB0
chmod 777 /dev/ttyUSB1
chmod 777 /dev/ttyUSB2
chmod 777 /dev/ttyUSB3
export ROS_LOG_DIR=/home/dut0bug/log_rmuc/serial/my_logs_$(date +%Y%m%d-%H%M%S)
source /home/dut0bug/sb_ws_5/install/setup.bash
ros2 run robo_bridge robo_bridge_node
