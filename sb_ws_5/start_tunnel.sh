export ROS_LOG_DIR=/home/dut0bug/log_rmuc/nav//my_logs_$(date +%Y%m%d-%H%M%S)
source /home/dut0bug/sb_ws_5/install/setup.bash
ros2 launch tunnel_mode_manager tunnel_mode_manager.launch.py
