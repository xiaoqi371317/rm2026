export ROS_LOG_DIR=/home/dut0bug/log_rmuc/nav//my_logs_$(date +%Y%m%d-%H%M%S)
source /home/dut0bug/sb_ws_5/install/setup.bash
ros2 launch rm_decision qianshao_delayed_outpost.launch.py
#ros2 launch rm_decision qianshao_delayed_outpost_1.launch.py
