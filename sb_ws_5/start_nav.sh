export ROS_LOG_DIR=/home/dut0bug/log_rmuc/nav//my_logs_$(date +%Y%m%d-%H%M%S)
source /home/dut0bug/sb_ws_5/install/setup.bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
world:=map \
slam:=False \
use_robot_state_pub:=True

#ros2 launch pb2025_nav_bringup rm_sentry_reality_launch.py \
#world:=725_sz \
#slam:=False \
#use_robot_state_pub:=True
