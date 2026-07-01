
xiaoqi_wen@xiaoqiwen:~/new_sim/src$ ls
pb2025_robot_description  rmoss_gazebo        rmu_gazebo_simulator
pb2025_sentry_nav         rmoss_gz_resources  sdformat_tools
rmoss_core                rmoss_interfaces



rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=release


可视化：
source install/setup.bash
ros2 launch pb2025_robot_description robot_description_launch.py

1.启动仿真环境
source install/setup.bash
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py

2.单机器人：
导航模式：
source install/setup.bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
world:=rmul_2026 \
slam:=False

建图模式：
source install/setup.bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
slam:=True

ros2 run rmoss_gz_base test_chassis_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base -p v:=1.3 -p w:=0.3

pcl_viewer -fc 255,255,255 -ax 3 scans.pcd



#### 2.3.2 实车

建图模式：

```bash
source install/setup.bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
slam:=True \
use_robot_state_pub:=True
```

source install/setup.bash
ros2 launch pcd2pgm pcd2pgm_launch.py

source install/setup.bash
ros2 run nav2_map_server map_saver_cli -f test05233


保存栅格地图：`ros2 run nav2_map_server map_saver_cli -f <YOUR_MAP_NAME>  --ros-args -r __ns:=/red_standard_robot1`
如果是使用small_point_lio，需要进行这些操作
**Step 1**: set `save_pcd` to `true` in config file.

**Step 2**: run small point lio until the map is finished.

**Step 3**: save map by calling service:

```cpp
ros2 service call /map_save std_srvs/srv/Trigger
```

导航模式：

注意修改 `world` 参数为实际地图的名称

```bash
source install/setup.bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
world:=rmsim \
slam:=False \
use_robot_state_pub:=True
```

source install/setup.bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
world:=map \
slam:=False \
use_robot_state_pub:=True


source install/setup.bash
ros2 launch tunnel_mode_manager tunnel_mode_manager.launch.py

source install/setup.bash
ros2 run rm_decision rmuc_decision_qianshao_delayed_outpost

source install/setup.bash
ros2 run reference_data_gui reference_data_gui

```
ros2 topic pub -r 50 /red_standard_robot1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.3, z: 0.0}, angular: {x: 0.3, y: 0.0, z: 0.5}}"
ros2 topic pub -r 50 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: -0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```
```
source install/setup.bash
ros2 run robo_bridge robo_bridge_node
```

