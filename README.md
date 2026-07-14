
# PCD文件记得从网盘下载到navbringup的pcd文件夹中
########### 凌BUG导航仿真+实车功能包简介-----by周星辰+文晓琦+江令文

########### 0.注意事项
>
参考北极熊导航分支：
>

！！！！！！！！！在配置代码的时候最好不要有conda环境！！！！！！！！！！！！！
如果一定要有conda，可以参考我的经历：
1.安装miniforge3（或者其他的conda，如anaconda,miniconda）
2.创建一个虚拟环境，python版本为3.10：conda create -n nav python=3.10
3.编译环境的时候会报错缺少包，pip下载即可，我缺少的包如下，可按照自己的实际情况安装
pip install catkin_pkg
pip install empy==3.3.4
pip install numpy
pip install lark
之后就可以正常运行代码了，由于ros2 humble默认的python版本为3.10,故创建虚拟环境的时候最好一致，避免不必要的报错

显示器问题，不用管
[rviz2-9] [INFO] [1765174334.462008463] [rviz2]: Trying to create a map of size 100 x 100 using 1 swatches
[rviz2-9] [ERROR] [1765174334.496056634] [rviz2]: Vertex Program:rviz/glsl120/indexed_8bit_image.vert Fragment Program:rviz/glsl120/indexed_8bit_image.frag GLSL link result : 
[rviz2-9] active samplers with a different type refer to the same texture image unit

-----------------------------------------------------
# 参数配置
1.livox修改ip
修改livox_ros_driver2/config/MID360_config.json 里面的雷达ip

2.urdf描述文件 (激光雷达建议斜45度摆放)
主要修改pb2025_sentry_robot.sdf.xmacro文件中的：
<xmacro_block name="livox" prefix="front_" parent="gimbal_yaw" pose="0.0 -0.2 0.10 0.0 ${pi/4} -${pi/2}" update_rate="20" samples="1875"/>
可以可视化urdf文件查看是否转换正确：
ros2 launch pb2025_robot_description robot_description_launch.py

3.pcd2pgm
主要修改pcd2pgm.yaml中的pcd_file(pcd文件位置)和odom_to_lidar_odom
odom_to_lidar_odom里面的参数与urdf里面的激光雷达位置相对应，例如
<xmacro_block name="livox" prefix="front_" parent="gimbal_yaw" pose="0.0 -0.2 0.10 0.0 ${pi/4} -${pi/2}" update_rate="20" samples="1875"/>
则odom_to_lidar_odom的相关参数为（需要取反）：
odom_to_lidar_odom: [0.0, 0.2, -0.1, 0.0, -0.785398, 1.570796]

4.pointlio重力参数
修改point_lio/config/mid360.yaml里面的gravity和gravity_init参数
命令启动：
ros2 launch livox_ros_driver2 msg_MID360_launch.py
新开一个终端使用（查看实时 IMU 数据，根据 IMU 线加速度值写入 gravity 和 gravity_init，注意重力方向加负号）：
ros2 topic echo livox/imu 
如果启动的是导航代码来查看IMU数据，需要加上命名空间：
ros2 topic echo /red_standard_robot1/livox/imu

5.仿真地图修改（可选）
修改rmu_gazebo_simulator/config/gz_world.yaml中的world参数

6.新地图替换
在建立了一个新地图后，需要在pb2025_nav_bringup中的pcd和map分别存储pcd文件和yaml、pgm文件

### 7.可能需要修改的参数
serial串口裁判系统数据
红蓝方点
重力加速度
决策条件
rmu_gazebo_simulator仿真环境的地图文件+生成的机器人描述文件+生成哪个机器人
simulation_robot_gimbal.sdf.xmacro仿真云台大yaw雷达哨兵
simulation_robot.sdf.xmacro仿真底盘雷达哨兵

pb2025_sentry_robot.sdf.xmacro真车哨兵

-----------------------------------------------------





########### 2.环境准备
-----------------------------------------------------
# small gicp install（third_party库下）
sudo apt install -y libeigen3-dev libomp-dev

git clone https://github.com/koide3/small_gicp.git
cd small_gicp
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j
sudo make install

# 检查是否安装成功
ls /usr/local/lib/cmake/small_gicp/
small_gicp-config.cmake          small_gicp-targets.cmake
small_gicp-config-version.cmake  small_gicp-targets-release.cmake
-----------------------------------------------------
# colcon build
```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=release
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=release
```

########### 3.仿真相关
1.启动仿真环境
```bash
unset LD_LIBRARY_PATH
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=release
pkill -9 -f "ign gazebo"
pkill -9 -f "ruby"
pkill -9 -f "gz sim"
source install/setup.bash
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
```

如果启动报错，大概率是没有给权限：
xiaoqi_wen@xiaoqiwen:~/Desktop/666/lastt/src/rmu_gazebo_simulator/rmu_gazebo_simulator/scripts/referee_system$ chmod +x simple_competition_1v1.py 


2.单机器人：
# 建图模式：
```bash
source install/setup.bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
slam:=True
```

ros2 run rmoss_gz_base test_chassis_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base -p v:=1.3 -p w:=0.3   #加载仿真控制小车扫描 

pcl_viewer -fc 255,255,255 -ax 3 scans.pcd   #加载扫描点云图

注：建图生成的pcd保存在pointlio中的PCD文件夹下面，默认命名为scans.pcd

保存栅格地图：`ros2 run nav2_map_server map_saver_cli -f <YOUR_MAP_NAME>  --ros-args -r __ns:=/red_standard_robot1`      
# 或者pcd2pgm在线制作地图
pcd2pgm:
ros2 launch pcd2pgm pcd2pgm_launch.py
ros2 run nav2_map_server map_saver_cli -f <YOUR_MAP_NAME>

# 导航模式：
```bash
source install/setup.bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
world:=rmuc_2026 \
slam:=False
```


-----------------------------------------------------
（以下内容可有可无）
# 测试
source install/setup.bash
ros2 launch rm_decision test_nav_goal.launch.py
# 决策树节点
source install/setup.bash
ros2 launch rm_decision test_bt_decision.launch.py
# 模拟裁判系统
source install/setup.bash
ros2 run rm_decision mock_referee

# 操作手模式
source install/setup.bash
python3 src/rmu_gazebo_simulator/rmu_gazebo_simulator/scripts/player_web/main_no_vision.py

http://localhost:5000/
-----------------------------------------------------



########### 4.真实车相关

# 建图模式：
```bash
source install/setup.bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
slam:=True \
use_robot_state_pub:=True
```
保存栅格地图：`ros2 run nav2_map_server map_saver_cli -f <YOUR_MAP_NAME>  --ros-args -r __ns:=/red_standard_robot1`      
# 或者pcd2pgm在线制作地图
pcd2pgm:
ros2 launch pcd2pgm pcd2pgm_launch.py
ros2 run nav2_map_server map_saver_cli -f <YOUR_MAP_NAME>

# 导航模式：
注意修改 `world` 参数为实际地图的名称
```bash
source install/setup.bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
world:=test0624 \
slam:=False \
use_robot_state_pub:=True
```

#用桥与电控通信
```
source install/setup.bash
ros2 run robo_bridge robo_bridge_node
```


########### n.常用操作命令（待更新）
1、仿真监察tf树（非仿真直接去掉red_standard_robot1）
ros2 run tf2_tools view_frames --ros-args -r /tf:=/red_standard_robot1/tf -r /tf_static:=/red_standard_robot1/tf_static


########### 1.项目目录架构+各个功能包分析

## 1.1 顶层目录结构

```text
lastt/
├── src/                         # ROS2 工作空间源码
│   ├── pb2025_sentry_nav/        # 哨兵导航主体：感知、定位、规划、控制、bringup
│   ├── pb2025_robot_description/ # 机器人 URDF/SDF/XMacro 描述与 TF 发布
│   ├── rmu_gazebo_simulator/     # RMU Ignition/Gazebo 仿真启动、世界、裁判系统脚本
│   ├── rmoss_core/               # RMOSS 通用底盘、相机、工具、弹道等基础库
│   ├── rmoss_gazebo/             # RMOSS Gazebo 桥接、底盘、相机、仿真插件
│   ├── rmoss_gz_resources/       # Gazebo 模型资源、RMUA 机器人模型、裁判模块资源
│   ├── rmoss_interfaces/         # RMOSS 自定义 msg/srv 接口
│   ├── rm_decision/              # 决策树、导航目标发送、裁判系统数据模拟/接口
│   ├── serial/                   # 与电控通信的 robo_bridge 和串口消息工具
│   ├── pcd2pgm/                  # PCD 点云地图转 PGM 栅格地图工具
│   └── sdformat_tools/           # SDF/URDF 转换辅助工具
├── third_party/                  # 第三方库，本项目主要使用 small_gicp
├── build/                        # colcon 编译产物
├── install/                      # colcon 安装空间，source install/setup.bash 后使用
└── log/                          # colcon 构建日志
```

## 1.2 导航主链路

当前导航链路可以按下面理解：

```text
雷达/仿真点云
  -> point_lio                 # 激光里程计，输出位姿与注册点云
  -> loam_interface             # 坐标系整理，将 lidar_odom / odom / base 关系接入导航
  -> sensor_scan_generation     # 生成雷达坐标系下的局部点云
  -> terrain_analysis/ext       # 根据点云高度与强度生成 terrain_map / terrain_map_ext
  -> Nav2 costmap               # local/global costmap
  -> ddr_opt_nav2_planner       # JPS 前端，发布 plan 和 GridBased/jps_path
  -> minco_trajectory_optimizer # 独立 MINCO 后端，订阅 JPS path，发布 minco_plan
  -> pb_omni_pid_pursuit_controller
  -> velocity_smoother / fake_vel_transform
  -> /cmd_vel                   # 仿真给底盘插件，实车给电控
```

建图模式下，`slam_toolbox` 负责生成 2D 栅格地图；导航模式下，`map_server` 加载已有地图，`small_gicp_relocalization` 使用先验 PCD 辅助 `map -> odom` 重定位。

## 1.3 pb2025_sentry_nav 主体功能包

`pb2025_nav_bringup`

导航启动和参数总入口。主要文件在 `launch/` 和 `config/` 下：

- `rm_navigation_simulation_launch.py`：单机器人仿真导航/建图入口。
- `rm_navigation_reality_launch.py`：实车导航/建图入口。
- `bringup_launch.py`：组合 localization、navigation、slam 等子启动文件。
- `navigation_launch.py`：启动 Nav2 controller/planner/behavior/bt、terrain analysis、MINCO 优化器等。
- `localization_launch.py`：启动 point_lio、map_server、small_gicp 重定位。
- `slam_launch.py`：启动 slam_toolbox、pointcloud_to_laserscan、point_lio。
- `config/simulation/nav2_params.yaml`：仿真参数。
- `config/reality/nav2_params.yaml`：实车参数。

`ddr_opt_nav2_planner`

Nav2 全局规划插件。当前作为 `planner_server.GridBased` 使用，核心职责是从 costmap 中计算 JPS 前端路径，并发布：

- `plan`：Nav2 标准全局路径。
- `GridBased/jps_path`：前端 JPS 调试/参考路径，独立 MINCO 节点会订阅它做后端优化。
- Debug marker：用于 RViz 中查看 JPS、DDR/MINCO 相关调试信息。

当前配置中 `enable_backend: False`，表示 planner 插件内部不再重复跑 MINCO 后端，后端统一交给独立的 `minco_trajectory_optimizer`。

`minco_trajectory_optimizer`

独立 MINCO 轨迹优化节点。它订阅 `plan` 和 `GridBased/jps_path`，根据上一条可微 MINCO 轨迹做连续重规划，发布：

- `minco_plan`：控制器实际优先跟踪的优化轨迹。
- `minco_waypoints`、`minco_trajectory_marker`：RViz 调试 marker。

重点参数在 `nav2_params.yaml` 的 `minco_trajectory_optimizer.ros__parameters` 下，例如：

- `optimizer_backend: "ddr_msplanner"`：使用 DDR/MSPlanner 后端。
- `subscribe_jps_path: true`、`jps_plan_topic: "GridBased/jps_path"`：让 MINCO 跟随 JPS 更新。
- `enable_replan: true`、`replan_path_change_threshold: 0.05`：行进中路径变化触发重规划。
- `enable_narrow_passage_retry: true`：窄通道失败时降低安全裕度重试。

`pb_omni_pid_pursuit_controller`

Nav2 控制器插件，负责跟踪 `minco_plan` 或普通全局路径并输出速度。它会发布 `local_plan`、`lookahead_point` 等调试信息，并订阅 `minco_plan`。如果 MINCO 成功，它优先跟踪 MINCO；如果 MINCO 失败，则保持上一条有效轨迹或回退到可用路径，具体行为由参数控制。

`pb_nav2_plugins`

Nav2 扩展插件集合，当前主要包含：

- `pb_nav2_costmap_2d::IntensityVoxelLayer`：根据 terrain map 点云强度/高度构建 costmap 障碍层。
- `pb_nav2_behaviors/BackUpFreeSpace`：自定义后退脱困行为。

`fake_vel_transform`

虚拟速度参考坐标系转换。项目中 Nav2 的 `robot_base_frame` 常用 `gimbal_yaw_fake`，该包会发布 `gimbal_yaw -> gimbal_yaw_fake` TF，并把导航速度从 fake frame 转回真实底盘 frame 后发布到 `/cmd_vel`。真车电控只需要订阅最终 `/cmd_vel`。

`point_lio`

激光惯性里程计。实车读取 Livox MID360 数据，仿真读取转换后的点云数据，输出里程计和注册点云。建图时 PCD 默认保存在 `point_lio/PCD/`。

`loam_interface`

里程计接口层，将 point_lio 输出的位姿、注册点云和项目中的 TF 约定接起来，解决雷达坐标、底盘坐标、odom 坐标之间的转换问题。

`sensor_scan_generation`

把已注册点云转换到雷达/车体相关坐标系下，给后续 terrain analysis 使用。

`terrain_analysis` / `terrain_analysis_ext`

地形分析节点：

- `terrain_analysis`：偏局部范围，用于 local costmap。
- `terrain_analysis_ext`：偏大范围，用于 global costmap。

它们根据点云高度、强度、时间衰减等规则生成 `terrain_map` / `terrain_map_ext`，最终进入 Nav2 costmap。

`small_gicp_relocalization`

基于 small_gicp 的重定位节点，用当前点云与先验 PCD 匹配，给导航提供 `map -> odom` 修正。导航模式使用，建图模式一般关闭或由 launch 发送静态 `map -> odom`。

`livox_ros_driver2`

Livox 雷达驱动。真车使用 MID360 时需要配置 `config/reality/mid360_user_config.json` 中的雷达 IP、广播码等。

`ign_sim_pointcloud_tool`

仿真点云格式转换工具。Gazebo 输出点云缺少 point_lio 需要的部分字段，该节点补齐/转换为可被 point_lio 使用的格式。

`pointcloud_to_laserscan`

建图模式辅助包，把 3D 点云转换为 2D LaserScan，供 `slam_toolbox` 使用。

`pb_teleop_twist_joy`

手柄遥控节点，同时支持底盘和云台控制。实车/仿真都可通过 `joy_teleop_launch.py` 启动。

`minco_mpc_bringup` / `mpc_trajectory_tracker`

MINCO + MPC 跟踪实验链路。当前主导航链路使用 `minco_trajectory_optimizer + pb_omni_pid_pursuit_controller`，MPC 相关包更适合后续验证轨迹跟踪方案时使用。

## 1.4 仿真与机器人描述相关包

`rmu_gazebo_simulator`

RMU Gazebo/Ignition 仿真总入口，提供比赛场景、机器人 spawn、裁判系统脚本、网页操作手等。常用入口：

```bash
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
```

`pb2025_robot_description`

机器人描述包，使用 XMacro 维护 SDF/URDF。主要修改位置是 `resource/xmacro/` 下的机器人描述文件，例如雷达安装位置、云台/底盘结构、仿真传感器参数等。修改后可用：

```bash
ros2 launch pb2025_robot_description robot_description_launch.py
```

检查 TF 和模型是否正确。

`rmoss_gazebo`

RMOSS Gazebo 适配层，包含：

- `rmoss_gz_base`：仿真底盘控制。
- `rmoss_gz_cam`：仿真相机桥接。
- `rmoss_gz_bridge`：ROS 与 Gazebo 消息桥。
- `rmoss_gz_plugins`：RoboMaster 仿真插件。

`rmoss_gz_resources`

Gazebo 模型、材质、裁判系统模块等资源包。

`rmoss_core` / `rmoss_interfaces`

RMOSS 基础库和接口定义。包括底盘、相机、工具函数、弹道模型，以及 RMOSS 自定义 msg/srv。

## 1.5 决策、通信与地图工具

`rm_decision`

决策层相关包，包含测试导航目标、行为树决策、模拟裁判系统等。当前 README 后面的测试命令中会用到：

```bash
ros2 launch rm_decision test_nav_goal.launch.py
ros2 launch rm_decision test_bt_decision.launch.py
ros2 run rm_decision mock_referee
```

`serial/ros2-robo-bridge`

实车与电控通信桥。导航算法最终发布 `/cmd_vel`，如需通过串口发给电控，启动：

```bash
ros2 run robo_bridge robo_bridge_node
```

`serial/ros2-robo-utils`

串口桥相关消息和工具定义。

`pcd2pgm`

PCD 点云地图转 2D 栅格地图工具。常用于建图后把 `scans.pcd` 转成 Nav2 可加载的 `.pgm + .yaml`。

`sdformat_tools`

SDF/URDF 转换辅助工具，主要服务于 `pb2025_robot_description` 的 XMacro 生成流程。

## 1.6 常见问题定位思路

- 地图/定位漂移：优先看 `point_lio`、`loam_interface`、`small_gicp_relocalization`、`map -> odom` TF。
- costmap 障碍异常：优先看 `terrain_analysis`、`terrain_analysis_ext`、`IntensityVoxelLayer`、`global_costmap/local_costmap` 参数。
- JPS 能规划但 MINCO 不更新：优先看 `ddr_opt_nav2_planner` 的 `GridBased/jps_path`、`minco_trajectory_optimizer` 的 `subscribe_jps_path`、`enable_replan`、`replan_path_change_threshold`。
- 轨迹能出但车不走：优先看 `pb_omni_pid_pursuit_controller` 是否收到 `minco_plan`，以及 `velocity_smoother`、`fake_vel_transform`、`/cmd_vel`。
- 仿真点云/建图异常：优先看 `ign_sim_pointcloud_tool`、`pointcloud_to_laserscan`、`slam_toolbox`。
- 真车雷达无数据：优先看 `livox_ros_driver2` 的 MID360 IP/广播码配置，以及 `/livox/lidar`、`/livox/imu`。





