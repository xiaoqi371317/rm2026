>
参考北极熊导航分支：
Commits on Feb 24, 2025
82a76439cd6fe46a69a70f1d44afb0875982a82d
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

-----------------------------------------------------
# small gicp install
sudo apt install -y libeigen3-dev libomp-dev

git clone https://github.com/koide3/small_gicp.git
cd small_gicp
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j
sudo make install
-----------------------------------------------------

# clone 代码到工作空间
 --nav                          哨兵导航代码
 --nav_sim                      哨兵导航工作空间
 --pb2025_robot_description     哨兵urdf描述文件
 --pcd2pgm                      点云转换为栅格地图
 --robo_bridge                  上下位机通信
-----------------------------------------------------

# build
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
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
-----------------------------------------------------


# Sim 仿真环境
启动仿真环境
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py 

打开 /home/xiaoqi_wen/lingrm/install/rmu_gazebo_simulator/lib/rmu_gazebo_simulator/simple_competition_1v1.py，确保第一行是：#!/usr/bin/env python3
⚠️ 注意：即使你本地开发目录（src/）里的 .py 文件有 shebang，安装后的文件（在 install/ 目录下）也必须有。如果你用的是 colcon build，请确认你的源文件有 shebang，否则安装后的脚本不会有。
4. launch 文件错误地将 .py 当作可执行程序启动
检查你的 bringup_sim.launch.py 中是如何调用 simple_competition_1v1.py 的。
    ❌ 错误方式（直接 exec 脚本路径）：
    ExecuteProcess(cmd=['/path/to/simple_competition_1v1.py'])
    ✅ 正确方式（通过 python3 显式调用）：
    ExecuteProcess(cmd=['python3', '/path/to/simple_competition_1v1.py'])

ros2 launch rmu_gazebo_simulator bringup_sim.launch.py 
启动仿真导航
ros2 launch pb2025_nav_bringup rm_sentry_simulation_launch.py \
world:=rmsim \
slam:=False 

启动仿真建图
ros2 launch pb2025_nav_bringup rm_sentry_simulation_launch.py \
world:=rmsim \
slam:=True 
注：建图生成的pcd保存在pointlio中的PCD文件夹下面，默认命名为scans.pcd

仿真中控制机器人移动
ros2 run rmoss_gz_base test_chassis_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base -p v:=1.3 -p w:=0.3




# 实际测试
如果是使用small_point_lio，需要进行这些操作
**Step 1**: set `save_pcd` to `true` in config file.

**Step 2**: run small point lio until the map is finished.

**Step 3**: save map by calling service:

```cpp
ros2 service call /map_save std_srvs/srv/Trigger
```


# utils
查看pcd点云：
pcl_viewer -fc 255,255,255 -ax 3 scans.pcd

pcd2pgm:
ros2 launch pcd2pgm pcd2pgm_launch.py
ros2 run nav2_map_server map_saver_cli -f <YOUR_MAP_NAME>

# 需要修改的参数
串口裁判系统数据
红蓝方点
重力加速度
决策条件



[rviz2-9] [INFO] [1765174334.462008463] [rviz2]: Trying to create a map of size 100 x 100 using 1 swatches
[rviz2-9] [ERROR] [1765174334.496056634] [rviz2]: Vertex Program:rviz/glsl120/indexed_8bit_image.vert Fragment Program:rviz/glsl120/indexed_8bit_image.frag GLSL link result : 
[rviz2-9] active samplers with a different type refer to the same texture image unit





