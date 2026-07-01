这是一个模拟裁判系统消息的功能包，用于测试哨兵自主决策代码是否能用。

安装依赖：
pip install PyQt5
或者
sudo apt-get install python3-pyqt5

运行方法：

colcon build --packages-select reference_data_gui

source install/setup.bash

ros2 run reference_data_gui reference_data_gui

