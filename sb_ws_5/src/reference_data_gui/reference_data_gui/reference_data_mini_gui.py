#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from robo_utils.msg import ReferenceDataMini
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QLabel, QSpinBox, QPushButton, 
                            QGroupBox, QCheckBox, QTabWidget, QGridLayout,
                            QProgressBar, QSlider, QTextEdit, QComboBox)
from PyQt5.QtCore import QTimer, Qt, QDateTime
from PyQt5.QtGui import QColor, QTextCursor
import sys

class ReferenceDataMiniPublisher(Node):
    def __init__(self):
        super().__init__('reference_data_mini_publisher')
        self.publisher_ = self.create_publisher(ReferenceDataMini, '/offboardlink/reference_data_mini', 10)
        
    def publish_message(self, msg_data):
        try:
            msg = ReferenceDataMini()
            
            # 设置header
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "reference_data"
            
            # 设置所有字段
            msg.game_state = msg_data['game_state']
            msg.remaining_time = msg_data['remaining_time']
            msg.self_color = msg_data['self_color']
            msg.self_health = msg_data['self_health']
            msg.at_base_area = msg_data['at_base_area']
            msg.at_castle_area = msg_data['at_castle_area']
            msg.at_supply_area_unoverlap = msg_data['at_supply_area_unoverlap']
            msg.at_supply_area_overlap = msg_data['at_supply_area_overlap']
            msg.supply_area_status = msg_data['supply_area_status']
            msg.reserved = msg_data['reserved']
            msg.auto_aiming_status = msg_data['auto_aiming_status']
            msg.energy_percent = msg_data['energy_percent']
            msg.shoot_heat = msg_data['shoot_heat']
            msg.remaining_bullet = msg_data['remaining_bullet']
            msg.at_center_gain_point = msg_data['at_center_gain_point']
            msg.at_supply_area = msg_data['at_supply_area']
            msg.center_gain_point_status = msg_data['center_gain_point_status']
            msg.supply_area_status_rmul = msg_data['supply_area_status_rmul']
            
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published: {msg_data}')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish: {e}')
            return False

class StatusIndicator(QWidget):
    """状态指示器组件"""
    def __init__(self, label_text, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout()
        self.label = QLabel(label_text)
        self.indicator = QLabel("●")
        self.indicator.setStyleSheet("color: gray")
        self.value_label = QLabel("未激活")
        layout.addWidget(self.indicator)
        layout.addWidget(self.label)
        layout.addWidget(self.value_label)
        layout.addStretch()
        self.setLayout(layout)
        
    def set_status(self, active, value=""):
        if active:
            self.indicator.setStyleSheet("color: green")
            self.value_label.setText(value if value else "激活")
        else:
            self.indicator.setStyleSheet("color: gray")
            self.value_label.setText("未激活")

class ReferenceDataControlPanel(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.init_ui()
        self.setup_timer()
        
    def setup_timer(self):
        """设置定时器，可选自动发布"""
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.publish_data)
        self.auto_publish = False
        
    def init_ui(self):
        self.setWindowTitle('ReferenceDataMini 完整控制面板')
        self.setGeometry(100, 100, 900, 700)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # 创建标签页
        tabs = QTabWidget()
        
        # 添加各个标签页
        tabs.addTab(self.create_game_status_tab(), "游戏状态")
        tabs.addTab(self.create_robot_status_tab(), "机器人状态")
        tabs.addTab(self.create_area_status_tab(), "区域状态")
        tabs.addTab(self.create_supply_status_tab(), "补给区状态")
        tabs.addTab(self.create_battle_status_tab(), "战斗状态")
        tabs.addTab(self.create_status_monitor_tab(), "状态监控")
        
        main_layout.addWidget(tabs)
        
        # 控制按钮区域
        control_layout = QHBoxLayout()
        
        self.publish_btn = QPushButton("立即发布")
        self.publish_btn.clicked.connect(self.publish_data)
        self.publish_btn.setStyleSheet("background-color: #4CAF50; color: white; font-size: 14px; padding: 5px;")
        
        self.auto_publish_btn = QPushButton("开启自动发布")
        self.auto_publish_btn.clicked.connect(self.toggle_auto_publish)
        
        self.publish_rate_spin = QSpinBox()
        self.publish_rate_spin.setRange(1, 50)
        self.publish_rate_spin.setValue(10)
        self.publish_rate_spin.valueChanged.connect(self.update_publish_rate)
        
        rate_label = QLabel("发布频率(Hz):")
        
        control_layout.addWidget(self.publish_btn)
        control_layout.addWidget(self.auto_publish_btn)
        control_layout.addWidget(rate_label)
        control_layout.addWidget(self.publish_rate_spin)
        control_layout.addStretch()
        
        main_layout.addLayout(control_layout)
        
        # 状态栏
        self.status_bar = self.statusBar()
        self.status_label = QLabel("就绪")
        self.status_bar.addWidget(self.status_label)
        
    def create_game_status_tab(self):
        """游戏状态标签页"""
        widget = QWidget()
        layout = QGridLayout(widget)
        
        # 游戏状态
        self.game_state_combo = QComboBox()
        game_states = {
            0: "未开始",
            1: "准备阶段",
            2: "比赛中",
            3: "暂停",
            4: "比赛结束"
        }
        for value, name in game_states.items():
            self.game_state_combo.addItem(name, value)
        
        layout.addWidget(QLabel("游戏状态:"), 0, 0)
        layout.addWidget(self.game_state_combo, 0, 1)
        
        # 剩余时间
        self.remaining_time_spin = self.create_spinbox_with_layout("剩余时间(秒):", 0, 420, 300, layout, 1, 0)
        
        # 己方颜色
        self.self_color_combo = QComboBox()
        self.self_color_combo.addItem("红色", 0)
        self.self_color_combo.addItem("蓝色", 1)
        
        layout.addWidget(QLabel("己方颜色:"), 2, 0)
        layout.addWidget(self.self_color_combo, 2, 1)
        
        # 能量百分比
        self.energy_percent_slider = QSlider(Qt.Horizontal)
        self.energy_percent_slider.setRange(0, 100)
        self.energy_percent_slider.setValue(100)
        self.energy_percent_label = QLabel("100%")
        self.energy_percent_slider.valueChanged.connect(
            lambda v: self.energy_percent_label.setText(f"{v}%"))
        
        layout.addWidget(QLabel("能量百分比:"), 3, 0)
        layout.addWidget(self.energy_percent_slider, 3, 1)
        layout.addWidget(self.energy_percent_label, 3, 2)
        
        # 预留字段
        self.reserved_spin = self.create_spinbox_with_layout("预留字段:", 0, 255, 0, layout, 4, 0)
        
        layout.setRowStretch(5, 1)
        return widget
        
    def create_robot_status_tab(self):
        """机器人状态标签页"""
        widget = QWidget()
        layout = QGridLayout(widget)
        
        # 自身血量（带进度条）
        layout.addWidget(QLabel("自身血量:"), 0, 0)
        self.self_health_spin = QSpinBox()
        self.self_health_spin.setRange(0, 400)
        self.self_health_spin.setValue(400)
        self.self_health_bar = QProgressBar()
        self.self_health_bar.setRange(0, 400)
        self.self_health_spin.valueChanged.connect(self.self_health_bar.setValue)
        
        health_layout = QHBoxLayout()
        health_layout.addWidget(self.self_health_spin)
        health_layout.addWidget(self.self_health_bar)
        layout.addLayout(health_layout, 0, 1, 1, 2)
        
        # 剩余子弹
        self.remaining_bullet_spin = self.create_spinbox_with_layout("剩余子弹:", 0, 500, 200, layout, 1, 0)
        
        # 枪管热度
        self.shoot_heat_spin = self.create_spinbox_with_layout("枪管热度:", 0, 1000, 0, layout, 2, 0)
        
        # 自动瞄准状态
        self.auto_aiming_check = QCheckBox("自动瞄准激活")
        layout.addWidget(self.auto_aiming_check, 3, 0, 1, 2)
        
        layout.setRowStretch(4, 1)
        return widget
        
    def create_area_status_tab(self):
        """区域状态标签页"""
        widget = QWidget()
        layout = QGridLayout(widget)
        
        # 各个区域状态
        self.at_base_check = QCheckBox("在基地区域")
        self.at_castle_check = QCheckBox("在城堡区域")
        self.at_center_gain_spin = self.create_spinbox_with_layout("中心增益点数值:", 0, 1000, 0, layout, 1, 0)
        self.at_supply_spin = self.create_spinbox_with_layout("补给区数值:", 0, 1000, 0, layout, 2, 0)
        
        layout.addWidget(self.at_base_check, 0, 0)
        layout.addWidget(self.at_castle_check, 0, 1)
        
        # 中心增益点状态
        self.center_gain_status_combo = QComboBox()
        center_status = {
            0: "未激活",
            1: "激活中(己方)",
            2: "激活中(敌方)",
            3: "已占领(己方)",
            4: "已占领(敌方)"
        }
        for value, name in center_status.items():
            self.center_gain_status_combo.addItem(name, value)
            
        layout.addWidget(QLabel("中心增益点状态:"), 3, 0)
        layout.addWidget(self.center_gain_status_combo, 3, 1)
        
        layout.setRowStretch(4, 1)
        return widget
        
    def create_supply_status_tab(self):
        """补给区状态标签页"""
        widget = QWidget()
        layout = QGridLayout(widget)
        
        # 补给区状态（不重叠）
        self.supply_unoverlap_check = QCheckBox("在补给区(不重叠)")
        layout.addWidget(self.supply_unoverlap_check, 0, 0)
        
        # 补给区状态（重叠）
        self.supply_overlap_check = QCheckBox("在补给区(重叠)")
        layout.addWidget(self.supply_overlap_check, 0, 1)
        
        # 补给区状态值
        self.supply_status_spin = self.create_spinbox_with_layout("补给区状态值:", 0, 100, 0, layout, 1, 0)
        self.supply_status_rmul_spin = self.create_spinbox_with_layout("补给区状态(RMUL):", 0, 100, 0, layout, 2, 0)
        
        layout.setRowStretch(3, 1)
        return widget
        
    def create_battle_status_tab(self):
        """战斗状态标签页"""
        widget = QWidget()
        layout = QGridLayout(widget)
        
        # 快速预设按钮
        preset_group = QGroupBox("快速预设")
        preset_layout = QHBoxLayout()
        
        presets = {
            "比赛开始": self.set_match_start,
            "满状态": self.set_full_status,
            "低血量": self.set_low_health,
            "补给中": self.set_supplying,
            "占领中心": self.set_center_occupied
        }
        
        for name, func in presets.items():
            btn = QPushButton(name)
            btn.clicked.connect(func)
            preset_layout.addWidget(btn)
            
        preset_group.setLayout(preset_layout)
        layout.addWidget(preset_group, 0, 0, 1, 2)
        
        layout.setRowStretch(1, 1)
        return widget
        
    def create_status_monitor_tab(self):
        """状态监控标签页"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # 状态指示器
        self.status_indicators = {
            'game': StatusIndicator("游戏进行中"),
            'auto_aim': StatusIndicator("自动瞄准"),
            'base': StatusIndicator("在基地"),
            'castle': StatusIndicator("在城堡"),
            'supply_unoverlap': StatusIndicator("补给区(不重叠)"),
            'supply_overlap': StatusIndicator("补给区(重叠)"),
            'center_gain': StatusIndicator("中心增益点")
        }
        
        for indicator in self.status_indicators.values():
            layout.addWidget(indicator)
            
        # 实时数据显示
        self.realtime_text = QTextEdit()
        self.realtime_text.setReadOnly(True)
        self.realtime_text.setMaximumHeight(200)
        layout.addWidget(QLabel("实时数据:"))
        layout.addWidget(self.realtime_text)
        
        return widget
        
    def create_spinbox_with_layout(self, label_text, min_val, max_val, default_val, layout, row, col):
        """创建带标签的spinbox"""
        label = QLabel(label_text)
        spinbox = QSpinBox()
        spinbox.setRange(min_val, max_val)
        spinbox.setValue(default_val)
        layout.addWidget(label, row, col)
        layout.addWidget(spinbox, row, col + 1)
        return spinbox
        
    def get_current_data(self):
        """获取当前所有数据"""
        return {
            'game_state': self.game_state_combo.currentData(),
            'remaining_time': self.remaining_time_spin.value(),
            'self_color': self.self_color_combo.currentData(),
            'self_health': self.self_health_spin.value(),
            'at_base_area': 1 if self.at_base_check.isChecked() else 0,
            'at_castle_area': 1 if self.at_castle_check.isChecked() else 0,
            'at_supply_area_unoverlap': 1 if self.supply_unoverlap_check.isChecked() else 0,
            'at_supply_area_overlap': 1 if self.supply_overlap_check.isChecked() else 0,
            'supply_area_status': self.supply_status_spin.value(),
            'reserved': self.reserved_spin.value(),
            'auto_aiming_status': 1 if self.auto_aiming_check.isChecked() else 0,
            'energy_percent': self.energy_percent_slider.value(),
            'shoot_heat': self.shoot_heat_spin.value(),
            'remaining_bullet': self.remaining_bullet_spin.value(),
            'at_center_gain_point': self.at_center_gain_spin.value(),
            'at_supply_area': self.at_supply_spin.value(),
            'center_gain_point_status': self.center_gain_status_combo.currentData(),
            'supply_area_status_rmul': self.supply_status_rmul_spin.value()
        }
        
    def update_monitor_display(self, data):
        """更新监控显示"""
        # 更新状态指示器
        self.status_indicators['game'].set_status(data['game_state'] == 2, "比赛中")
        self.status_indicators['auto_aim'].set_status(data['auto_aiming_status'] == 1)
        self.status_indicators['base'].set_status(data['at_base_area'] == 1)
        self.status_indicators['castle'].set_status(data['at_castle_area'] == 1)
        self.status_indicators['supply_unoverlap'].set_status(data['at_supply_area_unoverlap'] == 1)
        self.status_indicators['supply_overlap'].set_status(data['at_supply_area_overlap'] == 1)
        self.status_indicators['center_gain'].set_status(data['center_gain_point_status'] > 0)
        
        # 更新实时文本
        timestamp = QDateTime.currentDateTime().toString("hh:mm:ss.zzz")
        text = f"[{timestamp}] 游戏状态:{data['game_state']} | 血量:{data['self_health']} | "
        text += f"子弹:{data['remaining_bullet']} | 能量:{data['energy_percent']}% | "
        text += f"热度:{data['shoot_heat']}\n"
        
        self.realtime_text.moveCursor(QTextCursor.End)
        self.realtime_text.insertPlainText(text)
        # 限制显示行数
        if self.realtime_text.document().blockCount() > 100:
            cursor = self.realtime_text.textCursor()
            cursor.movePosition(QTextCursor.Start)
            cursor.select(QTextCursor.LineUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()
        
    def publish_data(self):
        """发布数据"""
        data = self.get_current_data()
        success = self.ros_node.publish_message(data)
        
        if success:
            self.update_monitor_display(data)
            self.status_label.setText(f"✓ 数据已发布 - {QDateTime.currentDateTime().toString('hh:mm:ss')}")
            self.status_label.setStyleSheet("color: green")
        else:
            self.status_label.setText("✗ 发布失败")
            self.status_label.setStyleSheet("color: red")
            
        # 3秒后恢复状态显示
        QTimer.singleShot(3000, lambda: self.status_label.setStyleSheet(""))
            
    def toggle_auto_publish(self):
        """切换自动发布"""
        if self.auto_publish:
            self.timer.stop()
            self.auto_publish = False
            self.auto_publish_btn.setText("开启自动发布")
            self.status_label.setText("自动发布已关闭")
        else:
            self.timer.start(1000 // self.publish_rate_spin.value())
            self.auto_publish = True
            self.auto_publish_btn.setText("关闭自动发布")
            self.status_label.setText("自动发布已开启")
            
    def update_publish_rate(self, rate):
        """更新发布频率"""
        if self.auto_publish:
            self.timer.stop()
            self.timer.start(1000 // rate)
            
    # 预设场景函数
    def set_match_start(self):
        """比赛开始预设"""
        self.game_state_combo.setCurrentIndex(2)  # 比赛中
        self.remaining_time_spin.setValue(300)
        self.self_health_spin.setValue(400)
        self.remaining_bullet_spin.setValue(200)
        self.energy_percent_slider.setValue(100)
        
    def set_full_status(self):
        """满状态预设"""
        self.self_health_spin.setValue(400)
        self.remaining_bullet_spin.setValue(500)
        self.energy_percent_slider.setValue(100)
        self.shoot_heat_spin.setValue(0)
        
    def set_low_health(self):
        """低血量预设"""
        self.self_health_spin.setValue(50)
        self.remaining_bullet_spin.setValue(30)
        self.energy_percent_slider.setValue(20)
        
    def set_supplying(self):
        """补给中预设"""
        self.supply_unoverlap_check.setChecked(True)
        self.supply_status_spin.setValue(50)
        self.auto_aiming_check.setChecked(False)
        
    def set_center_occupied(self):
        """占领中心预设"""
        self.at_center_gain_spin.setValue(100)
        self.center_gain_status_combo.setCurrentIndex(3)  # 已占领(己方)

def main(args=None):
    rclpy.init(args=args)
    
    app = QApplication(sys.argv)
    app.setStyle('Fusion')  # 使用Fusion风格，更现代
    
    try:
        publisher_node = ReferenceDataMiniPublisher()
        control_panel = ReferenceDataControlPanel(publisher_node)
        control_panel.show()
        
        # 使用Qt定时器处理ROS2事件
        ros_timer = QTimer()
        ros_timer.timeout.connect(lambda: rclpy.spin_once(publisher_node, timeout_sec=0))
        ros_timer.start(10)  # 100Hz
        
        sys.exit(app.exec_())
        
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"程序出错: {e}")
    finally:
        if 'publisher_node' in locals():
            publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
