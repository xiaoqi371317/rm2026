#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QLabel, QGroupBox, QPushButton,
                            QComboBox)
from PyQt5.QtCore import QTimer

class BoolPublisher(Node):
    def __init__(self):
        super().__init__('bool_publisher_gui')
        self.publisher_ = self.create_publisher(Bool, '/if_follow_yaw', 10)
        self.publish_rate = 10  # Hz
        self.current_value = False
        
    def publish_message(self, value):
        msg = Bool()
        msg.data = value
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

class ControlPanel(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.init_ui()
        
        # Set up a timer to periodically publish the data
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.publish_data)
        self.timer.start(int(1000 / self.ros_node.publish_rate))  # Convert Hz to ms
        
    def init_ui(self):
        self.setWindowTitle('Bool 消息发布器 - /if_follow_yaw')
        self.setGeometry(100, 100, 400, 250)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # Bool Value Control
        bool_group = QGroupBox("Bool 值控制")
        bool_layout = QVBoxLayout()
        
        # Current status display
        self.status_label = QLabel("当前值: False")
        self.status_label.setStyleSheet("font-size: 16px; font-weight: bold;")
        
        # Toggle button
        self.toggle_button = QPushButton("切换为 True")
        self.toggle_button.clicked.connect(self.toggle_value)
        self.toggle_button.setStyleSheet("font-size: 14px; padding: 10px;")
        
        # Direct set buttons
        button_layout = QHBoxLayout()
        self.set_true_button = QPushButton("设置为 True")
        self.set_false_button = QPushButton("设置为 False")
        self.set_true_button.clicked.connect(lambda: self.set_value(True))
        self.set_false_button.clicked.connect(lambda: self.set_value(False))
        
        button_layout.addWidget(self.set_true_button)
        button_layout.addWidget(self.set_false_button)
        
        bool_layout.addWidget(self.status_label)
        bool_layout.addWidget(self.toggle_button)
        bool_layout.addLayout(button_layout)
        bool_group.setLayout(bool_layout)
        
        # Publish Rate Control
        rate_group = QGroupBox("发布频率")
        rate_layout = QHBoxLayout()
        
        rate_label = QLabel("频率 (Hz):")
        self.rate_combo = QComboBox()
        self.rate_combo.addItems(["1", "5", "10", "20", "30", "50"])
        self.rate_combo.setCurrentText("10")
        self.rate_combo.currentTextChanged.connect(self.update_publish_rate)
        
        rate_layout.addWidget(rate_label)
        rate_layout.addWidget(self.rate_combo)
        rate_layout.addStretch()
        rate_group.setLayout(rate_layout)
        
        # Add all groups to main layout
        main_layout.addWidget(bool_group)
        main_layout.addWidget(rate_group)
        
        # Set initial value
        self.current_value = False
        
    def toggle_value(self):
        """Toggle between True and False"""
        new_value = not self.current_value
        self.set_value(new_value)
        
    def set_value(self, value):
        """Set the bool value"""
        self.current_value = value
        self.update_ui()
        
    def update_ui(self):
        """Update UI based on current value"""
        if self.current_value:
            self.status_label.setText("当前值: True")
            self.status_label.setStyleSheet("font-size: 16px; font-weight: bold; color: green;")
            self.toggle_button.setText("切换为 False")
        else:
            self.status_label.setText("当前值: False")
            self.status_label.setStyleSheet("font-size: 16px; font-weight: bold; color: red;")
            self.toggle_button.setText("切换为 True")
        
    def update_publish_rate(self):
        new_rate = int(self.rate_combo.currentText())
        self.ros_node.publish_rate = new_rate
        self.timer.setInterval(int(1000 / new_rate))
        
    def publish_data(self):
        self.ros_node.publish_message(self.current_value)

def main(args=None):
    rclpy.init(args=args)
    
    app = QApplication([])
    
    publisher_node = BoolPublisher()
    control_panel = ControlPanel(publisher_node)
    control_panel.show()
    
    # Use a timer to spin the ROS node
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(publisher_node, timeout_sec=0.1))
    timer.start(100)
    
    app.exec_()
    
    publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
