#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QLabel, QGroupBox, QPushButton,
                            QComboBox, QButtonGroup, QRadioButton)
from PyQt5.QtCore import QTimer

class PosturePublisher(Node):
    def __init__(self):
        super().__init__('posture_publisher_gui')
        self.publisher_ = self.create_publisher(UInt8, '/sentry/status/posture', 10)
        self.publish_rate = 10  # Hz
        self.current_value = 1  # 初始姿态值 (1, 2, 或 3)
        
    def publish_message(self, value):
        msg = UInt8()
        msg.data = value
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing posture: {msg.data}')

class PostureControlPanel(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.init_ui()
        
        # Set up a timer to periodically publish the data
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.publish_data)
        self.timer.start(int(1000 / self.ros_node.publish_rate))  # Convert Hz to ms
        
    def init_ui(self):
        self.setWindowTitle('哨兵姿态发布器 - /sentry/status/posture')
        self.setGeometry(100, 100, 400, 300)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # Posture Value Control
        posture_group = QGroupBox("姿态值控制 (范围: 1, 2, 3)")
        posture_layout = QVBoxLayout()
        
        # Current status display
        self.status_label = QLabel("当前姿态: 1")
        self.status_label.setStyleSheet("font-size: 16px; font-weight: bold; color: blue;")
        
        # Radio buttons for direct selection
        radio_layout = QHBoxLayout()
        self.radio_group = QButtonGroup()
        
        self.radio_1 = QRadioButton("姿态 1")
        self.radio_2 = QRadioButton("姿态 2")
        self.radio_3 = QRadioButton("姿态 3")
        
        self.radio_1.toggled.connect(lambda: self.set_value(1))
        self.radio_2.toggled.connect(lambda: self.set_value(2))
        self.radio_3.toggled.connect(lambda: self.set_value(3))
        
        self.radio_group.addButton(self.radio_1, 1)
        self.radio_group.addButton(self.radio_2, 2)
        self.radio_group.addButton(self.radio_3, 3)
        
        radio_layout.addWidget(self.radio_1)
        radio_layout.addWidget(self.radio_2)
        radio_layout.addWidget(self.radio_3)
        
        # Set initial selection
        self.radio_1.setChecked(True)
        
        # Increment/Decrement buttons
        button_layout = QHBoxLayout()
        self.decrement_button = QPushButton("- 减小姿态")
        self.increment_button = QPushButton("+ 增大姿态")
        self.decrement_button.clicked.connect(self.decrement_value)
        self.increment_button.clicked.connect(self.increment_value)
        
        button_layout.addWidget(self.decrement_button)
        button_layout.addWidget(self.increment_button)
        
        posture_layout.addWidget(self.status_label)
        posture_layout.addLayout(radio_layout)
        posture_layout.addLayout(button_layout)
        posture_group.setLayout(posture_layout)
        
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
        main_layout.addWidget(posture_group)
        main_layout.addWidget(rate_group)
        
        # Set initial value
        self.current_value = 1
        
    def set_value(self, value):
        """Set the posture value (must be 1, 2, or 3)"""
        if value in [1, 2, 3]:
            self.current_value = value
            self.update_ui()
        
    def increment_value(self):
        """Increase posture value (1->2->3->1)"""
        new_value = self.current_value + 1
        if new_value > 3:
            new_value = 1
        self.set_value(new_value)
        # Update radio button to match
        self.update_radio_buttons()
        
    def decrement_value(self):
        """Decrease posture value (1->3->2->1)"""
        new_value = self.current_value - 1
        if new_value < 1:
            new_value = 3
        self.set_value(new_value)
        # Update radio button to match
        self.update_radio_buttons()
        
    def update_radio_buttons(self):
        """Synchronize radio buttons with current value"""
        if self.current_value == 1:
            self.radio_1.setChecked(True)
        elif self.current_value == 2:
            self.radio_2.setChecked(True)
        elif self.current_value == 3:
            self.radio_3.setChecked(True)
        
    def update_ui(self):
        """Update UI based on current value"""
        posture_names = {1: "1 (常规)", 2: "2 (警戒)", 3: "3 (攻击)"}
        posture_text = posture_names.get(self.current_value, str(self.current_value))
        
        self.status_label.setText(f"当前姿态: {posture_text}")
        
        # Color coding based on posture
        if self.current_value == 1:
            color = "blue"
        elif self.current_value == 2:
            color = "orange"
        else:  # 3
            color = "red"
            
        self.status_label.setStyleSheet(f"font-size: 16px; font-weight: bold; color: {color};")
        
    def update_publish_rate(self):
        """Update the publishing frequency"""
        new_rate = int(self.rate_combo.currentText())
        self.ros_node.publish_rate = new_rate
        self.timer.setInterval(int(1000 / new_rate))
        
    def publish_data(self):
        """Publish the current posture value"""
        self.ros_node.publish_message(self.current_value)

def main(args=None):
    rclpy.init(args=args)
    
    app = QApplication([])
    
    publisher_node = PosturePublisher()
    control_panel = PostureControlPanel(publisher_node)
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
