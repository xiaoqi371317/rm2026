#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QLabel, QDoubleSpinBox, QGroupBox,
                            QPushButton, QComboBox)
from PyQt5.QtCore import QTimer

class TwistPublisher(Node):
    def __init__(self):
        super().__init__('twist_publisher_gui')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publish_rate = 50  # Hz
        
    def publish_message(self, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.linear.z = linear_z
        msg.angular.x = angular_x
        msg.angular.y = angular_y
        msg.angular.z = angular_z
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg}')

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
        self.setWindowTitle('Twist 消息发布器')
        self.setGeometry(100, 100, 600, 400)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # Linear Velocity Control
        linear_group = QGroupBox("线速度 (m/s)")
        linear_layout = QVBoxLayout()
        
        self.linear_x_spin = self.create_double_spinbox("X方向:", -5.0, 5.0, 0.0, 0.1)
        self.linear_y_spin = self.create_double_spinbox("Y方向:", -5.0, 5.0, 0.0, 0.1)
        self.linear_z_spin = self.create_double_spinbox("Z方向:", -5.0, 5.0, 0.0, 0.1)
        
        linear_layout.addLayout(self.linear_x_spin)
        linear_layout.addLayout(self.linear_y_spin)
        linear_layout.addLayout(self.linear_z_spin)
        linear_group.setLayout(linear_layout)
        
        # Angular Velocity Control
        angular_group = QGroupBox("角速度 (rad/s)")
        angular_layout = QVBoxLayout()
        
        self.angular_x_spin = self.create_double_spinbox("X方向:", -5.0, 5.0, 0.0, 0.1)
        self.angular_y_spin = self.create_double_spinbox("Y方向:", -5.0, 5.0, 0.0, 0.1)
        self.angular_z_spin = self.create_double_spinbox("Z方向:", -5.0, 5.0, 0.0, 0.1)
        
        angular_layout.addLayout(self.angular_x_spin)
        angular_layout.addLayout(self.angular_y_spin)
        angular_layout.addLayout(self.angular_z_spin)
        angular_group.setLayout(angular_layout)
        
        # Publish Rate Control
        rate_group = QGroupBox("发布频率")
        rate_layout = QHBoxLayout()
        
        rate_label = QLabel("频率 (Hz):")
        self.rate_combo = QComboBox()
        self.rate_combo.addItems(["10", "20", "30", "40", "50", "60"])
        self.rate_combo.setCurrentText("50")
        self.rate_combo.currentTextChanged.connect(self.update_publish_rate)
        
        rate_layout.addWidget(rate_label)
        rate_layout.addWidget(self.rate_combo)
        rate_layout.addStretch()
        rate_group.setLayout(rate_layout)
        
        # Add all groups to main layout
        main_layout.addWidget(linear_group)
        main_layout.addWidget(angular_group)
        main_layout.addWidget(rate_group)
        
    def create_double_spinbox(self, label_text, min_val, max_val, default_val, step):
        layout = QHBoxLayout()
        label = QLabel(label_text)
        spinbox = QDoubleSpinBox()
        spinbox.setRange(min_val, max_val)
        spinbox.setValue(default_val)
        spinbox.setSingleStep(step)
        spinbox.setDecimals(3)
        layout.addWidget(label)
        layout.addWidget(spinbox)
        return layout
        
    def get_double_spinbox_value(self, spinbox_layout):
        return spinbox_layout.itemAt(1).widget().value()
        
    def update_publish_rate(self):
        new_rate = int(self.rate_combo.currentText())
        self.ros_node.publish_rate = new_rate
        self.timer.setInterval(int(1000 / new_rate))
        
    def publish_data(self):
        linear_x = self.get_double_spinbox_value(self.linear_x_spin)
        linear_y = self.get_double_spinbox_value(self.linear_y_spin)
        linear_z = self.get_double_spinbox_value(self.linear_z_spin)
        angular_x = self.get_double_spinbox_value(self.angular_x_spin)
        angular_y = self.get_double_spinbox_value(self.angular_y_spin)
        angular_z = self.get_double_spinbox_value(self.angular_z_spin)
        
        self.ros_node.publish_message(linear_x, linear_y, linear_z, 
                                     angular_x, angular_y, angular_z)

def main(args=None):
    rclpy.init(args=args)
    
    app = QApplication([])
    
    publisher_node = TwistPublisher()
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
