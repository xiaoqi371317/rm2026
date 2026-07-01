#include "robo_bridge/robo_bridge.h"
#include "robo_bridge/my_listener.h"
#include "robo_bridge/OffboardLink.h"
#include "robo_bridge/msgs/IMUData.h"
#include "robo_bridge/msgs/CommandData.h"
#include "robo_bridge/msgs/ReferenceDataMini.h"
#include "robo_bridge/msgs/ChassisVelCommand.h"
#include "robo_bridge/msgs/ChassisVelYawRateCommand.h"
#include "robo_bridge/msgs/FollowYawCommand.h"
#include "robo_bridge/msgs/SentryPostureCommand.h"

// test
#include <cstring>
#include <cmath>


// self add
#include <geometry_msgs/msg/twist.hpp>

RoboBridge::RoboBridge()
{
    subscriber_imu_p = new olk::Subscriber();
    subscriber_ref_p = new olk::Subscriber();
    subscriber_cmd_p = new olk::Subscriber();
    publisher_chassis_vel_p = new olk::Publisher();
    publisher_status_p = new olk::Publisher();
    imu_data_p = new olk::IMUData();
    ref_data_p = new olk::ReferenceDataMini();
    cmd_data_p = new olk::CommandData();
    chassis_vel_cmd_p = new olk::ChassisVelCommand();
    chassis_vel_yaw_rate_cmd_p = new olk::ChassisVelYawRateCommand();
    follow_yaw_cmd_p = new olk::FollowYawCommand();
    posture_cmd_p = new olk::SentryPostureCommand();
}

// void RoboBridge::chassis_vel_cmd_callback(robo_utils::msg::ChassisVelCommand::ConstSharedPtr msg_p)
// {
//     chassis_vel_cmd_p->x = msg_p->x;
//     chassis_vel_cmd_p->y = msg_p->y;
//     publisher_chassis_vel_p->publish(chassis_vel_cmd_p);
// }
void RoboBridge::chassis_vel_cmd_callback(geometry_msgs::msg::Twist::SharedPtr msg_p)
{
    chassis_vel_cmd_p->x = msg_p->linear.x;
    chassis_vel_cmd_p->y = msg_p->linear.y;
    // 打印 x 和 y 方向的速度
    RCLCPP_INFO(rclcpp::get_logger("robo_bridge"), "Received x velocity: %.2f", chassis_vel_cmd_p->x);
    RCLCPP_INFO(rclcpp::get_logger("robo_bridge"), "Received y velocity: %.2f", chassis_vel_cmd_p->y);
    publisher_chassis_vel_p->publish(chassis_vel_cmd_p);
}

void RoboBridge::chassis_vel_yaw_rate_cmd_callback(geometry_msgs::msg::Twist::SharedPtr msg_p)
{
    
    // chassis_vel_yaw_rate_cmd_p->x = -1 * msg_p->linear.x;
    // chassis_vel_yaw_rate_cmd_p->y = msg_p->linear.y;
    // TODO
    chassis_vel_yaw_rate_cmd_p->x =  -1 * msg_p->linear.x;
    chassis_vel_yaw_rate_cmd_p->y =  -1 * msg_p->linear.y;
    chassis_vel_yaw_rate_cmd_p->yaw_rate = msg_p->angular.z;
    //chassis_vel_yaw_rate_cmd_p->yaw_rate = 0.0;

    // 打印 x 和 y 方向的速度
    RCLCPP_INFO(rclcpp::get_logger("robo_bridge_yaw"), "Received x velocity: %.2f", msg_p->linear.x);
    RCLCPP_INFO(rclcpp::get_logger("robo_bridge_yaw"), "Received y velocity: %.2f", msg_p->linear.y);
    RCLCPP_INFO(rclcpp::get_logger("robo_bridge_yaw"), "Received yaw angle velocity: %.2f", msg_p->angular.z);
    publisher_chassis_vel_p->publish(chassis_vel_yaw_rate_cmd_p);
}

void RoboBridge::follow_yaw_cmd_callback(std_msgs::msg::Bool::SharedPtr msg_p)
{
    follow_yaw_cmd_p->if_follow_yaw = msg_p->data ? 1 : 0;
    RCLCPP_INFO(rclcpp::get_logger("robo_bridge_follow_yaw"), "Received if_follow_yaw: %d", follow_yaw_cmd_p->if_follow_yaw);
    publisher_chassis_vel_p->publish(follow_yaw_cmd_p);
}

void RoboBridge::posture_callback(std_msgs::msg::UInt8::SharedPtr msg)
{
    uint8_t p = msg->data;
    if(p < 1 || p > 3)
    {
        RCLCPP_WARN(node->get_logger(), "Invalid posture=%u (expect 1..3)", p);
        return;
    }
    RCLCPP_INFO(node->get_logger(), "The posture=%u", p);
    posture_cmd_p->posture = p;
    publisher_status_p->publish(posture_cmd_p);
}


void RoboBridge::init(rclcpp::Node *node, int baudrate)
{
    this->node = node;
    // subscriber_chassis_vel = node->create_subscription<robo_utils::msg::ChassisVelCommand>(
    //         "/offboardlink/chassis_vel_cmd", 1,
    //         std::bind(&RoboBridge::chassis_vel_cmd_callback, this, std::placeholders::_1));
    subscriber_chassis_vel = node->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_Test", 1,
            std::bind(&RoboBridge::chassis_vel_cmd_callback, this, std::placeholders::_1));
    subscriber_chassis_vel_yaw_rate = node->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 1,
            std::bind(&RoboBridge::chassis_vel_yaw_rate_cmd_callback, this, std::placeholders::_1));
    subscriber_follow_yaw = node->create_subscription<std_msgs::msg::Bool>(
            "/if_follow_yaw", 1,
            std::bind(&RoboBridge::follow_yaw_cmd_callback, this, std::placeholders::_1));
    sub_posture_ = node->create_subscription<std_msgs::msg::UInt8>(
    	"/sentry/status/posture", 10,
    std::bind(&RoboBridge::posture_callback, this, std::placeholders::_1));

    listener_p = new MyListener(&sp);
    std::vector<SerialPortInfo> m_availablePortsList = CSerialPortInfo::availablePortInfos();

    if (0 == m_availablePortsList.size())
    {
        RCLCPP_INFO_STREAM(node->get_logger(), "No valid port");
        return;
    }
    
    SerialPortInfo available_port = m_availablePortsList[0];
    for(int i = 0; i < m_availablePortsList.size(); i ++)
    {
        if(strstr(m_availablePortsList[i].portName, "/dev/ttyUSB") != NULL)
        {
            available_port = m_availablePortsList[i];
        }
    }
    
    sp.init(available_port.portName, // windows:COM1 Linux:/dev/ttyS0
            baudrate,// 
            itas109::ParityNone, 
            itas109::DataBits8, 
            itas109::StopOne);

    sp.setReadIntervalTimeout(0); // read interval timeout 0ms

    sp.open();

    if (sp.isOpen())
    {
        RCLCPP_INFO_STREAM(node->get_logger(), "open " << available_port.portName << " success");
    }
    else
    {
        RCLCPP_INFO_STREAM(node->get_logger(), "open " << available_port.portName << " failed");
        return;
    }

    subscriber_imu_p->subscribe(0x01, imu_data_p);
    subscriber_ref_p->subscribe(0x40, ref_data_p);
    subscriber_cmd_p->subscribe(0x60, cmd_data_p);

    sp.connectReadEvent(listener_p);
}

void RoboBridge::write(uint8_t *data, int size)
{
    sp.writeData(data, size);
}

void RoboBridge::process_byte(uint8_t data)
{
    // ROS_INFO("0x%02x", data);
    subscriber_imu_p->processByte(data);
    subscriber_ref_p->processByte(data);
    subscriber_cmd_p->processByte(data);
}

RoboBridge::~RoboBridge()
{
    sp.close();
    delete listener_p;
    delete imu_data_p;
    delete ref_data_p;
    delete cmd_data_p;
    delete chassis_vel_cmd_p;
    delete chassis_vel_yaw_rate_cmd_p;
    delete follow_yaw_cmd_p;
    delete publisher_chassis_vel_p;
    delete subscriber_ref_p;
    delete subscriber_imu_p;
    delete posture_cmd_p;
    delete subscriber_cmd_p;
}
