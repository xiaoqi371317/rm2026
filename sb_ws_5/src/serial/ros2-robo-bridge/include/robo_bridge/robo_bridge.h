#pragma once

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "CSerialPort/SerialPort.h"
#include "CSerialPort/SerialPortInfo.h"

#include "robo_utils/msg/chassis_vel_command.hpp"
#include "robo_utils/msg/chassis_vel_yaw_rate_command.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>

class MyListener;

using namespace itas109;

namespace olk
{
    class Subscriber;
    class IMUData;
    class CommandData;
    class Publisher;
    class GimbalConstantAimShootCommand;
    class ChassisVelYawRateCommand;
    class ChassisVelCommand;
    class FollowYawCommand;
    class SentryPostureCommand;
    class ReferenceDataMini;
};

class RoboBridge
{
    public:
    RoboBridge();
    void init(rclcpp::Node *node, int baudrate = 5000000);
    // void init(rclcpp::Node *node, int baudrate = 921600);
    void write(uint8_t *data, int size);
    void process_byte(uint8_t data);
    // void chassis_vel_cmd_callback(robo_utils::msg::ChassisVelCommand::ConstSharedPtr msg_p);
    void chassis_vel_cmd_callback(geometry_msgs::msg::Twist::SharedPtr msg_p);
    void chassis_vel_yaw_rate_cmd_callback(geometry_msgs::msg::Twist::SharedPtr msg_p);
    void follow_yaw_cmd_callback(std_msgs::msg::Bool::SharedPtr msg_p);
    void posture_callback(std_msgs::msg::UInt8::SharedPtr msg);
    ~RoboBridge();

    rclcpp::Node *node;

    protected:
    CSerialPort sp;
    MyListener *listener_p;
    olk::Subscriber *subscriber_imu_p, *subscriber_ref_p, *subscriber_cmd_p;
    olk::Publisher *publisher_chassis_vel_p;
    olk::Publisher *publisher_status_p;
    olk::SentryPostureCommand *posture_cmd_p;

    olk::IMUData *imu_data_p;
    olk::CommandData *cmd_data_p;
    olk::ReferenceDataMini *ref_data_p;
    olk::ChassisVelCommand *chassis_vel_cmd_p;
    olk::ChassisVelYawRateCommand *chassis_vel_yaw_rate_cmd_p;
    olk::FollowYawCommand *follow_yaw_cmd_p;

    // rclcpp::Subscription<robo_utils::msg::ChassisVelCommand>::SharedPtr subscriber_chassis_vel;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_posture_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_chassis_vel;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_chassis_vel_yaw_rate;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_follow_yaw;
};
