#pragma once

#include "robo_bridge/OffboardLink.h"

#ifdef DEBUG_MODE
#include "UARTDriver.h"
#endif

#ifdef ON_MINIPC
#include <sensor_msgs/msg/imu.hpp>
#include "robo_bridge/robo_bridge.h"
extern RoboBridge bridge;
#endif

namespace olk
{
	struct IMUData : public MessageBase
	{
		public:
		int16_t accel[3]; // 24g
		int16_t gyro[3]; // 2000dps
		float w;
		float x;
		float y;
		float z;

		uint32_t test;

		#ifdef ON_MINIPC
		rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
		#endif
			
		IMUData() : MessageBase(0x01, 26)
		{
			#ifdef ON_MINIPC
			test = 0;
			// imu_pub = bridge.node_handle->advertise<sensor_msgs::Imu>("/offboardlink/imu_data", 1);  
			#endif
		}

		virtual void init(void)
		{
			#ifdef ON_MINIPC
			imu_pub = bridge.node->create_publisher<sensor_msgs::msg::Imu>("/offboardlink/imu_data", 1);  
			#endif
		}
			
		virtual void decode(uint8_t *buffer) override
		{
			for(int i = 0; i < 3; i ++)
            {
                accel[i] = (int16_t)((buffer[5 + 2 * i] << 8) | (buffer[4 + 2 * i]));// * 0.00718260498046875f;
            }
            for(int i = 0; i < 3; i ++)
            {
                gyro[i] = (int16_t)((buffer[11 + 2 * i] << 8) | (buffer[10 + 2 * i]));// * 0.00106526443603169529841533860381f;
            }
			float *quat[4] = {&w, &x, &y, &z};
            for(int i = 0; i < 4; i ++)
            {
                *(quat[i]) =  (int16_t)((buffer[17 + 2 * i] << 8) | (buffer[16 + 2 * i])) / 32760.0;
            }
			
			#ifdef ON_MINIPC
			sensor_msgs::msg::Imu imu_data;
            imu_data.header.stamp = bridge.node->now();
            imu_data.header.frame_id = "cam_earth";
            imu_data.orientation.x = x;
            imu_data.orientation.y = y;
            imu_data.orientation.z = z;
            imu_data.orientation.w = w;
            imu_data.linear_acceleration.x = -accel[0] * 0.00718260498046875f; 
            imu_data.linear_acceleration.y = -accel[1] * 0.00718260498046875f;
            imu_data.linear_acceleration.z = accel[2] * 0.00718260498046875f;
            imu_data.angular_velocity.x = -gyro[0] * 0.00106526443603169529841533860381f; 
            imu_data.angular_velocity.y = -gyro[1] * 0.00106526443603169529841533860381f; 
            imu_data.angular_velocity.z = gyro[2] * 0.00106526443603169529841533860381f;
            imu_pub->publish(imu_data);
// ROS_INFO("test %d", test++);
			#endif
		}
			
		virtual void packData(uint8_t *buf) override
		{
			int16_t quaternion[4];
			quaternion[0] = w * 32760.0f;
			quaternion[1] = x * 32760.0f;
			quaternion[2] = y * 32760.0f;
			quaternion[3] = z * 32760.0f;
			
			buf[4]  = accel[0];
			buf[5]  = accel[0] >> 8;
			buf[6]  = accel[1];
			buf[7]  = accel[1] >> 8;
			buf[8]  = accel[2];
			buf[9]  = accel[2] >> 8;

			buf[10] = gyro[0];
			buf[11] = gyro[0] >> 8;
			buf[12] = gyro[1];
			buf[13] = gyro[1] >> 8;
			buf[14] = gyro[2];
			buf[15] = gyro[2] >> 8;

			buf[16] = quaternion[0];
			buf[17] = quaternion[0] >> 8;
			buf[18] = quaternion[1];
			buf[19] = quaternion[1] >> 8;
			buf[20] = quaternion[2];
			buf[21] = quaternion[2] >> 8;
			buf[22] = quaternion[3];
			buf[23] = quaternion[3] >> 8;			
		}
	};
}
