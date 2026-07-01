#pragma once

#include "robo_bridge/OffboardLink.h"

#ifdef DEBUG_MODE
#include "UARTDriver.h"
#endif

#ifdef ON_MINIPC
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robo_utils/msg/reference_data_mini.hpp>
#include "robo_bridge/robo_bridge.h"
extern RoboBridge bridge;
#endif

namespace olk
{
	struct ReferenceDataMini : MessageBase
	{
		public:
        uint8_t game_state;      // 0-not_start 1-rmuc 2-rmul_3x3 3rmul_1x1
        uint16_t remaining_time;  
        uint8_t self_color;      // 1-red 2-blue
        uint16_t self_health;    
        float enermy_outpost_hp;
        float my_outpost_hp;
        float enermy_base_hp;
        float my_base_hp;
        float enermy_sentry_hp;
        uint16_t shoot_heat;
        uint16_t remaining_bullet;
        uint8_t who_is_balance;
        // 0-not_occupied 1-self_occupied 2-enermy_occupied 3-each_other_occupied
        uint8_t center_gain_point_status; 
        uint8_t at_supply_area;         // 0-not 1-yes
        uint8_t at_center_gain_point;   // 0-not 1-yes
        uint8_t supply_area_status;     // 1-occupied 0-free

		#ifdef ON_MINIPC
		rclcpp::Publisher<robo_utils::msg::ReferenceDataMini>::SharedPtr ros_pub;
		#endif
			
		ReferenceDataMini() : MessageBase(0x40, 14)
		{
			
		}

		virtual void init(void)
		{
			#ifdef ON_MINIPC
			ros_pub = bridge.node->create_publisher<robo_utils::msg::ReferenceDataMini>("/offboardlink/reference_data_mini", 20);  
			#endif
		}
			
		virtual void decode(uint8_t *buf) override
        {

            game_state = buf[OFFBOARDLINK_FRAME_HEAD_LEN + 0];
            remaining_time = (buf[OFFBOARDLINK_FRAME_HEAD_LEN + 2] << 8) | buf[OFFBOARDLINK_FRAME_HEAD_LEN + 1];

            self_color = buf[OFFBOARDLINK_FRAME_HEAD_LEN + 3];
            self_health = (buf[OFFBOARDLINK_FRAME_HEAD_LEN + 5] << 8) | buf[OFFBOARDLINK_FRAME_HEAD_LEN + 4];

            supply_area_status = buf[OFFBOARDLINK_FRAME_HEAD_LEN + 6];
            center_gain_point_status = buf[OFFBOARDLINK_FRAME_HEAD_LEN + 7];
            at_supply_area= buf[OFFBOARDLINK_FRAME_HEAD_LEN + 8];
            at_center_gain_point = buf[OFFBOARDLINK_FRAME_HEAD_LEN + 9];
            shoot_heat = (buf[OFFBOARDLINK_FRAME_HEAD_LEN + 11] << 8) | buf[OFFBOARDLINK_FRAME_HEAD_LEN + 10];

            remaining_bullet = (buf[OFFBOARDLINK_FRAME_HEAD_LEN + 13] << 8) | buf[OFFBOARDLINK_FRAME_HEAD_LEN + 12];
        
            #ifdef ON_MINIPC
			robo_utils::msg::ReferenceDataMini data;
            data.header.stamp = bridge.node->now();
            data.header.frame_id = "cam_earth";
			data.game_state = game_state;
			data.remaining_time = remaining_time;
			data.self_color = self_color;
			data.self_health = self_health;
            data.supply_area_status = supply_area_status;
            data.center_gain_point_status = center_gain_point_status;
            data.at_supply_area = at_supply_area;
            data.at_center_gain_point = at_center_gain_point;
			data.shoot_heat = shoot_heat;
			data.remaining_bullet = remaining_bullet;

            ros_pub->publish(data);
			#endif
        }
			
		virtual void packData(uint8_t *buf) override
		{
			// buf[4] = game_state;
			// buf[5] = remaining_time;
			// buf[6] = remaining_time >> 8;
			// buf[7] = self_color;
			// buf[8] = self_health;
			// buf[9] = self_health >> 8;
			// uint16_t bullet_speed_temp = bullet_speed_meas / 50.0 * 32760.0;
			// buf[10] = bullet_speed_temp;
			// buf[11] = bullet_speed_temp >> 8;
			// buf[12] = shoot_heat;
			// buf[13] = shoot_heat >> 8;
			// buf[14] = hurt_by_gimbal;
			// buf[15] = remaining_bullet;
			// buf[16] = remaining_bullet >> 8;
			// buf[17] = who_is_balance;
			;
		}
	};
}
