
/** @file
 *	@brief Offboardlink comm protocol generated from ReferenceDataSuper.json
 *  @author Xianhao Ji
 */

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
    struct ReferenceDataMini : public MessageBase
    {
        public:
        uint8_t game_state;//比赛状态：0-比赛未开始 1-比赛开始且为RMUC 2-比赛开始且为3V3
        uint16_t remaining_time;//比赛剩余时间
        uint8_t self_color;//己方颜色：0-红方 1-蓝方
        uint16_t self_health;//当前血量
		
        uint8_t at_base_area;//基地增益点RFID
        uint8_t at_castle_area;//堡垒增益点RFID
        uint8_t at_supply_area_unoverlap;//与资源区不重叠的补给区RFID
        uint8_t at_supply_area_overlap;//与资源区重叠的补给区RFID
        uint8_t supply_area_status;//补给区占领状态
        uint8_t reserved;
        
        uint8_t auto_aiming_status;//自瞄状态：瞄上为1，否则为0        
        uint8_t energy_percent;//剩余能量百分比指示，参考串口手册      
        uint16_t shoot_heat;//枪口热量
        uint16_t remaining_bullet;//剩余弹丸数
		
        //RMUL
        uint16_t at_center_gain_point;//中心增益点RFID
        uint16_t at_supply_area;//补给区RFID
        uint16_t center_gain_point_status;//中心增益点占领状态：0-未占领 1-己方占领 2-对方占领 3-双方占领
        uint16_t supply_area_status_RMUL;//补给区（RMUL）占领状态

        helper_float_u32 h;
            
        ReferenceDataMini() : MessageBase(0x40, 26)
		{
				
		}
		
        virtual void decode(uint8_t *buf) override
        {
			game_state = buf[OFFBOARDLINK_FRAME_HEAD_LEN + 0];        
			remaining_time = (buf[OFFBOARDLINK_FRAME_HEAD_LEN + 2] << 8) | buf[OFFBOARDLINK_FRAME_HEAD_LEN + 1];
			self_color = buf[OFFBOARDLINK_FRAME_HEAD_LEN + 3];        
			self_health = (buf[OFFBOARDLINK_FRAME_HEAD_LEN + 5] << 8) | buf[OFFBOARDLINK_FRAME_HEAD_LEN + 4];

			at_base_area = buf[OFFBOARDLINK_FRAME_HEAD_LEN + 6];
			at_castle_area = buf[OFFBOARDLINK_FRAME_HEAD_LEN + 7];
			at_supply_area_unoverlap = buf[OFFBOARDLINK_FRAME_HEAD_LEN + 8];
			at_supply_area_overlap = buf[OFFBOARDLINK_FRAME_HEAD_LEN + 9];
			supply_area_status = buf[OFFBOARDLINK_FRAME_HEAD_LEN + 10];
			reserved = buf[OFFBOARDLINK_FRAME_HEAD_LEN + 11];
          
			auto_aiming_status = buf[OFFBOARDLINK_FRAME_HEAD_LEN + 12];
			energy_percent = buf[OFFBOARDLINK_FRAME_HEAD_LEN + 13];
			shoot_heat = (buf[OFFBOARDLINK_FRAME_HEAD_LEN + 15] << 8) | buf[OFFBOARDLINK_FRAME_HEAD_LEN + 14];
			remaining_bullet = (buf[OFFBOARDLINK_FRAME_HEAD_LEN + 17] << 8) | buf[OFFBOARDLINK_FRAME_HEAD_LEN + 16];

			at_center_gain_point = buf[OFFBOARDLINK_FRAME_HEAD_LEN + 18];
			at_supply_area = buf[OFFBOARDLINK_FRAME_HEAD_LEN + 19];
			center_gain_point_status = buf[OFFBOARDLINK_FRAME_HEAD_LEN + 20];
			supply_area_status_RMUL = buf[OFFBOARDLINK_FRAME_HEAD_LEN + 21];
			
        
        }

        virtual void packData(uint8_t *buf) override
        {

			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 0] = game_state;

			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 1] = remaining_time;
            buf[OFFBOARDLINK_FRAME_HEAD_LEN + 2] = (uint16_t)(remaining_time) >> 8;

			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 3] = self_color;

			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 4] = self_health;
            buf[OFFBOARDLINK_FRAME_HEAD_LEN + 5] = (uint16_t)(self_health) >> 8;

			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 6] = at_base_area;
			
			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 7] = at_castle_area;
			
			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 8] = at_supply_area_unoverlap;
			
			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 9] = at_supply_area_overlap;
			
			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 10] = supply_area_status;
			
			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 11] = reserved;
			
			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 12] = auto_aiming_status;
			
			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 13] = energy_percent;
			
			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 14] = shoot_heat;
            buf[OFFBOARDLINK_FRAME_HEAD_LEN + 15] = (uint16_t)(shoot_heat) >> 8;

			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 16] = remaining_bullet;
            buf[OFFBOARDLINK_FRAME_HEAD_LEN + 17] = (uint16_t)(remaining_bullet) >> 8;
			
			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 18] = at_center_gain_point;
			
			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 19] = at_supply_area;
			
			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 20] = center_gain_point_status;
			
			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 21] = supply_area_status_RMUL;
						
        }
    };
}
        
