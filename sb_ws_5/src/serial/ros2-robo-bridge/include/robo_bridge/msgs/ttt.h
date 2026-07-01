/** @file
 *	@brief Offboardlink comm protocol generated from ReferenceDataSuper.json
 *  @author Xianhao Ji
 */

#pragma once
#include "OffboardLink.h"

namespace olk
{
    struct ReferenceDataSuper : public MessageBase
    {
        public:
        uint8_t game_state;
        uint16_t remaining_time;
        uint8_t self_color;
        uint16_t self_health;
		
		uint8_t at_base_area;//哨兵扫上基地增益点的RFID了吗，扫上为1，否则为0
		uint8_t at_castle_area;//哨兵扫上堡垒增益点的RFID了吗，扫上为1，否则为0
		uint8_t at_supply_area_unoverlap;//哨兵扫上与兑换区不重叠补给区域的RFID了吗，扫上为1，否则为0
		uint8_t at_supply_area_overlap;//哨兵扫上与兑换区重叠补给区域的RFID了吗，扫上为1，否则为0
		uint8_t supply_area_overlap_status;//与兑换区重叠补给区域状态，占领为1，没有为0
		uint8_t supply_area_unoverlap_status;//与兑换区不重叠补给区域状态，占领为1，没有为0
		uint8_t auto_aiming_status;//自瞄数据，在瞄上为1，否则为0
		uint8_t energy_percent;//剩余能量百分比指示，参考串口手册
		
		uint16_t shoot_heat;//枪口热量
        uint16_t remaining_bullet;//剩余弹丸数，超过1023为1023，其余正常
		
        uint16_t enermy_outpost_hp;//敌方前哨站血量
        uint16_t my_outpost_hp;//我方前哨站血量
        uint16_t enermy_base_hp;//敌方基地血量
        uint16_t my_base_hp;//我方基地血量

        helper_float_u32 h;
            
        ReferenceDataSuper() : MessageBase(0x40, 26)
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
			supply_area_overlap_status = buf[OFFBOARDLINK_FRAME_HEAD_LEN + 10];
			supply_area_unoverlap_status = buf[OFFBOARDLINK_FRAME_HEAD_LEN + 11];
			auto_aiming_status = buf[OFFBOARDLINK_FRAME_HEAD_LEN + 12];
			energy_percent = buf[OFFBOARDLINK_FRAME_HEAD_LEN + 13];
			shoot_heat = (buf[OFFBOARDLINK_FRAME_HEAD_LEN + 15] << 8) | buf[OFFBOARDLINK_FRAME_HEAD_LEN + 14];
			remaining_bullet = (buf[OFFBOARDLINK_FRAME_HEAD_LEN + 17] << 8) | buf[OFFBOARDLINK_FRAME_HEAD_LEN + 16];

			enermy_outpost_hp = (buf[OFFBOARDLINK_FRAME_HEAD_LEN + 19] << 8) | buf[OFFBOARDLINK_FRAME_HEAD_LEN + 18];
			my_outpost_hp = (buf[OFFBOARDLINK_FRAME_HEAD_LEN + 21] << 8) | buf[OFFBOARDLINK_FRAME_HEAD_LEN + 20];
			enermy_base_hp = (buf[OFFBOARDLINK_FRAME_HEAD_LEN + 23] << 8) | buf[OFFBOARDLINK_FRAME_HEAD_LEN + 22];
			my_base_hp = (buf[OFFBOARDLINK_FRAME_HEAD_LEN + 25] << 8) | buf[OFFBOARDLINK_FRAME_HEAD_LEN + 24];
			
        
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
			
			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 10] = supply_area_overlap_status;
			
			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 11] = supply_area_unoverlap_status;
			
			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 12] = auto_aiming_status;
			
			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 13] = energy_percent;
			
			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 14] = shoot_heat;
            buf[OFFBOARDLINK_FRAME_HEAD_LEN + 15] = (uint16_t)(shoot_heat) >> 8;

			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 16] = remaining_bullet;
            buf[OFFBOARDLINK_FRAME_HEAD_LEN + 17] = (uint16_t)(remaining_bullet) >> 8;
			
			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 18] = enermy_outpost_hp;
			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 19] = (uint16_t)(enermy_outpost_hp) >> 8;
			
			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 20] = my_outpost_hp;
			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 21] = (uint16_t)(my_outpost_hp) >> 8;			
			
			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 22] = enermy_base_hp;
			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 23] = (uint16_t)(enermy_base_hp) >> 8;	

			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 24] = my_base_hp;
			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 25] = (uint16_t)(my_base_hp) >> 8;	
			
        }
    };
}
        