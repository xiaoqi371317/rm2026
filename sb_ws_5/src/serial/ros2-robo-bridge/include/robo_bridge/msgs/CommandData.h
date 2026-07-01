#pragma once

#include "robo_bridge/OffboardLink.h"

#ifdef DEBUG_MODE
#include "UARTDriver.h"
#endif

#ifdef ON_MINIPC
#include <robo_utils/msg/command_data.hpp>
#include "robo_bridge/robo_bridge.h"
extern RoboBridge bridge;
#endif

namespace olk
{
	struct CommandData : public MessageBase
	{
		public:
		float x;
		float y;
		float z;
		uint8_t key;
		uint8_t target_id;
			
		helper_float_u32 h;

		#ifdef ON_MINIPC
		rclcpp::Publisher<robo_utils::msg::CommandData>::SharedPtr cmd_pub;
		#endif
			
		CommandData() : MessageBase(0x60, 14)
		{
				
		}

		virtual void init(void)
		{
			#ifdef ON_MINIPC
			cmd_pub = bridge.node->create_publisher<robo_utils::msg::CommandData>("/offboardlink/cmd_data", 1);  
			#endif
		}
			
		virtual void decode(uint8_t *buf) override
		{
			for(uint8_t i = 0; i < 4; i ++)
			{
				h.u[i] = buf[OFFBOARDLINK_FRAME_HEAD_LEN + i];
			}
			x = h.f;
			for(uint8_t i = 0; i < 4; i ++)
			{
				h.u[i] = buf[OFFBOARDLINK_FRAME_HEAD_LEN + i + 4];
			}
			y = h.f;
			for(uint8_t i = 0; i < 4; i ++)
			{
				h.u[i] = buf[OFFBOARDLINK_FRAME_HEAD_LEN + i + 8];
			}
			z = h.f;
			
			key = buf[OFFBOARDLINK_FRAME_HEAD_LEN + 12];
			target_id = buf[OFFBOARDLINK_FRAME_HEAD_LEN + 13];
			
//			#ifdef DEBUG_MODE
//			USART1_DMA_Debug_Printf("0x%x %f %f %f\r", frame, x, y, z);
//			#endif
			#ifdef ON_MINIPC
			robo_utils::msg::CommandData cmd_data;
            cmd_data.header.stamp = bridge.node->now();
            cmd_data.header.frame_id = "map";
            cmd_data.x = x;
			cmd_data.y = y;
			cmd_data.z = z;
			cmd_data.key = key;
			cmd_data.target_id = target_id;
            cmd_pub->publish(cmd_data);
			#endif
		}
			
		virtual void packData(uint8_t *buf) override
		{
			h.f = x;
			for(uint8_t i = 0; i < 4; i ++)
			{
				buf[OFFBOARDLINK_FRAME_HEAD_LEN + i] = h.u[i];
			}
			h.f = y;
			for(uint8_t i = 0; i < 4; i ++)
			{
				buf[OFFBOARDLINK_FRAME_HEAD_LEN + i + 4] = h.u[i];
			}
			h.f = z;
			for(uint8_t i = 0; i < 4; i ++)
			{
					buf[OFFBOARDLINK_FRAME_HEAD_LEN + i + 8] = h.u[i];
			}
				
			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 12] = key;
			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 13] = target_id;
		}
	};
}
