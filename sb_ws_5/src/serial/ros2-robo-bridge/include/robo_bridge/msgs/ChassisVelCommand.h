#pragma once

#include "robo_bridge/OffboardLink.h"

#ifdef DEBUG_MODE
#include "UARTDriver.h"
#endif

namespace olk
{
	struct ChassisVelCommand : public MessageBase
	{
		public:
		float x;
		float y;
		float z;
		uint8_t frame;
			
		helper_float_u32 h;
			
		ChassisVelCommand() : MessageBase(0x01, 13)
		{
				
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
			
			frame = buf[OFFBOARDLINK_FRAME_HEAD_LEN + 12];
			
			#ifdef DEBUG_MODE
			USART1_DMA_Debug_Printf("0x%x %f %f %f\r", frame, x, y, z);
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
				
			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 12] = frame;
		}
	};
}
