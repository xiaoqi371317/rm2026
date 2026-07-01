#pragma once

#include "robo_bridge/OffboardLink.h"

#ifdef DEBUG_MODE
#include "UARTDriver.h"
#endif

namespace olk
{
	struct ChassisVelYawRateCommand : public MessageBase
	{
		public:
		float x;
		float y;
		float z;
		float yaw_rate;
		uint8_t frame;
			
		helper_float_u32 h;
			
		ChassisVelYawRateCommand() : MessageBase(0x02, 17)
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
			
			for(uint8_t i = 0; i < 4; i ++)
			{
				h.u[i] = buf[OFFBOARDLINK_FRAME_HEAD_LEN + i + 12];
			}
			yaw_rate = h.f;
			
			frame = buf[OFFBOARDLINK_FRAME_HEAD_LEN + 16];
			
			#ifdef DEBUG_MODE
			USART1_DMA_Debug_Printf("0x%x %f %f %f %f\r", frame, x, y, z, yaw_rate);
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
			h.f = yaw_rate;
			for(uint8_t i = 0; i < 4; i ++)
			{
					buf[OFFBOARDLINK_FRAME_HEAD_LEN + i + 12] = h.u[i];
			}
				
			buf[OFFBOARDLINK_FRAME_HEAD_LEN + 16] = frame;
		}
	};
}
