#pragma once

#include "robo_bridge/OffboardLink.h"

namespace olk
{
	struct FollowYawCommand : public MessageBase
	{
		public:
		uint8_t if_follow_yaw;

		FollowYawCommand() : MessageBase(0x05, 1)
		{

		}

		virtual void decode(uint8_t *buf) override
		{
			if_follow_yaw = buf[OFFBOARDLINK_FRAME_HEAD_LEN];
		}

		virtual void packData(uint8_t *buf) override
		{
			buf[OFFBOARDLINK_FRAME_HEAD_LEN] = if_follow_yaw;
		}
	};
}
