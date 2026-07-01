#pragma once

#include "robo_bridge/OffboardLink.h"

#ifdef DEBUG_MODE
#include "UARTDriver.h"
#endif

#ifdef ON_MINIPC
#include <robo_utils/MinipcHeartbeat.h>
#include "robo_core.h"
extern Robo_Core core;
#endif

namespace olk
{
	struct MinipcHeartbeat : MessageBase
	{
		public:
		uint8_t current_target;
        uint8_t detect_num;
        uint8_t aim_mode;

		#ifdef ON_MINIPC
		ros::Publisher ros_pub;
		#endif
			
		MinipcHeartbeat() : MessageBase(0x50, 3)
		{
			#ifdef ON_MINIPC
			ros_pub = core.node_handle->advertise<robo_utils::MinipcHeartbeat>("/offboardlink/minipc_heartbeat", 20);  
			#endif
		}
			
		virtual void decode(uint8_t *buf) override
		{
			current_target = buf[4];
			detect_num = buf[5];
			aim_mode = buf[6];

			#ifdef ON_MINIPC
			robo_utils::MinipcHeartbeat data;
            data.header.stamp = ros::Time::now();
            data.header.frame_id = "cam_earth";
			data.current_target = current_target;
			data.detect_num = detect_num;
			data.aim_mode = aim_mode;
            ros_pub.publish(data);
			#endif
		}
			
		virtual void packData(uint8_t *buf) override
		{
			buf[4] = current_target;
			buf[5] = detect_num;
			buf[6] = aim_mode;
		}
	};
}
