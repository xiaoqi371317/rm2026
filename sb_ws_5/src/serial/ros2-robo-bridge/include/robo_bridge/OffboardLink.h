#pragma once

// #define ON_CBOARD
#define ON_MINIPC

#include <stdint.h>

#ifdef ON_CBOARD
#include "Scheduler_Common.h"
#include "UARTDriver.h"
#define TIME_T timeus_t
#define OFFBOARDLINK_TARGET_ADDRESS 0xEE
#define OFFBOARDLINK_SOURCE_ADDRESS 0xFF
#define OFFBOARDLINK_MESSAGE_TIMEOUT 500000
#endif
#ifdef ON_MINIPC
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include "robo_bridge/robo_bridge.h"
extern RoboBridge bridge;
#define TIME_T double
#define OFFBOARDLINK_TARGET_ADDRESS 0xFF
#define OFFBOARDLINK_SOURCE_ADDRESS 0xEE
#define OFFBOARDLINK_MESSAGE_TIMEOUT 0.5
#endif

#define OFFBOARDLINK_FRAME_HEAD 0xAA
#define OFFBOARDLINK_FRAME_HEAD_LEN 4
#define OFFBOARDLINK_FRAME_VERIFY_LEN 2
#define OFFBOARDLINK_FRAME_MAX_LEN 60

#define OFFBOARDLINK_MSG_MAX_NUM 0x100

namespace olk
{
	union helper_float_u32
	{
		float f;
		uint8_t u[4];
	};
	
	class Subscriber;
	
	struct MessageBase
	{
		friend class Subscriber;
		friend class Publisher;
		protected:
		TIME_T timestamp;
		
		uint8_t func_id;
		uint8_t data_len;
		
		uint8_t send_buf[OFFBOARDLINK_FRAME_MAX_LEN];
		
		virtual void decode(uint8_t *buf) = 0;
		
		void calcVerify(uint8_t *buf, uint8_t &sum_verify, uint8_t &sum_sum_verify)
		{
			sum_verify = 0;
			sum_sum_verify = 0;
			for(uint8_t i = 0; i < data_len + OFFBOARDLINK_FRAME_HEAD_LEN; i ++)
			{
				sum_verify += buf[i];
				sum_sum_verify += sum_verify;
			}
		}
		
		bool verify(uint8_t *buf)
		{
			uint8_t sum_verify, sum_sum_verify;
			calcVerify(buf, sum_verify, sum_sum_verify);
			if(sum_verify == buf[data_len + OFFBOARDLINK_FRAME_HEAD_LEN] && sum_sum_verify == buf[data_len + OFFBOARDLINK_FRAME_HEAD_LEN + 1])
			{
				return true;
			}
			else return false;
		}
		
		public:
		MessageBase(uint8_t func_id, uint8_t data_len)
		{
			this->func_id = func_id;
			this->data_len = data_len;
		}		
		
		TIME_T getTimestamp(void)
		{
			return timestamp;
		}
		
		bool isTimeout(void)
		{
			#ifdef ON_CBOARD
			return micros() - timestamp >= OFFBOARDLINK_MESSAGE_TIMEOUT;
			#endif
			#ifdef ON_MINIPC
			return bridge.node->now().seconds() - timestamp >= OFFBOARDLINK_MESSAGE_TIMEOUT;
			#endif
		}
		
		virtual void packData(uint8_t * buf) = 0;
		
		uint8_t * pack(void)
		{
			uint8_t sum_verify, sum_sum_verify;
			
			send_buf[0] = OFFBOARDLINK_FRAME_HEAD;
			send_buf[1] = OFFBOARDLINK_SOURCE_ADDRESS;
			send_buf[2] = func_id;
			send_buf[3] = data_len;
			packData(send_buf);
			
			calcVerify(send_buf, sum_verify, sum_sum_verify);
			send_buf[data_len + OFFBOARDLINK_FRAME_HEAD_LEN] = sum_verify;
			send_buf[data_len + OFFBOARDLINK_FRAME_HEAD_LEN + 1] = sum_sum_verify;
			
			return send_buf;
		}
		
		bool unpack(uint8_t * buf)
		{
			if(verify(buf))
			{
				decode(buf);
				
				#ifdef ON_CBOARD
				timestamp = micros();
				#endif
				#ifdef ON_MINIPC
				timestamp = bridge.node->now().seconds();
				#endif
				
				return true;
			}
			else return false;
		}

		virtual void init(void)
		{

		}
		
	};

	class Publisher
	{
		public:
			Publisher(void)
			{
				
			}
			
			bool sendData(uint8_t *data, uint8_t size)
			{
				#ifdef ON_CBOARD
				if(HAL_UART_Transmit_DMA(&huart1, data, size) == HAL_BUSY) return false;
				else return true;
				#endif
				#ifdef ON_MINIPC
				bridge.write(data, size);
				return true;
				#endif
			}
			
			bool publish(MessageBase *msg)
			{
				data_buf = msg->pack();
				return sendData(data_buf, msg->data_len + OFFBOARDLINK_FRAME_HEAD_LEN + OFFBOARDLINK_FRAME_VERIFY_LEN);
			}
		protected:
			uint8_t *data_buf;
	};
	
	class Subscriber
	{
		public:
			Subscriber(void)
			{
				for(uint16_t i = 0; i < OFFBOARDLINK_MSG_MAX_NUM; i ++)
				{
					sub_list[i] = nullptr;
				}
				for(uint16_t i = 0; i <= OFFBOARDLINK_FRAME_MAX_LEN; i ++)
				{
					frame_len_to_func_id_map[i] = nullptr;
				}
				index = 0;
				min_frame_len = OFFBOARDLINK_FRAME_MAX_LEN;
				max_frame_len = 0;
			}
		
			void subscribe(uint8_t func_id, MessageBase *msg)
			{
				uint8_t has_same = 0;
				
				for(uint16_t i = 0; i < OFFBOARDLINK_MSG_MAX_NUM; i ++)
				{
					if(sub_list[i] != nullptr && sub_list[i]->data_len == msg->data_len)
					{
						has_same = 1;
						break;
					}
				}
				
				msg->init();
				sub_list[func_id] = msg;
				
				uint16_t frame_len = msg->data_len + OFFBOARDLINK_FRAME_HEAD_LEN + OFFBOARDLINK_FRAME_VERIFY_LEN;
				if(has_same == 0 && frame_len <= OFFBOARDLINK_FRAME_MAX_LEN)
				{
					frame_len_to_func_id_map[frame_len] = msg;
				}
				
				getMinMaxFrameLen(msg);
			}
			
			void getMinMaxFrameLen(MessageBase *msg)
			{
				if(msg->data_len + OFFBOARDLINK_FRAME_HEAD_LEN + OFFBOARDLINK_FRAME_VERIFY_LEN < min_frame_len)
				{
					min_frame_len = msg->data_len + OFFBOARDLINK_FRAME_HEAD_LEN + OFFBOARDLINK_FRAME_VERIFY_LEN;
				}
				if(msg->data_len + OFFBOARDLINK_FRAME_HEAD_LEN + OFFBOARDLINK_FRAME_VERIFY_LEN > max_frame_len)
				{
					max_frame_len = msg->data_len + OFFBOARDLINK_FRAME_HEAD_LEN + OFFBOARDLINK_FRAME_VERIFY_LEN;
				}
			}
			
			void processByte(uint8_t data)
			{
				if(index >= OFFBOARDLINK_FRAME_MAX_LEN)
				{
					index = 0;
				}
				rec_buffer[index] = data;
				index = index + 1;
				if(index >= 2 && rec_buffer[index - 2] == OFFBOARDLINK_FRAME_HEAD && data == OFFBOARDLINK_TARGET_ADDRESS)
				{
						index = 2;
						rec_buffer[0] = OFFBOARDLINK_FRAME_HEAD;
						rec_buffer[1] = OFFBOARDLINK_TARGET_ADDRESS;
				}
				
				if(index >= OFFBOARDLINK_FRAME_HEAD_LEN)
				{
					MessageBase* msg = sub_list[rec_buffer[2]];
					if(msg != nullptr)
					{
						uint16_t expected_frame_len = msg->data_len + OFFBOARDLINK_FRAME_HEAD_LEN + OFFBOARDLINK_FRAME_VERIFY_LEN;
						if(rec_buffer[3] != msg->data_len)
						{
							index = 0;
						}
						else if(index == expected_frame_len)
						{
							if(frame_len_to_func_id_map[index] != nullptr)
							{
								msg->unpack(rec_buffer);
							}
							index = 0;
						}
						else if(index > expected_frame_len)
						{
							index = 0;
						}
					}
				}
				
				if(index > max_frame_len)
				{
					index = 0;
				}
			}
		protected:
			MessageBase *sub_list[OFFBOARDLINK_MSG_MAX_NUM];
			MessageBase *frame_len_to_func_id_map[OFFBOARDLINK_FRAME_MAX_LEN + 1];
			
			uint8_t rec_buffer[OFFBOARDLINK_FRAME_MAX_LEN];
			uint8_t index;
			uint8_t min_frame_len, max_frame_len;
	};
};
