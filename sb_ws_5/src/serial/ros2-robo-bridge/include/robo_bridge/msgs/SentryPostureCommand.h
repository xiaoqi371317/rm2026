#pragma once
#include "robo_bridge/OffboardLink.h"

namespace olk
{
struct SentryPostureCommand : public MessageBase
{
  uint8_t posture; // 1/2/3

  SentryPostureCommand() : MessageBase(0x03, 1), posture(3) {}

  void decode(uint8_t* buf) override { posture = buf[OFFBOARDLINK_FRAME_HEAD_LEN]; }

  void packData(uint8_t* buf) override { buf[OFFBOARDLINK_FRAME_HEAD_LEN] = posture; }
};
}