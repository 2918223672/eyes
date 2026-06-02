#pragma once
#include <cstdint>

namespace io
{
#pragma pack(push, 1)
struct VisionProtocol {
  uint8_t header = 0x5A;
  uint16_t control;
  uint16_t shoot;
  float yaw;
  float pitch;
  uint8_t reserved[4] = {0};
  uint8_t checksum;
  uint8_t tail = 0xFE;
};

struct CBoardPacket {
  uint8_t header = 0x5A;
  float qx;
  float qy;
  float qz;
  float qw;
  uint8_t checksum;
  uint8_t tail = 0xFE;
};
#pragma pack(pop)

}  // namespace io
