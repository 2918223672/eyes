#pragma once

#include <Eigen/Dense>

#include "io/my_protocol.hpp"
#include "io/serial/include/serial/serial.h"
#include "tools/utils/crc.hpp"
#include "tools/utils/logger.hpp"

namespace io
{
class CommManager
{
public:
  CommManager(const std::string & port, int baudrate)
  {
    
    try {
      device_.setPort(port);
      device_.setBaudrate(baudrate);

      auto timeout = serial::Timeout::simpleTimeout(10);
      device_.setTimeout(timeout);
      device_.setFlowcontrol(serial::flowcontrol_none);
      device_.open();

      if (device_.isOpen()) {
        tools::logger()->info("Serial [{}] opened, {} baud", port, baudrate);
      }
    } catch (const std::exception & e) {
      tools::logger()->error("Serial init failed: {}", e.what());
    }
  }

  void sendPacket(float y, float p, uint8_t state, bool shoot_cmd)
  {
    if (!device_.isOpen()) return;

    VisionProtocol packet;
    packet.yaw = y;
    packet.pitch = p;
    packet.control = 0;
    if (state > 0) packet.control |= (1 << 0);
    packet.shoot = 0;
    if (shoot_cmd) packet.shoot |= (1 << 0);

    uint8_t * ptr = reinterpret_cast<uint8_t *>(&packet);
    uint8_t sum = 0;
    for (int i = 0; i < 17; i++) sum += ptr[i];
    packet.checksum = sum;

    try {
      device_.write(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
    } catch (const std::exception & e) {
      tools::logger()->warn("Serial write failed: {}", e.what());
    }
  }

  bool receivePacket()
  {
    if (!device_.isOpen()) return false;

    while (device_.available() >= sizeof(CBoardPacket)) {
      uint8_t head;
      device_.read(&head, 1);
      if (head != 0x5A) continue;

      uint8_t buffer[sizeof(CBoardPacket)];
      buffer[0] = head;
      device_.read(buffer + 1, sizeof(CBoardPacket) - 1);

      if (buffer[sizeof(CBoardPacket) - 1] != 0xFE) continue;

      uint8_t sum = 0;
      for (int i = 0; i < static_cast<int>(sizeof(CBoardPacket) - 2); i++) sum += buffer[i];

      if (sum != buffer[sizeof(CBoardPacket) - 2]) {
        tools::logger()->warn("Serial checksum failed");
        continue;
      }

      CBoardPacket * pkt = reinterpret_cast<CBoardPacket *>(buffer);
      latest_quat_.x() = pkt->qx;
      latest_quat_.y() = pkt->qy;
      latest_quat_.z() = pkt->qz;
      latest_quat_.w() = pkt->qw;

      return true;
    }

    return false;
  }

  void sendDebugChar(char c)
  {
    if (device_.isOpen()) device_.write(reinterpret_cast<uint8_t *>(&c), 1);
  }

  Eigen::Quaterniond getQuaternion() const { return latest_quat_; }

private:
  serial::Serial device_;
  Eigen::Quaterniond latest_quat_ = Eigen::Quaterniond::Identity();
};

}  // namespace io
