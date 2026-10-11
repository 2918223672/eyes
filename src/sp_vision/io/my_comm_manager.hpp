#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "io/my_protocol.hpp"
#include "io/socketuart.hpp"
#include "tools/utils/logger.hpp"

// seasky 协议具体实现：CRC、小端读写、打包、解包，以及上层的 CommManager。
// 格式定义见 io/my_protocol.hpp，字节流传输（守护线程）见 io/socketuart.hpp。
namespace io
{

// ===== CRC =====
inline uint8_t seasky_crc8(const uint8_t * data, size_t len)
{
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++)
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x31) : static_cast<uint8_t>(crc << 1);
  }
  return crc;
}

// CRC-16/MODBUS：poly 0x8005（反射 0xA001），init 0xFFFF（与 C板 crc16.c 一致）
inline uint16_t seasky_crc16(const uint8_t * data, size_t len)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++)
      crc = (crc & 0x0001) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001) : static_cast<uint16_t>(crc >> 1);
  }
  return crc;
}

// ===== 小端读写（与主机字节序无关，逐字节一致）=====
inline void write_le16(uint8_t * p, uint16_t v)
{
  p[0] = v & 0xFF;
  p[1] = (v >> 8) & 0xFF;
}

inline void write_le_float(uint8_t * p, float v)
{
  uint32_t u;
  std::memcpy(&u, &v, 4);
  p[0] = u & 0xFF;
  p[1] = (u >> 8) & 0xFF;
  p[2] = (u >> 16) & 0xFF;
  p[3] = (u >> 24) & 0xFF;
}

inline uint16_t read_le16(const uint8_t * p)
{
  return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

inline float read_le_float(const uint8_t * p)
{
  uint32_t u = p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24);
  float v;
  std::memcpy(&v, &u, 4);
  return v;
}

// ===== 打包 =====
// 视觉→C板 flags 位布局（建议值；C板 master_process.c:68 仍未解码，回环时原样回传）
inline uint16_t pack_tx_flags(FireMode fm, TargetState ts, TargetType tt)
{
  return static_cast<uint16_t>(
    (static_cast<uint8_t>(fm) & 0x3) | ((static_cast<uint8_t>(ts) & 0x3) << 2) |
    ((static_cast<uint8_t>(tt) & 0xF) << 4));
}

// 打包视觉→C板帧，返回 18 字节。out 需至少 18 字节
inline size_t build_tx_frame(
  uint8_t * out, uint16_t flags, float pitch_rad, float yaw_rad)
{
  const uint16_t data_len = 10;  // 2 float * 4 + 2(flags)
  out[0] = SOF;
  write_le16(out + 1, data_len);
  out[3] = seasky_crc8(out, 3);
  write_le16(out + 4, 0x01);  // cmd_id：视觉→C板。C板接收不校验此值（回传时用 0x02），填 0x01 作标识
  write_le16(out + 6, flags);
  write_le_float(out + 8, pitch_rad);   // 先 pitch
  write_le_float(out + 12, yaw_rad);    // 后 yaw
  write_le16(out + 16, seasky_crc16(out, 16));
  return static_cast<size_t>(data_len + 8);
}

// ===== 解包：字节流状态机 =====
// 逐字节喂入，找帧头、校验 CRC8/CRC16，解析出完整帧后回调。
// 回环：C板回传 cmd_id=0x02、flags 原样、yaw/pitch 回显、roll=0。
class SeaskyDecoder
{
public:
  using RxCallback = std::function<void(const SeaskyRx &)>;

  void feed(const uint8_t * data, size_t size, const RxCallback & cb)
  {
    buf_.insert(buf_.end(), data, data + size);

    while (buf_.size() >= 8) {
      if (buf_[0] != SOF) {  // 找帧头
        buf_.erase(buf_.begin());
        continue;
      }

      uint16_t data_len = read_le16(&buf_[1]);
      size_t total = static_cast<size_t>(data_len) + 8;
      if (buf_.size() < total) return;  // 帧不完整，等更多字节

      if (seasky_crc8(buf_.data(), 3) != buf_[3]) {  // 帧头 CRC8 错
        buf_.erase(buf_.begin());
        continue;
      }

      uint16_t crc = read_le16(&buf_[total - 2]);
      if (seasky_crc16(buf_.data(), total - 2) != crc) {  // 整包 CRC16 错
        buf_.erase(buf_.begin());
        continue;
      }

      int float_count = (data_len - 2) / 4;
      if (float_count == 3) {
        SeaskyRx rx;
        rx.cmd_id = read_le16(&buf_[4]);
        rx.flags = read_le16(&buf_[6]);
        rx.yaw_deg = read_le_float(&buf_[8]);
        rx.pitch_deg = read_le_float(&buf_[12]);
        rx.roll_deg = read_le_float(&buf_[16]);
        cb(rx);
      }

      buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(total));
    }
  }

private:
  std::vector<uint8_t> buf_;
};

// ===== 通信管理：字节流收发 + seasky 编解码 =====
class CommManager
{
public:
  CommManager(const std::string & port, uint32_t baudrate)
  : uart_(port, baudrate, [this](const uint8_t * data, size_t size) {
      decoder_.feed(data, size, [this](const SeaskyRx & rx) {
        std::lock_guard<std::mutex> lock(rx_mutex_);
        latest_rx_ = rx;
        has_new_rx_ = true;
      });
    })
  {
    tools::logger()->info("[CommManager] ready.");
  }

  // 发送一帧：flags + 误差角（先 pitch 后 yaw），角度单位为 rad。
  void send(uint16_t flags, float pitch_rad, float yaw_rad)
  {
    uint8_t buf[18];
    size_t n = build_tx_frame(buf, flags, pitch_rad, yaw_rad);
    uart_.write(buf, n);
  }

  // 非阻塞：有新的完整帧时返回 true，并把内容填进 rx
  bool receive(SeaskyRx & rx)
  {
    std::lock_guard<std::mutex> lock(rx_mutex_);
    if (!has_new_rx_) return false;
    rx = latest_rx_;
    has_new_rx_ = false;
    return true;
  }

  bool isOpen() const { return uart_.isOpen(); }

private:
  SeaskyDecoder decoder_;  // 先于 uart_ 构造，避免读线程过早回调到未构造成员
  std::mutex rx_mutex_;
  SeaskyRx latest_rx_{};
  bool has_new_rx_ = false;
  SocketUART uart_;  // 最后构造（其构造函数会立即启动读线程）
};

}  // namespace io
