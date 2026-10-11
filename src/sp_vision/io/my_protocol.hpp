#pragma once
#include <array>
#include <cstdint>

// seasky 协议：视觉上位机 ↔ C板 串口通信（USART1，波特率 921600，8N1）
// 帧格式（两方向通用）：
//   [0] SOF=0xA5  [1-2] data_len(小端)=float数*4+2  [3] CRC8(对[0..2])
//   [4-5] cmd_id(小端)  [6-7] flags(小端)  [8..] float段(小端)  [末2] CRC16(小端)
// 已对照 C板 源码 modules/master_machine/seasky_protocol.c 逐字节确认。
// 这里只放格式定义，具体实现见 io/my_comm_manager.hpp。
namespace io
{

// ===== 常量 =====
constexpr uint8_t SOF = 0xA5;
constexpr double kDegToRad = 0.017453292519943295;
constexpr double kRadToDeg = 57.29577951308232;

// ===== 枚举（取值与 C板 master_process.h 完全一致）=====
enum class FireMode : uint8_t { NO_FIRE = 0, AUTO_FIRE = 1, AUTO_AIM = 2 };
enum class TargetState : uint8_t { NO_TARGET = 0, TARGET_CONVERGING = 1, READY_TO_FIRE = 2 };
enum class TargetType : uint8_t {
  NONE = 0, HERO = 1, ENGINEER = 2, INFANTRY3 = 3, INFANTRY4 = 4,
  INFANTRY5 = 5, OUTPOST = 6, SENTRY = 7, BASE = 8
};
enum class EnemyColor : uint8_t { NONE = 0, BLUE = 1, RED = 2 };
enum class WorkMode : uint8_t { AIM = 0, SMALL_BUFF = 1, BIG_BUFF = 2 };
enum class BulletSpeed : uint8_t { NONE = 0, BIG_10 = 10, SMALL_15 = 15, BIG_16 = 16, SMALL_18 = 18, SMALL_30 = 30 };

// ===== 帧结构 =====
// 视觉→C板：2 个 float（先 pitch 后 yaw），均为误差角，单位为 rad。
struct SeaskyTx
{
  uint16_t flags;
  float pitch_rad;
  float yaw_rad;
};

// C板→视觉：3 个 float（先 yaw 后 pitch 后 roll）。
// 电控回传原始单位是 degree，进入视觉后必须转换成 rad。
struct SeaskyRx
{
  uint16_t cmd_id;
  uint16_t flags;
  float yaw_deg;
  float pitch_deg;
  float roll_deg;
};

inline std::array<double, 3> ypr_rad(const SeaskyRx & rx)
{
  return {
    rx.yaw_deg * kDegToRad,
    rx.pitch_deg * kDegToRad,
    rx.roll_deg * kDegToRad
  };
}

}  // namespace io
