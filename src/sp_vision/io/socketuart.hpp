#ifndef IO__SOCKETUART_HPP
#define IO__SOCKETUART_HPP

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include "serial/serial.h"
#include "tools/utils/logger.hpp"

using namespace std::chrono_literals;

namespace io
{
// UART 串口传输层，结构与 io/socketcan.hpp 的 SocketCAN 对齐：
//   接收线程读字节流 -> 出错置 ok_ = false 退出
//   守护线程每 100ms 轮询 ok_ -> 收尸 + 关串口 + 重连（断线自愈）
//
// 相比 SocketCAN 做了三处并发安全修正：
//   1. quit_ / ok_ 用 std::atomic<bool>，消除数据竞争
//   2. mutex_ 保护 serial_ 的 open/close/write，避免主线程 write 与守护线程重连打架
//   3. ok_ 在 open() 里先置 true 再启动读线程，消除“守护线程 join 掉健康读线程”的启动竞态
class SocketUART
{
public:
  SocketUART(
    const std::string & port, uint32_t baudrate,
    std::function<void(const uint8_t * data, size_t size)> rx_handler)
  : port_(port),
    baudrate_(baudrate),
    rx_handler_(rx_handler),
    quit_(false),
    ok_(false)
  {
    try_open();

    // 守护线程：断线自动重连
    daemon_thread_ = std::thread{[this] {
      while (!quit_) {
        std::this_thread::sleep_for(100ms);

        if (ok_) continue;  // 链路健康，什么都不做

        // 收尸：先等读线程完全退出，再关串口重连
        if (read_thread_.joinable()) read_thread_.join();
        {
          std::lock_guard<std::mutex> lock(mutex_);
          close();
          try_open();
        }
      }
    }};
  }

  ~SocketUART()
  {
    quit_ = true;
    if (daemon_thread_.joinable()) daemon_thread_.join();
    if (read_thread_.joinable()) read_thread_.join();
    close();
    tools::logger()->info("SocketUART destructed.");
  }

  // 发送字节流，返回实际写入字节数；失败返回 0（不抛异常，避免打断控制循环）
  size_t write(const uint8_t * data, size_t size)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!serial_.isOpen()) return 0;
    try {
      return serial_.write(data, size);
    } catch (const std::exception & e) {
      tools::logger()->warn("SocketUART::write() failed: {}", e.what());
      return 0;
    }
  }

  size_t write(const std::string & data)
  {
    return write(reinterpret_cast<const uint8_t *>(data.data()), data.size());
  }

  bool isOpen() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return serial_.isOpen();
  }

private:
  std::string port_;
  uint32_t baudrate_;
  std::function<void(const uint8_t *, size_t)> rx_handler_;

  std::atomic<bool> quit_;
  std::atomic<bool> ok_;

  mutable std::mutex mutex_;  // 保护 serial_ 的 open/close/write
  serial::Serial serial_;

  std::thread read_thread_;
  std::thread daemon_thread_;

  uint8_t rx_buf_[256];

  void open()
  {
    serial_.setPort(port_);
    serial_.setBaudrate(baudrate_);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(50);
    serial_.setTimeout(timeout);
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.open();
    serial_.flush();

    ok_ = true;  // 先置健康，读线程出错时再置 false

    read_thread_ = std::thread([this] {
      while (!quit_) {
        try {
          read_once();
        } catch (const std::exception & e) {
          tools::logger()->warn("SocketUART::read() failed: {}", e.what());
          ok_ = false;
          break;
        }
      }
    });

    tools::logger()->info("SocketUART [{}] opened, {} baud", port_, baudrate_);
  }

  void try_open()
  {
    try {
      open();
    } catch (const std::exception & e) {
      tools::logger()->warn("SocketUART::open() failed: {}", e.what());
    }
  }

  void read_once()
  {
    serial_.waitReadable();  // 阻塞等待(带超时)，出错抛异常
    size_t n = serial_.available();
    if (n == 0) return;  // 超时，无数据
    size_t len = std::min(n, sizeof(rx_buf_));
    serial_.read(rx_buf_, len);
    if (rx_handler_) rx_handler_(rx_buf_, len);
  }

  void close()
  {
    try {
      if (serial_.isOpen()) serial_.close();
    } catch (const std::exception & e) {
      tools::logger()->warn("SocketUART::close() failed: {}", e.what());
    }
  }
};

}  // namespace io

#endif  // IO__SOCKETUART_HPP
