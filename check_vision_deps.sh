#!/bin/bash
# ===== vision 项目依赖检查脚本 =====

echo "========================================="
echo "  vision 项目依赖检查"
echo "========================================="

# ---- 1. 编译器 ----
echo -e "\n[1] C++17 编译器"
g++ --version 2>/dev/null | head -1 && echo "  ✓ g++ 可用" || echo "  ✗ g++ 未安装"

# ---- 2. CMake ----
echo -e "\n[2] CMake (>= 3.16)"
cmake --version 2>/dev/null | head -1 && echo "  ✓ cmake 可用" || echo "  ✗ cmake 未安装"

# ---- 3. ROS2 ----
echo -e "\n[3] ROS2 (rclcpp, std_msgs, builtin_interfaces)"
dpkg -l | grep -q "ros-.*-rclcpp" && echo "  ✓ rclcpp" || echo "  ✗ rclcpp 未安装"
dpkg -l | grep -q "ros-.*-std-msgs" && echo "  ✓ std_msgs" || echo "  ✗ std_msgs 未安装"
dpkg -l | grep -q "ros-.*-builtin-interfaces" && echo "  ✓ builtin_interfaces" || echo "  ✗ builtin_interfaces 未安装"
dpkg -l | grep -q "ros-.*-rosidl-default-generators" && echo "  ✓ rosidl_default_generators" || echo "  ✗ rosidl_default_generators 未安装"
dpkg -l | grep -q "ros-.*-rosidl-default-runtime" && echo "  ✓ rosidl_default_runtime" || echo "  ✗ rosidl_default_runtime 未安装"

# 检查 ROS2 环境是否可用
if [ -d /opt/ros ]; then
    ROS_DISTRO=$(ls /opt/ros/ 2>/dev/null | head -1)
    echo "  → 检测到 ROS2 发行版: $ROS_DISTRO"
else
    echo "  → 未检测到 /opt/ros 目录"
fi

# ---- 4. 第三方 C++ 库 ----
echo -e "\n[4] 第三方 C++ 库"

# Eigen3
if [ -f /usr/include/eigen3/Eigen/Core ] || [ -f /usr/include/eigen3/eigen3/Eigen/Core ] || [ -f /usr/include/Eigen/Core ]; then
    echo "  ✓ Eigen3"
else
    dpkg -l | grep -q libeigen3-dev && echo "  ✓ Eigen3 (dpkg)" || echo "  ✗ Eigen3 未安装"
fi

# OpenCV
pkg-config --modversion opencv4 2>/dev/null && echo "  ✓ OpenCV ($(pkg-config --modversion opencv4 2>/dev/null))" || {
    dpkg -l | grep -q libopencv-dev && echo "  ✓ OpenCV (dpkg)" || echo "  ✗ OpenCV 未安装"
}

# yaml-cpp
pkg-config --modversion yaml-cpp 2>/dev/null && echo "  ✓ yaml-cpp ($(pkg-config --modversion yaml-cpp 2>/dev/null))" || {
    dpkg -l | grep -q libyaml-cpp-dev && echo "  ✓ yaml-cpp (dpkg)" || echo "  ✗ yaml-cpp 未安装"
}

# spdlog
if [ -f /usr/include/spdlog/spdlog.h ]; then
    echo "  ✓ spdlog"
else
    dpkg -l | grep -q libspdlog-dev && echo "  ✓ spdlog (dpkg)" || echo "  ✗ spdlog 未安装"
fi

# fmt
if [ -f /usr/include/fmt/core.h ]; then
    echo "  ✓ fmt"
else
    dpkg -l | grep -q libfmt-dev && echo "  ✓ fmt (dpkg)" || echo "  ✗ fmt 未安装"
fi

# nlohmann_json
if [ -f /usr/include/nlohmann/json.hpp ]; then
    echo "  ✓ nlohmann_json"
else
    dpkg -l | grep -q nlohmann-json && echo "  ✓ nlohmann_json (dpkg)" || echo "  ✗ nlohmann_json 未安装"
fi

# libusb-1.0
pkg-config --modversion libusb-1.0 2>/dev/null && echo "  ✓ libusb-1.0 ($(pkg-config --modversion libusb-1.0 2>/dev/null))" || {
    dpkg -l | grep -q libusb-1.0-0-dev && echo "  ✓ libusb-1.0 (dpkg)" || echo "  ✗ libusb-1.0 未安装"
}

# ---- 5. 硬件/厂商 SDK ----
echo -e "\n[5] 硬件/厂商 SDK"

# OpenVINO
if [ -d /opt/intel/openvino_2024.6.0 ]; then
    echo "  ✓ OpenVINO (/opt/intel/openvino_2024.6.0)"
elif [ -d /opt/intel/openvino_2024 ]; then
    OPENVINO_VER=$(ls /opt/intel/ | grep openvino | head -1)
    echo "  ⚠ OpenVINO 版本不匹配: $OPENVINO_VER (需要 2024.6.0)"
else
    echo "  ✗ OpenVINO 未安装 (需要 /opt/intel/openvino_2024.6.0)"
fi

# 海康相机 SDK (项目中自带)
if [ -f /home/chen/Desktop/vision/src/sp_vision/io/hikrobot/lib/amd64/libMvCameraControl.so ]; then
    echo "  ✓ MvCameraControl (项目自带)"
else
    echo "  ⚠ MvCameraControl 未找到（检查项目是否完整复制）"
fi

# ---- 6. colcon (ROS2 构建工具) ----
echo -e "\n[6] colcon 构建工具"
colcon --version 2>/dev/null && echo "  ✓ colcon 可用" || echo "  ✗ colcon 未安装（sudo apt install python3-colcon-common-extensions）"

echo -e "\n========================================="
echo "  检查完毕"
echo "========================================="
