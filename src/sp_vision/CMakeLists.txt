# ===== [1. 基础配置] 项目元信息和编译标准 =====
cmake_minimum_required(VERSION 3.16)
project(sp_vision VERSION 0.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)          # 使用 C++17 标准
set(CMAKE_CXX_STANDARD_REQUIRED ON) # 编译器必须支持 C++17，否则报错
set(CMAKE_BUILD_TYPE Release)       # Release 模式：-O3 优化，不包含调试符号

# ===== [2. 依赖查找] ROS2 + 第三方库 =====
# --- ROS2 核心依赖 ---
find_package(ament_cmake REQUIRED)  # ROS2 构建系统（colcon/ament）
find_package(sp_msgs REQUIRED)      # 自定义消息包（AutoaimTargetMsg 等）
find_package(rclcpp REQUIRED)       # ROS2 C++ 客户端（Node, Publisher, Subscriber）
find_package(std_msgs REQUIRED)     # ROS2 标准消息（String 等）

# --- 第三方 C++ 库 ---
find_package(Eigen3 REQUIRED)       # 线性代数（矩阵乘法、SVD、四元数）
find_package(OpenCV REQUIRED)       # 图像处理（imread, cvtColor, solvePnP）
find_package(yaml-cpp REQUIRED)     # YAML 配置文件读写
find_package(spdlog REQUIRED)       # 高性能异步日志
find_package(fmt REQUIRED)          # 类型安全的字符串格式化（替代 printf）
find_package(nlohmann_json REQUIRED)# JSON 序列化（Plotter UDP 调试用）

# ===== [3. 子模块] OBJECT 库 =====
# 让所有 .cpp 用项目根作为 include 起点
# 效果：#include "tools/math/math_tools.hpp" 而非 "../../tools/..."
include_directories(${PROJECT_SOURCE_DIR})

# 每个子目录编译成一个 OBJECT 库，不生成 .so/.a，只生成 .o
# OBJECT 库的好处：所有 .o 直接链接到最终可执行文件，
# 无动态库搜索路径问题，部署时只有一个二进制文件
add_subdirectory(tools)              # 公共工具（数学、日志、控制、线程池）
add_subdirectory(io)                 # 硬件驱动（相机、云台、CAN、IMU、ROS2 通信）
add_subdirectory(tasks/auto_aim)     # 自瞄算法（检测、跟踪、解算、瞄准、MPC）

# ===== [4. 可执行目标] 生产程序 + 测试程序 =====

# --- 生产可执行文件 ---
add_executable(sentry src/sentry.cpp)
target_link_libraries(sentry tools io auto_aim)

add_executable(zero src/zero.cpp)
target_link_libraries(zero tools io auto_aim)

add_executable(mt_sentry src/mt_sentry.cpp)

add_executable(tracking_test src/tracking_test.cpp)
target_link_libraries(tracking_test tools io auto_aim)
target_link_libraries(mt_sentry tools io auto_aim)

# --- 硬件驱动测试 ---
add_executable(cboard_test tests/cboard_test.cpp)
target_link_libraries(cboard_test tools io)

add_executable(dm_test tests/dm_test.cpp)
target_link_libraries(dm_test tools io)

add_executable(gimbal_test tests/gimbal_test.cpp)
target_link_libraries(gimbal_test tools io)

add_executable(gimbal_response_test tests/gimbal_response_test.cpp)
target_link_libraries(gimbal_response_test tools io)

add_executable(usbcamera_test tests/usbcamera_test.cpp)
target_link_libraries(usbcamera_test tools io)

add_executable(multi_usbcamera_test tests/multi_usbcamera_test.cpp)
target_link_libraries(multi_usbcamera_test tools io)

# --- 算法模块测试 ---
add_executable(auto_aim_test tests/auto_aim_test.cpp)
target_link_libraries(auto_aim_test tools io auto_aim)

add_executable(detector_video_test tests/detector_video_test.cpp)
target_link_libraries(detector_video_test tools io auto_aim)

add_executable(fire_test tests/fire_test.cpp)
target_link_libraries(fire_test tools io)

add_executable(planner_test tests/planner_test.cpp)
target_link_libraries(planner_test tools io auto_aim)

add_executable(planner_test_offline tests/planner_test_offline.cpp)
target_link_libraries(planner_test_offline tools io auto_aim)

add_executable(handeye_test tests/handeye_test.cpp)
target_link_libraries(handeye_test tools io auto_aim)

add_executable(camera_test tests/camera_test.cpp)
target_link_libraries(camera_test tools io auto_aim)

add_executable(math_test tests/math_test.cpp)
target_link_libraries(math_test tools)

# --- 集成 / ROS2 测试 ---
add_executable(camera_detect_test tests/camera_detect_test.cpp)
target_link_libraries(camera_detect_test tools io auto_aim)

add_executable(camera_thread_test tests/camera_thread_test.cpp)
target_link_libraries(camera_thread_test tools io auto_aim)

add_executable(usbcamera_detect_test tests/usbcamera_detect_test.cpp)
target_link_libraries(usbcamera_detect_test tools io auto_aim)

add_executable(publish_test tests/publish_test.cpp)
target_link_libraries(publish_test tools io)

add_executable(subscribe_test tests/subscribe_test.cpp)
target_link_libraries(subscribe_test tools io)

add_executable(topic_loop_test tests/topic_loop_test.cpp)
target_link_libraries(topic_loop_test tools io)

add_executable(minimum_vision_system tests/minimum_vision_system.cpp)
target_link_libraries(minimum_vision_system tools io auto_aim)

# --- 标定工具 ---
add_executable(calibrate_camera calibration/calibrate_camera.cpp)
target_link_libraries(calibrate_camera tools yaml-cpp)

add_executable(calibrate_handeye calibration/calibrate_handeye.cpp)
target_link_libraries(calibrate_handeye tools yaml-cpp)

add_executable(calibrate_robotworld_handeye calibration/calibrate_robotworld_handeye.cpp)
target_link_libraries(calibrate_robotworld_handeye tools yaml-cpp)

add_executable(capture calibration/capture.cpp)
target_link_libraries(capture tools io)

add_executable(capture_calib calibration/capture_calib.cpp)
target_link_libraries(capture_calib tools io)

add_executable(split_video calibration/split_video.cpp)
target_link_libraries(split_video tools)
