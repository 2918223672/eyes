add_library(io OBJECT
  camera.cpp
  cboard.cpp
  hikrobot/hikrobot.cpp
  usbcamera/usbcamera.cpp
  gimbal/gimbal.cpp
  dm_imu/dm_imu.cpp
  ros2/publish2nav.cpp
  ros2/ros2.cpp
  ros2/subscribe2nav.cpp
  serial/src/serial.cc
  serial/src/impl/unix.cc
  serial/src/impl/list_ports/list_ports_linux.cc
)

target_include_directories(io PUBLIC hikrobot/include serial/include ${std_msgs_INCLUDE_DIRS} ${sp_msgs_INCLUDE_DIRS})

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(io PUBLIC hikrobot/lib/amd64)
else()
  target_link_directories(io PUBLIC hikrobot/lib/arm64)
endif()

target_link_libraries(io PUBLIC tools Eigen3::Eigen ${OpenCV_LIBS} spdlog::spdlog fmt::fmt yaml-cpp MvCameraControl usb-1.0 rclcpp::rclcpp ${std_msgs_LIBRARIES} ${sp_msgs_LIBRARIES})
