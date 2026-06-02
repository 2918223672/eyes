cmake_minimum_required(VERSION 3.16)

  add_library(tools OBJECT
      math/math_tools.cpp
      math/trajectory.cpp
      utils/logger.cpp
      utils/exiter.cpp
      utils/img_tools.cpp
      utils/crc.cpp
      control/extended_kalman_filter.cpp
      control/pid.cpp
      plotter.cpp
      ransac_sine_fitter.cpp
      recorder.cpp
  )

 target_link_libraries(tools PUBLIC Eigen3::Eigen ${OpenCV_LIBS} spdlog::spdlog fmt::fmt nlohmann_json::nlohmann_json)
