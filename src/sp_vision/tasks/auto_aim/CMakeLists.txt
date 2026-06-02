set(OVINO_DIR /opt/intel/openvino_2024.6.0/runtime)

add_library(auto_aim OBJECT
  aimer.cpp
  armor.cpp
  classifier.cpp
  detector.cpp
  shooter.cpp
  solver.cpp
  target.cpp
  tracker.cpp
  voter.cpp
  yolo.cpp
  multithread/commandgener.cpp
  multithread/mt_detector.cpp
  planner/planner.cpp
  planner/tinympc/admm.cpp
  planner/tinympc/codegen.cpp
  planner/tinympc/rho_benchmark.cpp
  planner/tinympc/tiny_api.cpp
  yolos/yolo11.cpp
  yolos/yolov5.cpp
  yolos/yolov8.cpp
)

target_include_directories(auto_aim PUBLIC ${OVINO_DIR}/include planner/tinympc)
target_link_directories(auto_aim PUBLIC ${OVINO_DIR}/lib/intel64)
target_link_libraries(auto_aim PUBLIC tools Eigen3::Eigen ${OpenCV_LIBS} yaml-cpp fmt::fmt openvino)
