add_library(omniperception OBJECT
  decider.cpp
  perceptron.cpp
)

target_link_libraries(omniperception PUBLIC Eigen3::Eigen ${OpenCV_LIBS} openvino)
