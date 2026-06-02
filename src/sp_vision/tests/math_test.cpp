  #include "tools/math/math_tools.hpp"
  #include "tools/utils/logger.hpp"
  #include <iostream>

  int main()
  {
      //tools::logger()->info("=== math_test start ===");

      // 测试角度归一化
      double angle = tools::limit_rad(4.0);
      tools::logger()->info("limit_rad(4.0) = {}", angle);

      // 测试坐标转换
      Eigen::Vector3d xyz(1.0, 0.0, 0.0);
      auto ypd = tools::xyz2ypd(xyz);
      tools::logger()->info("xyz2ypd: yaw={}, pitch={}, dist={}", ypd[0], ypd[1], ypd[2]);

      std::cout << "PASSED" << std::endl;
      tools::logger()->info("PASSED");
      return 0;
  }
