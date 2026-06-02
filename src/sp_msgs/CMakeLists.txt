cmake_minimum_required(VERSION 3.8)
project(sp_msgs)

# 1. 查找依赖项
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED) # 必须添加这一行

# 2. 生成接口
 rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/ImuData.msg"
    "msg/AutoaimTargetMsg.msg"
    "msg/EnemyStatusMsg.msg"
    "msg/Command.msg"
    DEPENDENCIES builtin_interfaces
  )

# 3. 导出依赖（以便视觉项目能找到）
ament_export_dependencies(builtin_interfaces)
ament_package()
