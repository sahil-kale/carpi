cmake_minimum_required(VERSION 3.8)
project(robot_controller_interface)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotCommandRaw.msg"
  "msg/BuzzerCommand.msg"
  "msg/LedCommand.msg"
)

ament_package()
