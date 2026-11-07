cmake_minimum_required(VERSION 3.8)
project(chassis_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ServoStatus.msg"
  "msg/ServoCmd.msg"
  "msg/ImuData.msg"
  "msg/Xl9535Data.msg"
  "msg/IrData.msg"
  "msg/MotionData.msg"
  "msg/BatteryData.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
