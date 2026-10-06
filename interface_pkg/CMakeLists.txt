cmake_minimum_required(VERSION 3.8)
project(interface_pkg)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WebcamDetection.msg"
  "msg/RcCarTarget.msg"
  "msg/AmrState.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
