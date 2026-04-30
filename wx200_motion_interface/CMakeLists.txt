cmake_minimum_required(VERSION 3.8)
project(wx200_motion_interface)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToPose.action"
  DEPENDENCIES geometry_msgs std_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
