cmake_minimum_required(VERSION 3.20)
project(realtime_servo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RelativeMove.msg"
)
ament_export_dependencies(rosidl_default_runtime)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(jacobian_velctrl_node src/jacobian_velctrl.cpp)

ament_target_dependencies(jacobian_velctrl_node
  rclcpp
  geometry_msgs
  trajectory_msgs
  sensor_msgs
  std_msgs
  tf2_ros
  kdl_parser
  orocos_kdl
)

target_link_libraries(jacobian_velctrl_node Eigen3::Eigen yaml-cpp)

install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS jacobian_velctrl_node #
  DESTINATION lib/${PROJECT_NAME})

rosidl_target_interfaces(jacobian_velctrl_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
