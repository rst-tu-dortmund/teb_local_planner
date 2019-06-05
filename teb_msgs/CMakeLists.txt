cmake_minimum_required(VERSION 3.5)

project(teb_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(costmap_converter_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(teb_msgs
  "msg/FeedbackMsg.msg"
  "msg/TrajectoryMsg.msg"
  "msg/TrajectoryPointMsg.msg"
  DEPENDENCIES builtin_interfaces costmap_converter_msgs geometry_msgs std_msgs
)

ament_package()
