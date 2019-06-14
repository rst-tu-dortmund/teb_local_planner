cmake_minimum_required(VERSION 3.5)
project(teb_local_planner)

# Set to Release in order to speed up the program significantly
#set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

## Find catkin macros and libraries
find_package(ament_cmake REQUIRED)
find_package(costmap_converter REQUIRED)
find_package(dwb_core REQUIRED)
find_package(dwb_critics REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
#find_package(interactive_markers REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(motion_msg REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(teb_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

message(STATUS "System: ${CMAKE_SYSTEM}")
## System dependencies are found with CMake's conventions
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
message(STATUS "${CMAKE_MODULE_PATH}")
find_package(SUITESPARSE REQUIRED)
find_package(G2O REQUIRED)
find_package(Boost REQUIRED)

# Eigen3 FindScript Backward compatibility (ubuntu saucy)
# Since FindEigen.cmake is deprecated starting from jade.
if (EXISTS "FindEigen3.cmake")
  find_package(Eigen3 REQUIRED)
  set(Eigen_INCLUDE_DIRS ${Eigen3_INCLUDE_DIRS})
elseif (EXISTS "FindEigen.cmake")
  find_package(Eigen REQUIRED)
elseif (EXISTS "FindEigen.cmake")
  message(WARNING "No findEigen cmake script found. You must provde one of them,
  e.g. by adding it to ${PROJECT_SOURCE_DIR}/cmake_modules.")
endif (EXISTS "FindEigen3.cmake")

set(EXTERNAL_INCLUDE_DIRS ${Eigen_INCLUDE_DIRS} ${SUITESPARSE_INCLUDE_DIRS} ${G2O_INCLUDE_DIR})
set(EXTERNAL_LIBS ${Boost_LIBRARIES} ${SUITESPARSE_LIBRARIES} ${G2O_LIBRARIES})

#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
#generate_dynamic_reconfigure_options(
#  cfg/TebLocalPlannerReconfigure.cfg
#)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${costmap_converter_INCLUDE_DIRS}
  ${dwb_core_INCLUDE_DIRS}
  ${dwb_critics_INCLUDE_DIRS}
  ${nav_2d_utils_INCLUDE_DIRS}
  ${nav2_costmap_2d_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${motion_msg_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${pluginlib_INCLUDE_DIRS}
  ${teb_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  ${EXTERNAL_INCLUDE_DIRS}
)

## Build the teb_local_planner library

add_library(teb_local_planner SHARED
   src/timed_elastic_band.cpp
   src/optimal_planner.cpp
   src/obstacles.cpp
   src/visualization.cpp
   src/recovery_behaviors.cpp
   src/teb_config.cpp
   src/homotopy_class_planner.cpp
   src/teb_local_planner_ros.cpp
   src/graph_search.cpp
)

# Dynamic reconfigure: make sure configure headers are built before any node using them
#add_dependencies(teb_local_planner ${PROJECT_NAME}_gencfg)
# Generate messages before compiling the lib
#add_dependencies(teb_local_planner ${PROJECT_NAME}_generate_messages_cpp)
set(ament_dependencies
    costmap_converter
    dwb_core
    dwb_critics
    nav_2d_utils
    nav2_costmap_2d
    geometry_msgs
    nav_msgs
    rclcpp
    std_msgs
    pluginlib
    teb_msgs
    tf2
    tf2_eigen
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
)
ament_target_dependencies(teb_local_planner
   ${ament_dependencies}
)
target_link_libraries(teb_local_planner
   ${EXTERNAL_LIBS}
)

add_executable(teb_local_planner_node
    src/teb_local_planner_node.cpp
)
ament_target_dependencies(teb_local_planner_node
    motion_msg
)
target_link_libraries(teb_local_planner_node
    teb_local_planner
    ${EXTERNAL_LIBS}
)

## Build additional executables
#add_executable(test_optim_node src/test_optim_node.cpp)

#target_link_libraries(test_optim_node
#   teb_local_planner
#   ${EXTERNAL_LIBS}
#)


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS teb_local_planner
   DESTINATION lib
)
install(TARGETS teb_local_planner_node
   DESTINATION bin
)

## Mark cpp header files for installation
install(DIRECTORY include/
   DESTINATION include/
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  teb_local_planner_plugin.xml
  DESTINATION share
)

#install(DIRECTORY
#  launch cfg scripts
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#  PATTERN ".svn" EXCLUDE
#)

ament_export_include_directories(include)
ament_export_libraries(teb_local_planner)
ament_export_dependencies(${ament_dependencies})
pluginlib_export_plugin_description_file(teb_local_planner teb_local_planner_plugin.xml)
ament_package()

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(homotopy_class_planner_test
      test/homotopy_class_planner_test.cpp
  )
  target_link_libraries(homotopy_class_planner_test
      teb_local_planner
  )
endif()
## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_teb_local_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
