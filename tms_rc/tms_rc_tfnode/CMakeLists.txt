################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(tms_rc_tfnode)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(std_msgs REQUIRED)
#find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
#find_package(nav_msgs REQUIRED)
#find_package(tms_msg_ss REQUIRED) 
#find_package(tms_msg_rc REQUIRED)
#find_package(tms_msg_db REQUIRED)
#find_package(turtlebot3_msgs REQUIRED)
#find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
#find_package(builtin_interfaces REQUIRED)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

add_executable(Tfnode Tfnode.cpp)
ament_target_dependencies(Tfnode rclcpp geometry_msgs tf2_ros)

install(TARGETS
  Tfnode

  DESTINATION lib/${PROJECT_NAME}
)

ament_package()