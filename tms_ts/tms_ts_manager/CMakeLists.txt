cmake_minimum_required(VERSION 3.8)
project(tms_ts_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set (CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

find_package(tms_ts_subtask REQUIRED)


set(dependencies
  rclcpp
  rclcpp_action
  std_msgs
  ament_cmake
  behaviortree_cpp_v3
  nlohmann_json
  geometry_msgs
  nav2_msgs
  rclpy
  tms_ts_subtask)

# add_executable(task_schedular_manager src/task_schedular_manager.cpp)
# target_include_directories(task_schedular_manager PUBLIC include)
# ament_target_dependencies(task_schedular_manager ${dependencies})

install(TARGETS
  # task_schedular_manager
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
