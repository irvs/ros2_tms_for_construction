cmake_minimum_required(VERSION 3.8)
project(tms_ts_subtask)

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

# -- for C++

set(dependencies_cpp
  rclcpp
  rclcpp_action
  std_msgs
  behaviortree_cpp_v3
  nlohmann_json
  geometry_msgs
  nav2_msgs
  ament_cmake_python
  rclpy)

add_executable(subtask_nodes src/subtask_nodes.cpp)
# target_include_directories(subtask_nodes PUBLIC include)
ament_target_dependencies(subtask_nodes ${dependencies_cpp})

install(TARGETS
  subtask_nodes
  DESTINATION lib/${PROJECT_NAME}
)

# -- end C++

# -- for Python

set(dependencies_py
  rclpy
  ament_cmake_python
  rclpy_action
  geometry_msgs
  nav2_msgs
  ament_cmake_python)

ament_python_install_package(scripts)

install(PROGRAMS
  scripts/format_task_sequence.py
  DESTINATION lib/${PROJECT_NAME}
)

# -- end Python

install(DIRECTORY include config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
