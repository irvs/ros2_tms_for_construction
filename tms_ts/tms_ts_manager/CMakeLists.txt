cmake_minimum_required(VERSION 3.8)
project(tms_ts_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set (CMAKE_CXX_STANDARD 17)

# -- for C++
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(bsoncxx REQUIRED)
find_package(mongocxx REQUIRED)
find_package(tms_ts_subtask REQUIRED)
find_package(tms_msg_ur REQUIRED)

set(dependencies_cpp
  rclcpp
  rclcpp_action
  std_msgs
  ament_cmake
  behaviortree_cpp_v3
  nlohmann_json
  geometry_msgs
  rclpy
  tms_ts_subtask
  bsoncxx
  mongocxx
  tms_msg_ur)

link_directories(
  /usr/local/lib)

link_libraries(
  mongocxx
  bsoncxx)

# ExprTk のインクルードディレクトリを追加
include_directories(/usr/local/include)


add_executable(task_schedular_manager src/task_schedular_manager.cpp)
ament_target_dependencies(task_schedular_manager ${dependencies_cpp})

add_executable(task_schedular_manager2 src/task_schedular_manager2.cpp)
ament_target_dependencies(task_schedular_manager2 ${dependencies_cpp})

# -- end C++

# -- for Python

set(dependencies_py
  rclpy
  ament_cmake_python
  rclpy_action
  geometry_msgs
  ament_cmake_python)

ament_python_install_package(scripts)

install(PROGRAMS
  scripts/task_generator.py
  scripts/task_remover.py
  DESTINATION lib/${PROJECT_NAME}
)

# -- end Python

install(TARGETS
  task_schedular_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  task_schedular_manager2
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
