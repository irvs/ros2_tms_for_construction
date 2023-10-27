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
  ament_cmake
  behaviortree_cpp_v3
  nlohmann_json
  geometry_msgs
  nav2_msgs
  rclpy)

add_library(${PROJECT_NAME} SHARED
  src/subtask_base.cpp
  src/zx120/subtasks.cpp
  src/zx200/subtasks.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} ${dependencies_cpp})

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
  scripts/task_generator.py
  scripts/task_remover.py
  DESTINATION lib/${PROJECT_NAME}
)

# -- end Python

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


install(
  TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


ament_export_dependencies(${dependencies_cpp})
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})  
ament_export_include_directories(include)
ament_package()