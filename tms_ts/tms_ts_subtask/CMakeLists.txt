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


# ic120
# add_library(ic120_subtask_base_parts  SHARED src/ic120/subtask_base_parts.cpp)
# target_include_directories(ic120_subtask_base_parts PUBLIC include)
# ament_target_dependencies(ic120_subtask_base_parts ${dependencies_cpp})

# add_library(ic120_subtasks SHARED src/ic120/subtasks.cpp)
# target_include_directories(ic120_subtasks PUBLIC include)
# target_link_libraries(ic120_subtasks ic120_subtask_base_parts)
# ament_target_dependencies(ic120_subtasks ${dependencies_cpp})

# add_executable(ic120_task_schedular_manager src/ic120/task_schedular_manager.cpp)
# target_include_directories(ic120_task_schedular_manager PUBLIC include)
# target_link_libraries(ic120_task_schedular_manager ic120_subtasks)
# ament_target_dependencies(ic120_task_schedular_manager ${dependencies_cpp})

# zx120
add_library(zx120_subtask_base_parts  SHARED src/zx120/subtask_base_parts.cpp)
target_include_directories(zx120_subtask_base_parts PUBLIC include)
ament_target_dependencies(zx120_subtask_base_parts ${dependencies_cpp})

add_library(zx120_subtasks SHARED src/zx120/subtasks.cpp)
target_include_directories(zx120_subtasks PUBLIC include)
target_link_libraries(zx120_subtasks zx120_subtask_base_parts)
ament_target_dependencies(zx120_subtasks ${dependencies_cpp})

add_executable(zx120_task_schedular_manager src/zx120/task_schedular_manager.cpp)
target_include_directories(zx120_task_schedular_manager PUBLIC include)
target_link_libraries(zx120_task_schedular_manager zx120_subtasks)
ament_target_dependencies(zx120_task_schedular_manager ${dependencies_cpp})

# zx200
# add_library(zx200_subtask_base_parts  SHARED src/zx200/subtask_base_parts.cpp)
# target_include_directories(zx200_subtask_base_parts PUBLIC include)
# ament_target_dependencies(zx200_subtask_base_parts ${dependencies_cpp})

# add_library(zx200_subtasks SHARED src/zx200/subtasks.cpp)
# target_include_directories(zx200_subtasks PUBLIC include)
# target_link_libraries(zx200_subtasks zx200_subtask_base_parts)
# ament_target_dependencies(zx200_subtasks ${dependencies_cpp})

# add_executable(zx200_task_schedular_manager src/zx200/task_schedular_manager.cpp)
# target_include_directories(zx200_task_schedular_manager PUBLIC include)
# target_link_libraries(zx200_task_schedular_manager zx200_subtasks)
# ament_target_dependencies(zx200_task_schedular_manager ${dependencies_cpp})


install(TARGETS
  # ic120_task_schedular_manager
  zx120_task_schedular_manager
  # zx200_task_schedular_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  # ic120_subtasks
  # ic120_subtask_base_parts
  zx120_subtasks
  zx120_subtask_base_parts
  # zx200_subtasks
  # zx200_subtask_base_parts
  DESTINATION lib/
)

install(DIRECTORY include/
  DESTINATION include/
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
  scripts/task_generator.py
  scripts/task_remover.py
  DESTINATION lib/${PROJECT_NAME}
)

# -- end Python

install(DIRECTORY include config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()
