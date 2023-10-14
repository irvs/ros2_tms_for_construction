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
find_package(tms_msg_ts REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  std_msgs
  behaviortree_cpp_v3
  tms_msg_ts
  nlohmann_json
  geometry_msgs
  nav2_msgs)


add_executable(subtask_nodes src/subtask_nodes.cpp)
# target_include_directories(subtask_nodes PUBLIC include)
ament_target_dependencies(subtask_nodes ${dependencies})

install(TARGETS
  subtask_nodes
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
