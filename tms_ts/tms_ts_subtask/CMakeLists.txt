cmake_minimum_required(VERSION 3.8)
project(tms_ts_subtask)

option(BUILD_TESTING "Build tests" ON)

set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set (CMAKE_CXX_STANDARD 17)
set(CMAKE_PREFIX_PATH /user/local/lib)

# find dependencies
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
find_package(tms_msg_ts REQUIRED)
find_package(tms_msg_rp REQUIRED)
find_package(tms_msg_db REQUIRED)
find_package(glog REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(com3_msgs)

add_library(${PROJECT_NAME} SHARED
  src/leaf_node_base.cpp
  src/subtask_node_base.cpp)


target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/local/include/mongocxx/v_noabi
  /usr/local/include/bsoncxx/v_noabi)

target_link_directories(${PROJECT_NAME} PUBLIC
  /usr/local/lib)

target_link_libraries(${PROJECT_NAME} 
mongocxx
bsoncxx)

# Subtask Nodes for MST2200
add_executable(subtask_mst2200_follow_waypoints_deg_server src/FUJITA/mst2200/subtask_mst2200_follow_waypoints_deg_server.cpp)
add_executable(subtask_mst2200_follow_waypoints_server src/FUJITA/mst2200/subtask_mst2200_follow_waypoints_server.cpp)
add_executable(subtask_mst2200_navigate_anywhere_deg_server src/FUJITA/mst2200/subtask_mst2200_navigate_anywhere_deg_server.cpp)
add_executable(subtask_mst2200_navigate_anywhere_server src/FUJITA/mst2200/subtask_mst2200_navigate_anywhere_server.cpp)
add_executable(subtask_mst2200_navigate_through_poses_deg_server src/FUJITA/mst2200/subtask_mst2200_navigate_through_poses_deg_server.cpp)
add_executable(subtask_mst2200_navigate_through_poses_server src/FUJITA/mst2200/subtask_mst2200_navigate_through_poses_server.cpp)
add_executable(subtask_mst2200_release_soil_server src/FUJITA/mst2200/subtask_mst2200_release_soil_server.cpp)

# Subtask Nodes for IC120
add_executable(subtask_ic120_follow_waypoints_deg_server src/OPERA/ic120/subtask_ic120_follow_waypoints_deg_server.cpp)
add_executable(subtask_ic120_follow_waypoints_server src/OPERA/ic120/subtask_ic120_follow_waypoints_server.cpp)
add_executable(subtask_ic120_navigate_anywhere_deg_server src/OPERA/ic120/subtask_ic120_navigate_anywhere_deg_server.cpp)
add_executable(subtask_ic120_navigate_anywhere_server src/OPERA/ic120/subtask_ic120_navigate_anywhere_server.cpp)
add_executable(subtask_ic120_navigate_through_poses_deg_server src/OPERA/ic120/subtask_ic120_navigate_through_poses_deg_server.cpp)
add_executable(subtask_ic120_navigate_through_poses_server src/OPERA/ic120/subtask_ic120_navigate_through_poses_server.cpp)
add_executable(subtask_ic120_release_soil_server src/OPERA/ic120/subtask_ic120_release_soil_server.cpp)

# Sample Subtask Nodes for ZX120 and ZX200
add_executable(zx120_sample_boom_subtask src/OPERA/sample/zx120/sample_boom_subtask.cpp)
add_executable(zx120_sample_swing_subtask src/OPERA/sample/zx120/sample_swing_subtask.cpp)
add_executable(zx120_sample_arm_subtask src/OPERA/sample/zx120/sample_arm_subtask.cpp)
add_executable(zx120_sample_bucket_subtask src/OPERA/sample/zx120/sample_bucket_subtask.cpp)
add_executable(zx200_sample_boom_subtask src/OPERA/sample/zx200/sample_boom_subtask.cpp)
add_executable(zx200_sample_swing_subtask src/OPERA/sample/zx200/sample_swing_subtask.cpp)
add_executable(zx200_sample_arm_subtask src/OPERA/sample/zx200/sample_arm_subtask.cpp)
add_executable(zx200_sample_bucket_subtask src/OPERA/sample/zx200/sample_bucket_subtask.cpp)

# Subtask Nodes for ZX200
add_executable(subtask_zx200_change_pose src/OPERA/zx200/subtask_zx200_change_pose.cpp)
add_executable(subtask_zx200_excavate_simple src/OPERA/zx200/subtask_zx200_excavate_simple.cpp)
add_executable(subtask_zx200_release_simple src/OPERA/zx200/subtask_zx200_release_simple.cpp)
add_executable(subtask_zx200_follow_waypoints_server src/OPERA/zx200/subtask_zx200_follow_waypoints_server.cpp)
add_executable(subtask_zx200_navigate_anywhere_server src/OPERA/zx200/subtask_zx200_navigate_anywhere_server.cpp)
add_executable(subtask_zx200_navigate_through_poses_server src/OPERA/zx200/subtask_zx200_navigate_through_poses_server.cpp)

# Substak Nodes for MST110CR
add_executable(subtask_mst110cr_follow_waypoints_server src/OPERA/mst110cr/subtask_mst110cr_follow_waypoints_server.cpp)
add_executable(subtask_mst110cr_navigate_anywhere_server src/OPERA/mst110cr/subtask_mst110cr_navigate_anywhere_server.cpp)
add_executable(subtask_mst110cr_navigate_through_poses_server src/OPERA/mst110cr/subtask_mst110cr_navigate_through_poses_server.cpp)
add_executable(subtask_mst110cr_release_soil_server src/OPERA/mst110cr/subtask_mst110cr_release_soil_server.cpp)

set(TARGETS
  subtask_mst2200_follow_waypoints_deg_server
  subtask_mst2200_follow_waypoints_server
  subtask_mst2200_navigate_anywhere_deg_server
  subtask_mst2200_navigate_anywhere_server
  subtask_mst2200_navigate_through_poses_deg_server
  subtask_mst2200_navigate_through_poses_server
  subtask_mst2200_release_soil_server
  subtask_ic120_follow_waypoints_deg_server
  subtask_ic120_follow_waypoints_server
  subtask_ic120_navigate_anywhere_deg_server
  subtask_ic120_navigate_anywhere_server
  subtask_ic120_navigate_through_poses_deg_server
  subtask_ic120_navigate_through_poses_server
  subtask_ic120_release_soil_server
  zx120_sample_boom_subtask
  zx120_sample_swing_subtask
  zx120_sample_arm_subtask
  zx120_sample_bucket_subtask
  zx200_sample_boom_subtask
  zx200_sample_swing_subtask
  zx200_sample_arm_subtask
  zx200_sample_bucket_subtask
  subtask_zx200_change_pose
  subtask_zx200_excavate_simple
  subtask_zx200_release_simple
  subtask_zx200_follow_waypoints_server
  subtask_zx200_navigate_anywhere_server
  subtask_zx200_navigate_through_poses_server
  subtask_mst110cr_follow_waypoints_server
  subtask_mst110cr_navigate_anywhere_server
  subtask_mst110cr_navigate_through_poses_server
  subtask_mst110cr_release_soil_server
)

set(COMMON_INCLUDE_DIRS
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/local/include/mongocxx/v_noabi
  /usr/local/include/bsoncxx/v_noabi
)

set(COMMON_LINK_DIR /usr/local/lib)

set(COMMON_LIBS
  ${PROJECT_NAME}
  mongocxx
  bsoncxx
  glog::glog
)

set(dependencies_cpp
  rclcpp
  rclcpp_action
  std_msgs
  ament_cmake
  behaviortree_cpp_v3
  nlohmann_json
  geometry_msgs
  tms_msg_ts
  tms_msg_rp
  tms_msg_db
  nav2_msgs
  com3_msgs)

foreach(target IN LISTS TARGETS)
target_include_directories(${target} PUBLIC ${COMMON_INCLUDE_DIRS})
  target_link_directories(${target} PUBLIC ${COMMON_LINK_DIR})
  target_link_libraries(${target} ${COMMON_LIBS})
endforeach()

ament_target_dependencies(${PROJECT_NAME} ${dependencies_cpp})

install(TARGETS ${TARGETS}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(
  TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()


ament_export_dependencies(${dependencies_cpp})
ament_export_libraries(${PROJECT_NAME} mongocxx bsoncxx)
ament_export_targets(${PROJECT_NAME})  
ament_export_include_directories(include)
ament_package()