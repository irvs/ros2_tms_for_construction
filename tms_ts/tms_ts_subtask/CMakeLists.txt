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
find_package(Qt5Widgets REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/leaf_node_base.cpp
  src/subtask_node_base.cpp
  src/common/wait_for_click_node.cpp)


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

# Common Subtask Nodes 
add_executable(wait_for_click_button src/common/wait_for_click_button.cpp)

# Subtask Nodes for D37PXI 
add_executable(subtask_d37pxi_follow_waypoints_deg src/OPERA/d37pxi/subtask_d37pxi_follow_waypoints_deg.cpp)
add_executable(subtask_d37pxi_follow_waypoints src/OPERA/d37pxi/subtask_d37pxi_follow_waypoints.cpp)
add_executable(subtask_d37pxi_navigate_anywhere_deg src/OPERA/d37pxi/subtask_d37pxi_navigate_anywhere_deg.cpp)
add_executable(subtask_d37pxi_navigate_anywhere src/OPERA/d37pxi/subtask_d37pxi_navigate_anywhere.cpp)
add_executable(subtask_d37pxi_navigate_through_poses_deg src/OPERA/d37pxi/subtask_d37pxi_navigate_through_poses_deg.cpp)
add_executable(subtask_d37pxi_navigate_through_poses src/OPERA/d37pxi/subtask_d37pxi_navigate_through_poses.cpp)

# Subtask Nodes for MST2200
add_executable(subtask_mst2200_follow_waypoints_deg src/FUJITA/mst2200/subtask_mst2200_follow_waypoints_deg.cpp)
add_executable(subtask_mst2200_follow_waypoints src/FUJITA/mst2200/subtask_mst2200_follow_waypoints.cpp)
add_executable(subtask_mst2200_navigate_anywhere_deg src/FUJITA/mst2200/subtask_mst2200_navigate_anywhere_deg.cpp)
add_executable(subtask_mst2200_navigate_anywhere src/FUJITA/mst2200/subtask_mst2200_navigate_anywhere.cpp)
add_executable(subtask_mst2200_navigate_through_poses_deg src/FUJITA/mst2200/subtask_mst2200_navigate_through_poses_deg.cpp)
add_executable(subtask_mst2200_navigate_through_poses src/FUJITA/mst2200/subtask_mst2200_navigate_through_poses.cpp)
add_executable(subtask_mst2200_release_soil src/FUJITA/mst2200/subtask_mst2200_release_soil.cpp)
add_executable(subtask_mst2200_swing src/FUJITA/mst2200/subtask_mst2200_swing.cpp)

# Subtask Nodes for IC120
add_executable(subtask_ic120_follow_waypoints_deg src/OPERA/ic120/subtask_ic120_follow_waypoints_deg.cpp)
add_executable(subtask_ic120_follow_waypoints src/OPERA/ic120/subtask_ic120_follow_waypoints.cpp)
add_executable(subtask_ic120_navigate_anywhere_deg src/OPERA/ic120/subtask_ic120_navigate_anywhere_deg.cpp)
add_executable(subtask_ic120_navigate_anywhere src/OPERA/ic120/subtask_ic120_navigate_anywhere.cpp)
add_executable(subtask_ic120_navigate_through_poses_deg src/OPERA/ic120/subtask_ic120_navigate_through_poses_deg.cpp)
add_executable(subtask_ic120_navigate_through_poses src/OPERA/ic120/subtask_ic120_navigate_through_poses.cpp)
add_executable(subtask_ic120_release_soil src/OPERA/ic120/subtask_ic120_release_soil.cpp)

# Subtask Nodes for ZX200
add_executable(subtask_zx200_change_pose src/OPERA/zx200/subtask_zx200_change_pose.cpp)
add_executable(subtask_zx200_excavate_simple_plan src/OPERA/zx200/subtask_zx200_excavate_simple_plan.cpp)
add_executable(subtask_zx200_excavate_simple src/OPERA/zx200/subtask_zx200_excavate_simple.cpp)
add_executable(subtask_zx200_release_simple src/OPERA/zx200/subtask_zx200_release_simple.cpp)
add_executable(subtask_zx200_follow_waypoints src/OPERA/zx200/subtask_zx200_follow_waypoints.cpp)
add_executable(subtask_zx200_follow_waypoints_deg src/OPERA/zx200/subtask_zx200_follow_waypoints_deg.cpp)
add_executable(subtask_zx200_navigate_anywhere src/OPERA/zx200/subtask_zx200_navigate_anywhere.cpp)
add_executable(subtask_zx200_navigate_anywhere_deg src/OPERA/zx200/subtask_zx200_navigate_anywhere_deg.cpp)
add_executable(subtask_zx200_navigate_through_poses src/OPERA/zx200/subtask_zx200_navigate_through_poses.cpp)
add_executable(subtask_zx200_navigate_through_poses_deg src/OPERA/zx200/subtask_zx200_navigate_through_poses_deg.cpp)

# Substak Nodes for MST110CR
add_executable(subtask_mst110cr_follow_waypoints src/OPERA/mst110cr/subtask_mst110cr_follow_waypoints.cpp)
add_executable(subtask_mst110cr_follow_waypoints_deg src/OPERA/mst110cr/subtask_mst110cr_follow_waypoints_deg.cpp)
add_executable(subtask_mst110cr_navigate_anywhere src/OPERA/mst110cr/subtask_mst110cr_navigate_anywhere.cpp)
add_executable(subtask_mst110cr_navigate_anywhere_deg src/OPERA/mst110cr/subtask_mst110cr_navigate_anywhere_deg.cpp)
add_executable(subtask_mst110cr_navigate_through_poses src/OPERA/mst110cr/subtask_mst110cr_navigate_through_poses.cpp)
add_executable(subtask_mst110cr_navigate_through_poses_deg src/OPERA/mst110cr/subtask_mst110cr_navigate_through_poses_deg.cpp)
add_executable(subtask_mst110cr_release_soil src/OPERA/mst110cr/subtask_mst110cr_release_soil.cpp)
add_executable(subtask_mst110cr_swing src/OPERA/mst110cr/subtask_mst110cr_swing.cpp)


set(TARGETS
  wait_for_click_button
  subtask_d37pxi_follow_waypoints_deg
  subtask_d37pxi_follow_waypoints
  subtask_d37pxi_navigate_anywhere_deg
  subtask_d37pxi_navigate_anywhere
  subtask_d37pxi_navigate_through_poses_deg
  subtask_d37pxi_navigate_through_poses
  subtask_mst2200_follow_waypoints_deg
  subtask_mst2200_follow_waypoints
  subtask_mst2200_navigate_anywhere_deg
  subtask_mst2200_navigate_anywhere
  subtask_mst2200_navigate_through_poses_deg
  subtask_mst2200_navigate_through_poses
  subtask_mst2200_release_soil
  subtask_mst2200_swing
  subtask_ic120_follow_waypoints_deg
  subtask_ic120_follow_waypoints
  subtask_ic120_navigate_anywhere_deg
  subtask_ic120_navigate_anywhere
  subtask_ic120_navigate_through_poses_deg
  subtask_ic120_navigate_through_poses
  subtask_ic120_release_soil
  subtask_zx200_change_pose
  subtask_zx200_excavate_simple_plan
  subtask_zx200_excavate_simple
  subtask_zx200_release_simple
  subtask_zx200_follow_waypoints_deg
  subtask_zx200_follow_waypoints
  subtask_zx200_navigate_anywhere_deg
  subtask_zx200_navigate_anywhere
  subtask_zx200_navigate_through_poses_deg
  subtask_zx200_navigate_through_poses
  subtask_mst110cr_follow_waypoints_deg
  subtask_mst110cr_follow_waypoints
  subtask_mst110cr_navigate_anywhere_deg
  subtask_mst110cr_navigate_anywhere
  subtask_mst110cr_navigate_through_poses_deg
  subtask_mst110cr_navigate_through_poses
  subtask_mst110cr_release_soil
  subtask_mst110cr_swing
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
  com3_msgs
  Qt5Widgets)

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