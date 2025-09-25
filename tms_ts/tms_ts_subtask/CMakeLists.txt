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
find_package(sensor_msgs REQUIRED)
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

# Subtask Nodes for Bulldozer 
add_executable(subtask_bulldozer_follow_waypoints_deg src/Bulldozer/subtask_bulldozer_follow_waypoints_deg.cpp)
add_executable(subtask_bulldozer_follow_waypoints src/Bulldozer/subtask_bulldozer_follow_waypoints.cpp)
add_executable(subtask_bulldozer_navigate_anywhere_deg src/Bulldozer/subtask_bulldozer_navigate_anywhere_deg.cpp)
add_executable(subtask_bulldozer_navigate_anywhere src/Bulldozer/subtask_bulldozer_navigate_anywhere.cpp)
add_executable(subtask_bulldozer_navigate_through_poses_deg src/Bulldozer/subtask_bulldozer_navigate_through_poses_deg.cpp)
add_executable(subtask_bulldozer_navigate_through_poses src/Bulldozer/subtask_bulldozer_navigate_through_poses.cpp)

# Subtask Nodes for CrawlerDump
add_executable(subtask_crawlerdump_follow_waypoints_deg src/CrawlerDump/subtask_crawlerdump_follow_waypoints_deg.cpp)
add_executable(subtask_crawlerdump_follow_waypoints src/CrawlerDump/subtask_crawlerdump_follow_waypoints.cpp)
add_executable(subtask_crawlerdump_navigate_anywhere_deg src/CrawlerDump/subtask_crawlerdump_navigate_anywhere_deg.cpp)
add_executable(subtask_crawlerdump_navigate_anywhere src/CrawlerDump/subtask_crawlerdump_navigate_anywhere.cpp)
add_executable(subtask_crawlerdump_navigate_through_poses_deg src/CrawlerDump/subtask_crawlerdump_navigate_through_poses_deg.cpp)
add_executable(subtask_crawlerdump_navigate_through_poses src/CrawlerDump/subtask_crawlerdump_navigate_through_poses.cpp)
add_executable(subtask_crawlerdump_release_soil src/CrawlerDump/subtask_crawlerdump_release_soil.cpp)
add_executable(subtask_crawlerdump_swing src/CrawlerDump/subtask_crawlerdump_swing.cpp)
add_executable(subtask_crawlerdump_swing_align_to_heading src/CrawlerDump/subtask_crawlerdump_swing_align_to_heading.cpp)

# Subtask Nodes for Excavator
add_executable(subtask_excavator_change_pose src/Excavator/subtask_excavator_change_pose.cpp)
add_executable(subtask_excavator_level src/Excavator/subtask_excavator_level.cpp)
add_executable(subtask_excavator_change_pose_plan src/Excavator/subtask_excavator_change_pose_plan.cpp)
add_executable(subtask_excavator_excavate_simple_plan src/Excavator/subtask_excavator_excavate_simple_plan.cpp)
add_executable(subtask_excavator_excavate_simple src/Excavator/subtask_excavator_excavate_simple.cpp)
add_executable(subtask_excavator_release_simple src/Excavator/subtask_excavator_release_simple.cpp)
add_executable(subtask_excavator_follow_waypoints src/Excavator/subtask_excavator_follow_waypoints.cpp)
add_executable(subtask_excavator_follow_waypoints_deg src/Excavator/subtask_excavator_follow_waypoints_deg.cpp)
add_executable(subtask_excavator_navigate_anywhere src/Excavator/subtask_excavator_navigate_anywhere.cpp)
add_executable(subtask_excavator_navigate_anywhere_deg src/Excavator/subtask_excavator_navigate_anywhere_deg.cpp)
add_executable(subtask_excavator_navigate_through_poses src/Excavator/subtask_excavator_navigate_through_poses.cpp)
add_executable(subtask_excavator_navigate_through_poses_deg src/Excavator/subtask_excavator_navigate_through_poses_deg.cpp)


set(TARGETS
  subtask_bulldozer_follow_waypoints_deg
  subtask_bulldozer_follow_waypoints
  subtask_bulldozer_navigate_anywhere_deg
  subtask_bulldozer_navigate_anywhere
  subtask_bulldozer_navigate_through_poses_deg
  subtask_bulldozer_navigate_through_poses
  subtask_crawlerdump_follow_waypoints_deg
  subtask_crawlerdump_follow_waypoints
  subtask_crawlerdump_navigate_anywhere_deg
  subtask_crawlerdump_navigate_anywhere
  subtask_crawlerdump_navigate_through_poses_deg
  subtask_crawlerdump_navigate_through_poses
  subtask_crawlerdump_release_soil
  subtask_crawlerdump_swing
  subtask_crawlerdump_swing_align_to_heading
  subtask_excavator_change_pose
  subtask_excavator_level
  subtask_excavator_change_pose_plan
  subtask_excavator_excavate_simple_plan
  subtask_excavator_excavate_simple
  subtask_excavator_release_simple
  subtask_excavator_follow_waypoints_deg
  subtask_excavator_follow_waypoints
  subtask_excavator_navigate_anywhere_deg
  subtask_excavator_navigate_anywhere
  subtask_excavator_navigate_through_poses_deg
  subtask_excavator_navigate_through_poses
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
  sensor_msgs
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

ament_export_dependencies(${dependencies_cpp})
ament_export_libraries(${PROJECT_NAME} mongocxx bsoncxx)
ament_export_targets(${PROJECT_NAME})  
ament_export_include_directories(include)
ament_package()
