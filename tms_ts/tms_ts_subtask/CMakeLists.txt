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
find_package(glog REQUIRED)




set(dependencies_cpp
  rclcpp
  rclcpp_action
  std_msgs
  ament_cmake
  behaviortree_cpp_v3
  nlohmann_json
  geometry_msgs
  tms_msg_ts)


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

ament_target_dependencies(${PROJECT_NAME} ${dependencies_cpp})

add_executable(zx120_sample_boom_subtask src/crane/zx120/sample_boom_subtask.cpp)
add_executable(zx120_sample_swing_subtask src/crane/zx120/sample_swing_subtask.cpp)
add_executable(zx120_sample_arm_subtask src/crane/zx120/sample_arm_subtask.cpp)
add_executable(zx120_sample_bucket_subtask src/crane/zx120/sample_bucket_subtask.cpp)
add_executable(zx200_sample_boom_subtask src/crane/zx200/sample_boom_subtask.cpp)
add_executable(zx200_sample_swing_subtask src/crane/zx200/sample_swing_subtask.cpp)
add_executable(zx200_sample_arm_subtask src/crane/zx200/sample_arm_subtask.cpp)
add_executable(zx200_sample_bucket_subtask src/crane/zx200/sample_bucket_subtask.cpp)

target_include_directories(zx120_sample_boom_subtask PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/local/include/mongocxx/v_noabi
  /usr/local/include/bsoncxx/v_noabi)

target_include_directories(zx120_sample_swing_subtask PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/local/include/mongocxx/v_noabi
  /usr/local/include/bsoncxx/v_noabi)

target_include_directories(zx120_sample_arm_subtask PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/local/include/mongocxx/v_noabi
  /usr/local/include/bsoncxx/v_noabi)

target_include_directories(zx120_sample_bucket_subtask PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/local/include/mongocxx/v_noabi
  /usr/local/include/bsoncxx/v_noabi)

  target_include_directories(zx200_sample_boom_subtask PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/local/include/mongocxx/v_noabi
  /usr/local/include/bsoncxx/v_noabi)

target_include_directories(zx200_sample_swing_subtask PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/local/include/mongocxx/v_noabi
  /usr/local/include/bsoncxx/v_noabi)

target_include_directories(zx200_sample_arm_subtask PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/local/include/mongocxx/v_noabi
  /usr/local/include/bsoncxx/v_noabi)

target_include_directories(zx200_sample_bucket_subtask PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/local/include/mongocxx/v_noabi
  /usr/local/include/bsoncxx/v_noabi)


target_link_directories(zx120_sample_boom_subtask PUBLIC
  /usr/local/lib)

target_link_directories(zx120_sample_swing_subtask PUBLIC
/usr/local/lib)

target_link_directories(zx120_sample_arm_subtask PUBLIC
/usr/local/lib)

target_link_directories(zx120_sample_bucket_subtask PUBLIC
/usr/local/lib)

target_link_directories(zx200_sample_boom_subtask PUBLIC
  /usr/local/lib)

target_link_directories(zx200_sample_swing_subtask PUBLIC
/usr/local/lib)

target_link_directories(zx200_sample_arm_subtask PUBLIC
/usr/local/lib)

target_link_directories(zx200_sample_bucket_subtask PUBLIC
/usr/local/lib)

target_link_libraries(zx120_sample_boom_subtask
  ${PROJECT_NAME}
  mongocxx
  bsoncxx
  glog::glog
)

target_link_libraries(zx120_sample_swing_subtask
  ${PROJECT_NAME}
  mongocxx
  bsoncxx
  glog::glog
)

target_link_libraries(zx120_sample_arm_subtask
  ${PROJECT_NAME}
  mongocxx
  bsoncxx
  glog::glog
)

target_link_libraries(zx120_sample_bucket_subtask
  ${PROJECT_NAME}
  mongocxx
  bsoncxx
  glog::glog
)

target_link_libraries(zx200_sample_boom_subtask
  ${PROJECT_NAME}
  mongocxx
  bsoncxx
  glog::glog
)

target_link_libraries(zx200_sample_swing_subtask
  ${PROJECT_NAME}
  mongocxx
  bsoncxx
  glog::glog
)

target_link_libraries(zx200_sample_arm_subtask
  ${PROJECT_NAME}
  mongocxx
  bsoncxx
  glog::glog
)

target_link_libraries(zx200_sample_bucket_subtask
  ${PROJECT_NAME}
  mongocxx
  bsoncxx
  glog::glog
)

ament_target_dependencies(zx120_sample_boom_subtask ${dependencies_cpp})
ament_target_dependencies(zx120_sample_swing_subtask ${dependencies_cpp})
ament_target_dependencies(zx120_sample_arm_subtask ${dependencies_cpp})
ament_target_dependencies(zx120_sample_bucket_subtask ${dependencies_cpp})
ament_target_dependencies(zx200_sample_boom_subtask ${dependencies_cpp})
ament_target_dependencies(zx200_sample_swing_subtask ${dependencies_cpp})
ament_target_dependencies(zx200_sample_arm_subtask ${dependencies_cpp})
ament_target_dependencies(zx200_sample_bucket_subtask ${dependencies_cpp})

install(TARGETS
  zx120_sample_boom_subtask
  zx120_sample_swing_subtask
  zx120_sample_arm_subtask
  zx120_sample_bucket_subtask
  zx200_sample_boom_subtask
  zx200_sample_swing_subtask
  zx200_sample_arm_subtask
  zx200_sample_bucket_subtask
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
  # find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()


ament_export_dependencies(${dependencies_cpp})
ament_export_libraries(${PROJECT_NAME} mongocxx bsoncxx)
ament_export_targets(${PROJECT_NAME})  
ament_export_include_directories(include)
ament_package()
