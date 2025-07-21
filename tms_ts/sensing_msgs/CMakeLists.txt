cmake_minimum_required(VERSION 3.8)
project(sensing_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  
  "msg/FollowWaypoints.msg"
  "msg/FollowWaypointsDeg.msg"
  "msg/NavigateAnywhere.msg"
  "msg/NavigateAnywhereDeg.msg"
  "msg/NavigateThroughPoses.msg"
  "msg/NavigateThroughPosesDeg.msg"
  "msg/ReleaseSoil.msg"
  "msg/ExcavateSimple.msg"
  "msg/ReleaseSimple.msg"
  "msg/ChangePose.msg"
  "msg/Sample.msg"
  "msg/Zx200EndEffector.msg"
  "msg/Zx200CollisionObjectsIc120.msg"
  "msg/Flgs.msg"
  "msg/Flgs202506.msg"
  DEPENDENCIES builtin_interfaces
)

ament_package()

