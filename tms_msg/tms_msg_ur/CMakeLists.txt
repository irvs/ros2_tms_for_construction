cmake_minimum_required(VERSION 3.5)
project(tms_msg_ur)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


# 両方のサービスファイルを一度に渡す
rosidl_generate_interfaces(tms_msg_ur
  "srv/SpeakerSrv.srv"
  "srv/TmsdbTerrainDBPoseWriteSrv.srv"
  "msg/TmsdbPoseWriteMsg.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs
  DEPENDENCIES builtin_interfaces sensor_msgs
)

set(msg_files
  "msg/Demo202412.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  ${msg_files}
  DEPENDENCIES builtin_interfaces 
)


ament_package()
