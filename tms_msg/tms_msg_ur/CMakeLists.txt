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

# メッセージとサービスのファイルリストを明確に定義
set(msg_files
  "msg/Demo202412.msg"
  "msg/TmsdbPoseWriteMsg.msg"
)

set(srv_files
  "srv/SpeakerSrv.srv"
  "srv/TmsdbTerrainDBPoseWriteSrv.srv"
)

# インターフェースの生成
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs
)

ament_package()
