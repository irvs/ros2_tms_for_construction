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

set(srv_files
  "srv/SpeakerSrv.srv"
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
