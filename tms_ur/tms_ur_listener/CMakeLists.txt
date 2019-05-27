cmake_minimum_required(VERSION 3.5)
project(tms_ur_listener)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(tms_ur_listener
  "msg/JuliusMsg.msg"
  "srv/GSpeechSrv.srv"
  DEPENDENCIES builtin_interfaces
)

ament_package()