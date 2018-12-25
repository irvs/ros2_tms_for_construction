cmake_minimum_required(VERSION 3.5)
project(tms_ss_vicon)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED) 
find_package(rclcpp REQUIRED)
find_package(tms_msg_ss REQUIRED) 
find_package(tms_msg_db REQUIRED)
find_package(visualization_msgs REQUIRED)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_BINARY_DIR}
)
#ament_export_include_directories(include)
ament_export_include_directories(${PROJECT_SOURCE_DIR}/viconSDK)

#link_directories(${LINK_DIRECTORIES} ${PROJECT_SOURCE_DIR}/viconSDK)

add_executable(vicon_stream src/vicon_stream.cpp)
#add_executable(vicon_stream src/vicon_stream.cpp)


ament_target_dependencies(vicon_stream
    "rclcpp"
    "rcutils"
    "std_msgs"

)

ament_target_dependencies(vicon_stream tms_msg_ss)
ament_target_dependencies(vicon_stream tms_msg_db)


ament_export_dependencies(ament_cmake)

ament_export_dependencies(vicon_stream)
ament_export_dependencies(tms_msg_ss_generate_messages_cpp)
ament_export_dependencies(tms_msg_db_generate_messages_cpp)

ament_export_include_directories(include)

#add_dependencies(vicon_stream tms_msg_ss_generate_messages_cpp tms_msg_db_generate_messages_cpp)


target_link_libraries(vicon_stream ViconDataStreamSDK_CPP)
#target_link_libraries(test_executor ${PROJECT_NAME})


ament_package(
  #INCLUDE_DIRS include
  #LIBRARIES vicon_stream
  #AMENT_DEPENDS rclcpp
  #DEPENDS system_lib
)
