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

find_library(VICON_SDK_LIBRARY NAMES ViconDataStreamSDK_CPP PATHS /home/song/ros_tms/tms_ss/tms_ss_vicon/viconSDK)

set(EXECUTABLE_OUTPUT_PATH bin/${CMAKE_BUILD_TYPE}) 


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_BINARY_DIR}
  
)
#ament_export_include_directories(include)
ament_export_include_directories(${PROJECT_SOURCE_DIR}/viconSDK)

link_directories(${LINK_DIRECTORIES} ${PROJECT_SOURCE_DIR}/viconSDK)
link_directories(viconSDK/; /usr/local/lib/; /usr/lib/)
#add_library(ViconDataStreamSDK_CPP  src/vicon_streamtest.cpp)





# Including ViconDataStreamSDK_CPP directory
include_directories(
  viconSDK/ViconDataStreamSDK_CPP
)

# Linking directory 
link_directories(
  viconSDK/ViconDataStreamSDK_CPP
)

# Declare an executable
add_executable(vicon_stream src/vicon_stream.cpp)


# Linking libraries to executables
target_link_libraries(vicon_stream ViconDataStreamSDK_CPP)
#target_link_libraries(vicon_stream ${VICON_SDK_LIBRARY})

#target_link_libraries(test_executor ${PROJECT_NAME})
#ament_target_dependencies(vicon_stream ViconDataStreamSDK_CPP)

ament_target_dependencies(vicon_stream
    "rclcpp"
    "rcutils"
    "std_msgs"
    "visualization_msgs"

)

ament_target_dependencies(vicon_stream tms_msg_ss)
ament_target_dependencies(vicon_stream tms_msg_db)


ament_export_dependencies(ament_cmake)

ament_export_dependencies(vicon_stream)
ament_export_dependencies(tms_msg_ss_generate_messages_cpp)
ament_export_dependencies(tms_msg_db_generate_messages_cpp)

ament_export_include_directories(include)

#add_dependencies(vicon_stream tms_msg_ss_generate_messages_cpp tms_msg_db_generate_messages_cpp)

target_include_directories(vicon_stream PUBLIC
      ${tms_ss_vicon_INCLUDE_DIRS}
      ${rclcpp_INCLUDE_DIRS}
    )




install(TARGETS
  vicon_stream
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS vicon_stream
  DESTINATION lib/${PROJECT_NAME}
)

ament_package(
  #INCLUDE_DIRS include
  #LIBRARIES vicon_stream
  #AMENT_DEPENDS rclcpp
  #DEPENDS system_lib
)
