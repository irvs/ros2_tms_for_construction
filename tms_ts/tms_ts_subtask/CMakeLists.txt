# cmake_minimum_required(VERSION 3.8)
# project(tms_ts_subtask)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_action REQUIRED)
# find_package(tms_msg_ts REQUIRED)
# find_package(tms_msg_rp REQUIRED)
# find_package(moveit_ros_planning_interface REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(RapidJSON REQUIRED)
# find_package(mongocxx REQUIRED)
# find_package(bsoncxx REQUIRED)
# find_package(glog REQUIRED)

# include_directories(include)
# include_directories(${LIBMONGOCXX_INCLUDE_DIRS})
# include_directories(${LIBBSONCXX_INCLUDE_DIRS})

# # Subtask node base library
# add_library(subtask_node_base
#   src/subtask_node_base.cpp
# )

# target_include_directories(subtask_node_base PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#   /usr/local/include/mongocxx/v_noabi
#   /usr/local/include/bsoncxx/v_noabi
# )

# target_link_directories(subtask_node_base PUBLIC
#   /usr/local/lib
# )

# target_link_libraries(subtask_node_base
#   mongocxx
#   bsoncxx
# )

# ament_target_dependencies(subtask_node_base
#   rclcpp
#   std_msgs
# )

# # Pose converter library
# add_library(excavator_pose_converter
#   src/Excavator/lib/excavator_pose_converter.cpp
# )

# target_include_directories(excavator_pose_converter PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# # Subtask excavation retract arm
# # add_executable(subtask_excavation_retract_arm
# #   # src/Excavator/subtask_excavation_retract_arm.cpp
# # )

# ament_target_dependencies(subtask_excavation_retract_arm
#   rclcpp
#   rclcpp_action
#   tms_msg_ts
#   tms_msg_rp
#   moveit_ros_planning_interface
#   geometry_msgs
# )

# target_include_directories(subtask_excavation_retract_arm PRIVATE
#   ${RapidJSON_INCLUDE_DIRS}
# )

# target_link_libraries(subtask_excavation_retract_arm
#   excavator_pose_converter
#   subtask_node_base
#   glog::glog
# )

# install(TARGETS
#   subtask_excavation_retract_arm
#   excavator_pose_converter
#   subtask_node_base
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(DIRECTORY include/
#   DESTINATION include
# )

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()