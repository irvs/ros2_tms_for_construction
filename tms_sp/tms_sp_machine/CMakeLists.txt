cmake_minimum_required(VERSION 3.8)
project(tms_sp_machine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tms_msg_db REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# C++ nodes
add_executable(tms_sp_machine_points src/tms_sp_machine_points.cpp)
ament_target_dependencies(
  tms_sp_machine_points
  geometry_msgs
  sensor_msgs
  rclcpp
  tf2
  tf2_ros
  tf2_sensor_msgs
  # pcl_ros
  # tf2_eigen
)
install(TARGETS
  tms_sp_machine_points
  DESTINATION lib/${PROJECT_NAME}
)

# Python nodes
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/tms_sp_machine_odom.py
  scripts/tms_sp_machine_odom_yammer.py
  scripts/tms_sp_machine_posest.py
  scripts/tms_sp_machine_joints.py
  scripts/tms_sp_pose_param.py
  scripts/tms_sp_tf_pose_param.py
  DESTINATION lib/${PROJECT_NAME}
)

# install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
