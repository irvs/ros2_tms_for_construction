###############################
## cmake and project setting ##
###############################
cmake_minimum_required(VERSION 3.5)
project(tms_msg_rc)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()

################################################
## Declare ROS messages, services and actions ##
################################################
rosidl_generate_interfaces(tms_msg_rc
  "srv/TurtlebotControl.srv"



 #add_message_files(
  #FILES
  #vicon_data.msg
  #accumulate_data.msg
  # fss_class_data.msg
  # fss_cluster_data.msg
  # fss_detected_cluster_data.msg
  # fss_object_data.msg
  # fss_observed_datas.msg
  # fss_person_trajectory_data.msg
  # fss_pre_data.msg
  # fss_tf_data.msg
  # fss_tf_datas.msg
  #bas_behavior_data.msg
  # ics_object_data.msg
  # iss_object_data.msg
  #lrf_analyze_data.msg
  #lrf_cluster.msg
  #lrf_clusters_data.msg
  #lrf_datas.msg
  #lrf_human_data.msg
  #lrf_trajectory_data.msg
  #ods_person_dt.msg
  #Robot.msg
  #Furniture.msg
  #tracking_points.msg
  #tracking_grid.msg
  #pot_tracking_points.msg
  #pot_tracking_grid.msg
  #pot_tracking_psen.msg
  #pot_tracking_psens.msg
  #pot_manager.msg
  #pot_clustering.msg
  #tracking_cluster.msg
  #tracking_clusters_data.msg

  #Sensor.msg

  #Skeleton.msg
  #SkeletonArray.msg
  #CameraPosture.msg
  #SkeletonStreamWrapper.msg
  #Beacon.msg

  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs

)

#add_service_files(
#  FILES
  #ods_cylinder.srv
  #ods_face_detection.srv
  #ods_change_detection.srv
  #ods_furniture.srv
  #ods_pcd.srv
  #ods_get_robots_pos.srv
  #ods_skincolor_extraction.srv
  #ods_wagon.srv
  #ods_wagon_object.srv
  #ods_person_detection.srv
#)

#generate_messages(

#)

###################################
## catkin specific configuration ##
###################################
ament_package(
  #CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs
)

###########
## Build ##
###########
# none
