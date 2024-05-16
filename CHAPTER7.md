### 7. How to update parameters in mongodb based on topics from sensing pc

> **Note**
> The pc running cps and the sensing pc must be on the same network and they must be aligned if ROS_DOMAIN_ID is set.
> For more information for ROS_DOMAIN_ID, please refer to the official ROS documentation ( https://docs.ros.org/en/humble/Concepts/Intermediate/About-Domain-ID.html ).

1. Place the .msg file directly under the ros2-tms-for-construction_ws/tms_ts/sensing_msgs/msg directory. This .msg file represents the type of data to be stored from the sensing pc to the parameter collection in mongodb via ros2 topic.
2. After storing the .msg file in place, execute the following command.
  ```
  cd ~/ros2-tms-for-construction_ws
  colcon build --packages-select sensing_msgs tms_sp_sensing && source install/setup.bash
  ros2 run tms_sp_sensing sample
  ```
3. From then on, processing will be performed on a different ubuntu pc for sensing processing than the pc running ros2-tms-for-construction. These personal computers must be located on the same network. In this description, the sensing pc is assumed to be running Ubuntu 22.04 lts with ROS2 Humble on it.
4. Once the sensing pc is ready, open a terminal and execute the following command.
  ```
  cd 
  mkdir -p sensing_ws/src
  cd sensing_ws/src
  git clone -b master https://github.com/kasahara-san/sensing_sample_cps.git
  cd ..
  colcon build --packages-select sample_sensing_nodes sensing_msgs && source install/setup.bash
  ros2 run sample_sensing_nodes sample_publisher
  ```
5. Then you can see that the following parameter on parameter collection in mongodb change dynamically using mongodb compass. Note that when using mongodb compass to check parameter values, you must press the refresh button shown in the following image each time to reflect the latest values of the parameters on mongodb.

![](docs/dynamic_parameter.png)