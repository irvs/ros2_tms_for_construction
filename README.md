# ROS2-TMS-FOR-CONSTRUCTION

ROS2-TMS-FOR-CONSTRUCTION is an IoRT (Internet of Robots and Things) library for construction applications developed based on ROS2-TMS.

https://github.com/irvs/ros2_tms_for_construction/assets/63947554/d7fb02dd-37d9-4d72-aa2c-a2c2d6f7824a

## ROS2-TMS

ROS2-TMS is an IoRT (Internet of Robotic Things) library for TMS (Town Management System), which is the management system of an informationally structured environment (ISE). ROS2-TMS is designed on the basis of [ROS-TMS](https://github.com/irvs/ros_tms/wiki) and adopts the state-of-the-art robot middleware, ROS2. This system integrates various information collected by distributed sensors, stores it in an on-line database, plans appropriate service tasks, and manages and executes robot's motion.

Wiki page : [https://github.com/irvs/ros2_tms/wiki/ROS2-TMS](https://github.com/irvs/ros2_tms/wiki/ROS2-TMS)

ROS2-TMS was developed as a part of "**MyIoT Store**" in "**MyIoT Project**" supported by the Cabinet Office (CAO), **Cross-ministerial Strategic Innovation Promotion Program (SIP)**, “An intelligent knowledge processing infrastructure, integrating physical and virtual domains” (funding agency: NEDO).

## ROS2-TMS-FOR-CONSTRUCTION

ROS2-TMS-FOR-CONSTRUCTION is newly developed as an IoRT library for construction applications based on ROS2-TMS with the support of JST Moonshot R&D, Grant Number JPMJPS2032 entitled “Collaborative AI robots for adaptation of diverse environments and innovation of infrastructure construction” in “Moonshot Goal 3: Realization of Artificial Intelligence (AI) robots that autonomously learn, adapt to their environment, evolve itself in intelligence, and act alongside human beings, by 2050.”

Project page: [https://moonshot-cafe-project.org/en/](https://moonshot-cafe-project.org/en/)

### Architecture

![](docs/ros2_tms_for_construction_architecture.png)

## Setup
There are two ways to set up ROS2-TMS for Construction: by entering individual commands directly or by using scripts.

1. [Etering individual commands directly](setup.md)
2. Using scripts. 

    ```
    cd
    mkdir -p ~/ros2-tms-for-construction_ws/src && cd ~/ros2-tms-for-construction_ws/src
    git clone https://github.com/irvs/ros2_tms_for_construction.git
    cd ros2_tms_for_construction
    . setup.sh
    ```

## File Structure

ROS2-TMS-FOR-CONSTRUCTION consists of the following packages. The details of each package are as follows:

### tms_db

-  [tms_db_manager](tms_db/tms_db_manager)

   ROS2-TMS database manager. This package has tms_db_reader(_gridfs) and tms_db_writer(_gridfs) nodes.

### tms_sd

- [tms_sd_ground](tms_sd/tms_sd_ground)

  tms_sd_ground is a package for formatting OccupancyGrid msg to Tmsdb msg and sending it to tms_db_writer.

  The received OccupancyGrid msg is a heatmap showing the hardness of the ground.

- [tms_sd_terrain](tms_sd/tms_sd_terrain)

  tms_sd_terrain is a package for handling point cloud data of static and dynamic terrain.

  The received PointCloud2 msg is a point cloud data of terrain.

  Static terrain refers to terrain that does not change during construction operations. And dynamic terrain refers to terrain that changes during construction work.

- [tms_sd_theta](tms_sd/tms_sd_theta/)

  tms_sd_theta is a package for formatting CompressedImage msg to Tmsdb msg and sending it to tms_db_writer.

  The received CompressedImage msg is the jpeg image taken by a 360-degree camera.  

### tms_sp

- [tms_sp_machine](tms_sp/tms_sp_machine/)

  tms_sp_machine is a package for handling data realated to construction machines.


- [tms_sp_sensing](tms_sp/tms_sp_sensing/)

  tms_sp_sensing is a package  to update dynamic parameters stored in mongoDB based on ros topic sent from another pc.

### tms_ss

- [tms_ss_terrain_static](tms_ss/tms_ss_terrain_static)

  tms_ss_terrain_static is a package for handling point cloud data of static terrain.

  The received PointCloud2 msg is a point cloud data of terrain.

### tms_tf

- [tms_tf_gui](tms_tf/tms_tf_gui/)

  tms_tf_gui is a package for transforming construction data (ex. machine's location, terrain and hardness of ground) using GUI tools.

### tms_ts
- [tms_ts_launch](tms_ts/tms_ts_launch/)
  
  tms_ts_launch_ts is a  package for specifying and executing tasks to run the actual construction machinery.
  

- [tms_ts_subtask](tms_ts/tms_ts_subtask/)

  tms_ts_subtask is a package included in subtasks.
  
  If you want to implement new subtasks, please refer programs in the tms_ts_subtask directory.

- [tms_ts_manager](tms_ts/tms_ts_manager/)

  tms_ts_manager is a package that contains the main program of task schedular.

- [sensing_msgs](tms_ts/sensing_msgs/)
  
  sensing_msgs is a package that contains .msg files for sensing process.

### tms_ur

- [tms_ur_button_input](tms_ur/tms_ur_button_input)

  tms_ur_button_input is a package that searches data in mongodb for the corresponding task when the GUI button is pressed, and passes the corresponding task sequence to the task scheduler.

## Demo

Here are some demonstrations. Before starting the demonstrations, each terminal must be set up with the following commands.

```
cd ~/ros2-tms-for-construction_ws
source install/setup.bash
```

1. [Store data](CHAPTER1.md)
2. [Get stored data](CHAPTER2.md)
3. [Store and get data simultaneously in real-time](CHAPTER3.md)
4. [Try running the task schedular](CHAPTER4.md)
5. [Try running the task schedular with OperaSim-PhysX](CHAPTER5.md)
6. [Insert new task data to tms_db](CHAPTER6.md)
7. [How to update parameters in mongodb based on topics from sensing pc](CHAPTER7.md)
<!-- 8. [Try running the actual OPERA-compatible construction machinery](#8-Try-running-the-actual-opera-compatible-construction-machinery) -->




<!-- ### 8. Try running the actual OPERA-compatible construction machinery

※ Execution of this section requires a actual OPERA-compatible construction machinery.

First, in accordance with the instructions outlined in section 4, specify the task_id of the task data you wish to execute and run the following command to launch the task management mechanism of ROS2-TMS for Construction.

#### For operating actual OPERA-compatible construction machinery ZX200

< LOCAL PC > 

```
# Open the 1st terminal
cd ~/ros2-tms-for-construction_ws && source install/setup.bash
ros2 launch tms_if_for_opera tms_if_for_opera.launch.py

# Open the 2nd terminal
cd ~/ros2-tms-for-construction_ws && source install/setup.bash
ros2 launch tms_if_for_opera tms_if_for_opera.launch.py

# Open the 2nd terminal
cd ~/ros2-tms-for-construction_ws && source install/setup.bash
ros2 launch tms_ts_launch tms_ts_construction.launch.py
```

< REMOTE PC >

```
cd ~/ros2-tms-for-construction_ws && source install/setup.bash
ros2 launch zx200_bringup vehicle.launch.py
```

#### For operating the actual OPOERA-compatible construction machinery IC120

< LOCAL PC >
```
# Open the 1st terminal
cd ~/ros2-tms-for-construction_ws && source install/setup.bash
ros2 launch ic120_bringup ic120_remote.launch.py

# Open the 3rd terminal
cd ~/ros2-tms-for-construction_ws && source install/setup.bash
ros2 launch tms_ts_launch tms_ts_construction.launch.py
```

< REMOTE PC >
```
cd ~/ros2-tms-for-construction_ws && source install/setup.bash
ros2 launch ic120_bringup ic120_vehicle.launch.py
``` -->



## Version Information

* Date : 2022.8.19 (since 2022.8.19 ROS2-TMS-FOR-CONSTRUCTION / since 2019.2.14 ROS2-TMS / since 2012.5.1 ROS-TMS / since 2005.11.1 TMS)  

* Ubuntu 22.04 LTS 64BIT  

* ROS2 Humble Hawksbill : https://docs.ros.org/en/humble/Installation.html   

* mongodb 6.0  

* pymongo 4.3.3

* open3d 0.16.0

* numpy 1.22.3

* catkin-pkg 0.5.2

* empy 3.3.4

* lark 1.1.3

* setuptools 58.2.0

* numpy-quaternion 2022.4.3
