# ROS2-TMS-FOR-CONSTRUCTION

ROS2-TMS-FOR-CONSTRUCTION is an IoRT library for construction applications based on ROS2-TMS.

https://github.com/irvs/ros2_tms_for_construction/assets/63947554/d7fb02dd-37d9-4d72-aa2c-a2c2d6f7824a

## ROS2-TMS

ROS2-TMS is an IoRT (Internet of Robotic Things) library for TMS (Town Management System), which is the management system of an informationally structured environment (ISE). ROS2-TMS is newly constructed on the basis of [ROS-TMS](https://github.com/irvs/ros_tms/wiki) and adopts the state-of-the-art robot middleware, ROS2. This system integrates various information gathered by distributed sensors, stores them to an on-line database, plans proper service tasks, and manages and executes robot motion.

Wiki page : [https://github.com/irvs/ros2_tms/wiki/ROS2-TMS](https://github.com/irvs/ros2_tms/wiki/ROS2-TMS)

ROS2-TMS is being developed as a part of "**MyIoT Store**" in "**MyIoT Project**" supported by the Cabinet Office (CAO), **Cross-ministerial Strategic Innovation Promotion Program (SIP)**, “An intelligent knowledge processing infrastructure, integrating physical and virtual domains” (funding agency: NEDO).

## ROS2-TMS-FOR-CONSTRUCTION

ROS2-TMS-FOR-CONSTRUCTION is developed as an IoRT library for construction applications based on ROS2-TMS with the support of JST Moonshot R&D, Grant Number JPMJPS2032 entitled “Collaborative AI robots for adaptation of diverse environments and innovation of infrastructure construction” in “Moonshot Goal 3: Realization of Artificial Intelligence (AI) robots that autonomously learn, adapt to their environment, evolve itself in intelligence, and act alongside human beings, by 2050.”

Project page: [https://moonshot-cafe-project.org/en/](https://moonshot-cafe-project.org/en/)

### Architecture

![](docs/ros2_tms_for_construction_architecture.png)

## Install

### ROS2 Humble

https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html

### Git

https://git-scm.com/book/en/v2/Getting-Started-Installing-Git

### MongoDB

https://www.mongodb.com/docs/v6.0/tutorial/install-mongodb-on-ubuntu/

### MongoDB Compass

https://www.mongodb.com/docs/compass/current/install/


### Related packages for ROS2-TMS-FOR-CONSTRUCTION
- pymongo 4.3.3
- open3d 0.16.0
- numpy 1.22.3
- catkin-pkg 0.5.2
- empy 3.3.4
- lark 1.1.3
- setuptools 58.2.0
- numpy-quaternion 2022.4.3


## Setup

### Create a workspace

```
cd
mkdir -p ~/ros2-tms-for-construction_ws/src
```

### Clone this repository

```
cd ~/ros2-tms-for-construction_ws/src
git clone https://github.com/irvs/ros2_tms_for_construction.git -b develop/ts # modify if this is the other branch
```


### Install required python packages
```
cd ~/ros2-tms-for-construction_ws/src/ros2_tms_for_construction
python3 -m pip install -r requirements.txt
```

### Setup MongoDB

> **Note**
> If a rostmsdb database already exists on mongodb, an error will occur during the execution of the following command. In such a case, delete the rostmsdb database in your environment and then execute the following command.


```
sudo systemctl start mongod
cd ~/ros2-tms-for-construction_ws/src/ros2_tms_for_construction/demo
unzip rostmsdb_collections.zip
mongorestore dump
```

## Setup BehaviorTree.CPP

```
sudo apt install libzmq3-dev libboost-dev libncurses5-dev libncursesw5-dev
cd
git clone --branch v3.8 https://github.com/BehaviorTree/BehaviorTree.CPP.git
cd BehaviorTree.CPP
mkdir build; cd build
cmake ..
make
sudo make install
cd && rm -rf BehaviorTree.CPP
```

### Setup mongocxx / bsoncxx

```
cd
wget https://github.com/mongodb/mongo-c-driver/releases/download/1.24.4/mongo-c-driver-1.24.4.tar.gz
tar -xzf mongo-c-driver-1.24.4.tar.gz
cd mongo-c-driver-1.24.4
mkdir cmake-build
cd cmake-build
cmake -DENABLE_AUTOMATIC_INIT_AND_CLEANUP=OFF .. -DCMAKE_INSTALL_PREFIX=/usr/local
sudo make install

cd
curl -OL https://github.com/mongodb/mongo-cxx-driver/releases/download/r3.8.1/mongo-cxx-driver-r3.8.1.tar.gz
tar -xzf mongo-cxx-driver-r3.8.1.tar.gz
cd mongo-cxx-driver-r3.8.1/build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DBSONCXX_POLY_USE_BOOST=1 -DMONGOCXX_OVERRIDE_DEFAULT_INSTALL_PREFIX=OFF
cmake --build .
sudo cmake --build . --target install

export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
cd && rm -rf mongo-c-driver-1.24.4 mongo-c-driver-1.24.4.tar.gz mongo-cxx-driver-r3.8.1 mongo-cxx-driver-r3.8.1.tar.gz
```

### Setup Groot
```
sudo apt install qtbase5-dev libqt5svg5-dev libzmq3-dev libdw-dev
cd ~/ros2-tms-for-construction_ws/src && git clone https://github.com/BehaviorTree/Groot.git
cd .. && colcon build --packages-select groot
./build/groot/Groot
```

### Install nlohmann-json library
```
sudo apt install nlohmann-json3-dev
```

### Setup OPERA
```
# Install dbcppp
cd && git clone --recurse-submodules https://github.com/genkiiii/dbcppp.git
cd dbcppp && mkdir build && cd build

# Install caranry
cd && git clone https://github.com/djarek/canary.git
cd canary && mkdir build && cd build
cmake ..
sudo make install

# Install rttr
sudo apt install doxygen
https://github.com/irvs/rttr.git # This package is a private repository. Please wait a while until it is made public.
cd rttr && mkdir build && cd build
cmake ..
sudo make install
echo 'export RTTR_DIR=/home/common/rttr/build/install/' >> ~/.bashrc
source ~/.bashrc

# Install tms_if_for_opera
cd ~/ros2-tms-for-construction_ws/src
git clone -b develop/top https://github.com/irvs/tms_if_for_opera.git

# Install common packages for OPERA
mkdir -p opera/common && cd opera/common
git clone https://github.com/pwri-opera/com3
git clone https://github.com/pwri-opera/com3_ros.git

# Install the package for OperaSim-PhysX
cd .. && mkdir simulator && cd simulator
git clone -b main-ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git

# Install packages for OPERA-compatible backhoe ZX200
cd .. && mkdir zx200 && cd zx200
git clone https://github.com/pwri-opera/zx200_ros2.git

# Install packages for OPERA-compatible crawler dump IC120
cd .. && mkdir ic120 && cd ic120
git clone https://github.com/pwri-opera/ic120_ros2.git  # This package is a private repository. Please wait a while until it is made public.
git clone https://github.com/pwri-opera/gnss_localizer_ros2.git   # This package is a private repository. Please wait a while until it is made public.
git clone https://github.com/pwri-opera/ic120_com3_ros.git  # This package is a private repository. Please wait a while until it is made public.
```

### Setup MoveIt! and Nav2
```
# install MoveIt!
sudo apt install ros-humble-*moveit*
# install Nav2
sudo apt install ros-humble-*nav2*
```

### Build the workspace

```
cd ~/ros2-tms-for-construction_ws
colcon build && source install/setup.bash
```


## File Structure

ROS2-TMS-FOR-CONSTRUCTION is composed of the following packages. Each packages's detail was described as follows.

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

The following demonstrations are presented here.
Before demonstration, you need to setup each terminals before running the commands described in the following demonstrations.

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