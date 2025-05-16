## Setup Environmental Variables
export WS_SRC=~/ros2-tms-for-construction_ws/src

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
sudo apt install -y qtbase5-dev libqt5svg5-dev libzmq3-dev libdw-dev
git clone https://github.com/BehaviorTree/Groot.git "$WS_SRC/Groot"
cd ~/ros2-tms-for-construction_ws && colcon build --packages-select groot
```

### Install nlohmann-json library
```
sudo apt install nlohmann-json3-dev
```

### Setup OPERA
```
mkdir -p \
  "$WS_SRC/opera/common" \
  "$WS_SRC/opera/simulator" \
  "$WS_SRC/opera/zx200" \
  "$WS_SRC/opera/ic120"

# Install tms_if_for_opera
git clone https://github.com/irvs/tms_if_for_opera.git "$WS_SRC/tms_if_for_opera"

# Install common packages for OPERA
git clone https://github.com/pwri-opera/com3_ros.git "$WS_SRC/opera/common/com3_ros"

# Install the package for OperaSim-PhysX
git clone -b main-ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git \
  "$WS_SRC/opera/simulator/ROS-TCP-Endpoint"

# Install packages for OPERA-compatible backhoe ZX200
git clone https://github.com/pwri-opera/zx200_ros2.git \
  "$WS_SRC/opera/zx200/zx200_ros2"

# Install packages for OPERA-compatible crawler dump IC120
git clone https://github.com/pwri-opera/ic120_ros2.git "$WS_SRC/opera/ic120/ic120_ros2"
# git clone https://github.com/pwri-opera/gnss_localizer_ros2.git 
# git clone https://github.com/pwri-opera/ic120_com3_ros.git 

```


### Setup MoveIt!, Nav2 and depended packages
```
# install MoveIt!
sudo apt -y install ros-humble-*moveit*
# install Nav2
sudo apt -y install ros-humble-*nav2*
# install robot_localization package
sudo apt -y install ros-humble-robot-localization 
#install tf package
sudo apt -y install ros-humble-*tf*
```

### Build the workspace

```
cd ~/ros2-tms-for-construction_ws
colcon build --packages-select com3_msgs && source install/setup.bash
colcon build && source install/setup.bash
```