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