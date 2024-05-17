# This file is used to setup the environment for ROS2-TMS for Construction.

# Copyright 2023, IRVS Laboratory, Kyushu University, Japan.
# //  Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#       http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
##################################################################################################

# Setup ROS2
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

sudo apt install -y software-properties-common
#
yes | sudo add-apt-repository universe
#
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt -y upgrade
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-humble-ros-base
sudo apt install -y ros-dev-tools

sudo apt install pip
sudo pip install pymongo

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc


#MongoDB
sudo apt-get install gnupg curl
curl -fsSL https://www.mongodb.org/static/pgp/server-6.0.asc | \
   sudo gpg -o /usr/share/keyrings/mongodb-server-6.0.gpg \
   --dearmor
   echo "deb [ arch=amd64,arm64 signed-by=/usr/share/keyrings/mongodb-server-6.0.gpg ] https://repo.mongodb.org/apt/ubuntu jammy/mongodb-org/6.0 multiverse" | sudo tee /etc/apt/sources.list.d/mongodb-org-6.0.list
sudo apt-get update
sudo apt-get install -y mongodb-org
echo "mongodb-org hold" | sudo dpkg --set-selections
echo "mongodb-org-database hold" | sudo dpkg --set-selections
echo "mongodb-org-server hold" | sudo dpkg --set-selections
echo "mongodb-mongosh hold" | sudo dpkg --set-selections
echo "mongodb-org-mongos hold" | sudo dpkg --set-selections
echo "mongodb-org-tools hold" | sudo dpkg --set-selections

#MongoDB Compass
wget https://downloads.mongodb.com/compass/mongodb-compass_1.43.0_amd64.deb
sudo dpkg -i mongodb-compass_1.43.0_amd64.deb

#Create a worlspace
cd
mkdir -p ~/ros2-tms-for-construction_ws/src

#Clone this repository
cd ~/ros2-tms-for-construction_ws/src
git clone https://github.com/irvs/ros2_tms_for_construction.git

# install required python packages
cd ~/ros2-tms-for-construction_ws/src/ros2_tms_for_construction
python3 -m pip install -r requirements.txt --quiet --no-input

#setup MongoDB
sudo systemctl start mongod
cd ~/ros2-tms-for-construction_ws/src/ros2_tms_for_construction/demo
unzip rostmsdb_collections.zip
mongorestore --drop dump

#Setup BehaviorTree.cpp
sudo apt install -y libzmq3-dev libboost-dev libncurses5-dev libncursesw5-dev
cd
git clone --branch v3.8 https://github.com/BehaviorTree/BehaviorTree.CPP.git
cd BehaviorTree.CPP
mkdir build; cd build
cmake ..
make
sudo make install
cd && rm -rf BehaviorTree.CPP

#Setup mongocxx/bsoncxx
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

#Setup Groot
sudo apt install -y qtbase5-dev libqt5svg5-dev libzmq3-dev libdw-dev
cd ~/ros2-tms-for-construction_ws/src && git clone https://github.com/BehaviorTree/Groot.git
cd .. && colcon build --packages-select groot

#Install nlohmann-json library
sudo apt install nlohmann-json3-dev

#Setup OPERA
# Install dbcppp
cd && git clone --recurse-submodules https://github.com/genkiiii/dbcppp.git
cd dbcppp && mkdir build && cd build

# Install caranry
cd && git clone https://github.com/djarek/canary.git
cd canary && mkdir build && cd build
cmake ..
sudo make install

# Install rttr
# sudo apt install doxygen
# https://github.com/irvs/rttr.git # This package is a private repository. Please wait a while until it is made public.
# cd rttr && mkdir build && cd build
# cmake ..
# sudo make install
# echo 'export RTTR_DIR=/home/common/rttr/build/install/' >> ~/.bashrc
# source ~/.bashrc

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
# cd .. && mkdir ic120 && cd ic120
# git clone https://github.com/pwri-opera/ic120_ros2.git  # This package is a private repository. Please wait a while until it is made public.
# git clone https://github.com/pwri-opera/gnss_localizer_ros2.git   # This package is a private repository. Please wait a while until it is made public.
# git clone https://github.com/pwri-opera/ic120_com3_ros.git  # This package is a private repository. Please wait a while until it is made public.



#Setup Moveit! and Nav2
# install MoveIt!
sudo apt -y install ros-humble-*moveit*
# install Nav2
sudo apt -y install ros-humble-*nav2*

#Build the workspace 
cd ~/ros2-tms-for-construction_ws
colcon build && source install/setup.bash


