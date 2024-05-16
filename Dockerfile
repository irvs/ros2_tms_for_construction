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



# pUll the tiryoh/ros2-desktop-vnc docker image
FROM tiryoh/ros2-desktop-vnc:humble
# LABEL maintainer= "kasahara.yuichiro.res@gmail.com"
RUN . /opt/ros/humble/setup.sh

# set the environment variable OVERLAY_WS to the path of the workspace(ros2_tms_for_construction_ws)
ARG OVERLAY_WS=/opt/ros2_tms_for_construction_ws

WORKDIR $OVERLAY_WS/src

# copy ros2_tms_for_construction packages from local environment to docker environment
COPY ./ ./ros2_tms_for_construction

# install dependencies
RUN apt update && apt install -y \
    git \
    pip \
    nlohmann-json3-dev \
    python3-pip \
    libzmq3-dev \
    libboost-dev \
    libncurses5-dev \
    libncursesw5-dev \
    wget \
    libgoogle-glog-dev \ 
    ros-humble-behaviortree-cpp-v3 \
    libgtk-3-0 \
    libnotify4 \
    libnss3 \
    libxtst6 \
    xdg-utils \
    libatspi2.0-0 \
    libdrm2 \
    libgbm1 \
    libxcb-dri3-0 \
    libglib2.0-bin \
    gvfs \
    libsecret-1-0 \
    gnome-keyring \
    wget \
    unzip \
    xvfs \
    && rm -rf /var/lib/apt/lists/*

# install pymongo
RUN pip install pymongo

# install mongodb
WORKDIR /home
RUN apt update && apt install -y gnupg curl \
    && curl -fsSL https://www.mongodb.org/static/pgp/server-6.0.asc | \
    gpg -o /usr/share/keyrings/mongodb-server-6.0.gpg \
    --dearmor
RUN echo "deb [ arch=amd64,arm64 signed-by=/usr/share/keyrings/mongodb-server-6.0.gpg ] https://repo.mongodb.org/apt/ubuntu jammy/mongodb-org/6.0 multiverse" | tee /etc/apt/sources.list.d/mongodb-org-6.0.list \
    && apt update && apt install -y mongodb-org \
    && echo "mongodb-org hold" | dpkg --set-selections && echo "mongodb-org-database hold" | dpkg --set-selections \
    && echo "mongodb-org-server hold" | dpkg --set-selections && echo "mongodb-mongosh hold" | dpkg --set-selections \
    && echo "mongodb-org-mongos hold" | dpkg --set-selections && echo "mongodb-org-tools hold" | dpkg --set-selections
# データディレクトリの作成
RUN mkdir -p /data/db

# install mongodb compass
RUN wget https://downloads.mongodb.com/compass/mongodb-compass_1.43.0_amd64.deb \
    && dpkg -i mongodb-compass_1.43.0_amd64.deb
# Xvfb のセットアップと dbus & MongoDB serverの起動
RUN chmod +x /usr/bin/mongodb-compass
CMD Xvfb :1 -screen 0 1024x768x16 & \
    service dbus start && \
    export DISPLAY=:1 && \
    xhost +local:root

# install related python packages for RSO2-TMS for Construction
WORKDIR $OVERLAY_WS/src/ros2_tms_for_construction
RUN python3 -m pip install -r requirements.txt

# setup MongoDB
WORKDIR $OVERLAY_WS/src/ros2_tms_for_construction/demo
RUN unzip rostmsdb_collections.zip

# execute entrypoint.sh
WORKDIR $OVERLAY_WS/src/ros2_tms_for_construction
RUN chmod +x entrypoint.sh
CMD ["./entrypoint.sh"]

# setup BehaviorTree.CPP 
WORKDIR /home
RUN apt install  -y libzmq3-dev libboost-dev libncurses5-dev libncursesw5-dev \
    && git clone --branch v3.8 https://github.com/BehaviorTree/BehaviorTree.CPP.git
WORKDIR /home/BehaviorTree.CPP
RUN mkdir build
WORKDIR /home/BehaviorTree.CPP/build
RUN cmake .. && make && make install
WORKDIR /home
RUN rm -rf BehaviorTree.CPP

# setup mongocxx / bsoncxx
WORKDIR /home
RUN wget https://github.com/mongodb/mongo-c-driver/releases/download/1.24.4/mongo-c-driver-1.24.4.tar.gz \
    && tar -xzf mongo-c-driver-1.24.4.tar.gz
WORKDIR /home/mongo-c-driver-1.24.4
RUN mkdir cmake-build
WORKDIR /home/mongo-c-driver-1.24.4/cmake-build
RUN cmake -DENABLE_AUTOMATIC_INIT_AND_CLEANUP=OFF .. -DCMAKE_INSTALL_PREFIX=/usr/local \
    && make install
WORKDIR /home
RUN curl -OL https://github.com/mongodb/mongo-cxx-driver/releases/download/r3.8.1/mongo-cxx-driver-r3.8.1.tar.gz \
    && tar -xzf mongo-cxx-driver-r3.8.1.tar.gz
WORKDIR /home/mongo-cxx-driver-r3.8.1/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DBSONCXX_POLY_USE_BOOST=1 -DMONGOCXX_OVERRIDE_DEFAULT_INSTALL_PREFIX=OFF \
    && cmake --build . \
    && cmake --build . --target install \
    && export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH \
    && echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
WORKDIR /home
RUN rm -rf mongo-c-driver-1.24.4 mongo-c-driver-1.24.4.tar.gz mongo-cxx-driver-r3.8.1 mongo-cxx-driver-r3.8.1.tar.gz

# # setup groot
WORKDIR $OVERLAY_WS/src
RUN apt install -y qtbase5-dev libqt5svg5-dev libzmq3-dev libdw-dev \
    && git clone https://github.com/BehaviorTree/Groot.git
WORKDIR $OVERLAY_WS/src/Groot
RUN git submodule update --init --recursive
WORKDIR $OVERLAY_WS/
RUN colcon build --packages-select groot


# setup nlohmann-json3-dev
RUN apt install -y nlohmann-json3-dev

# setup related opera packages
WORKDIR /home
RUN git clone --recurse-submodules https://github.com/genkiiii/dbcppp.git
WORKDIR /home/dbcppp
RUN mkdir build
WORKDIR /home
RUN git clone https://github.com/djarek/canary.git
WORKDIR /home/canary
RUN mkdir build
WORKDIR /home/canary/build
RUN cmake .. && make install
RUN apt install -y doxygen
WORKDIR $OVERLAY_WS/src
RUN git clone -b develop/top https://github.com/irvs/tms_if_for_opera.git
RUN mkdir -p opera/common
WORKDIR $OVERLAY_WS/src/opera/common
RUN git clone https://github.com/pwri-opera/com3.git \
    && git clone https://github.com/pwri-opera/com3_ros.git
WORKDIR $OVERLAY_WS/src/opera
RUN mkdir simulator
WORKDIR $OVERLAY_WS/src/opera/simulator
RUN git clone -b main-ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
WORKDIR $OVERLAY_WS/src/opera
RUN mkdir zx200
WORKDIR $OVERLAY_WS/src/opera/zx200
RUN git clone https://github.com/pwri-opera/zx200_ros2.git
WORKDIR $OVERLAY_WS/src/opera
RUN mkdir ic120
WORKDIR $OVERLAY_WS/src/opera/ic120
# RUN git clone https://github.com/pwri-opera/ic120_ros2.git
# RUN git clone https://github.com/pwri-opera/gnss_localizer_ros2.git
# RUN git clone https://github.com/pwri-opera/ic120_com3_ros.git

# setup moveit! & navigation2
RUN apt install -y ros-humble-*moveit* \
    && apt install -y ros-humble-*nav2*

# build the workspace
WORKDIR $OVERLAY_WS
RUN /bin/bash -c "source /opt/ros/humble/setup.bash"

# source underlay for shell
RUN echo 'source "/opt/ros/humble/setup.sh"' >> /etc/bash.bashrc
RUN echo 'source "$OVERLAY_WS/install/setup.bash"' >> /etc/bash.bashrc