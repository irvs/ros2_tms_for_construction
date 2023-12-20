FROM ros:humble
ARG OVERLAY_WS=/opt/ros2_tms_for_construction_ws

ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src
COPY ./ ./ros2_tms_for_construction

# python packages
WORKDIR $OVERLAY_WS/src/ros2_tms_for_construction

# install dependencies
RUN apt update && apt install -y \
    git \
    nlohmann-json3-dev \
    gnupg \
    curl \
    python3-pip \
    libzmq3-dev \
    libboost-dev \
    libncurses5-dev \
    libncursesw5-dev \
    wget \
    libgoogle-glog-dev \ 
    && rm -rf /var/lib/apt/lists/*


RUN python3 -m pip install -r requirements.txt

WORKDIR /home
RUN curl -fsSL https://pgp.mongodb.com/server-7.0.asc | \
   sudo gpg -o /usr/share/keyrings/mongodb-server-7.0.gpg \
   --dearmor

RUN echo "deb [ arch=amd64,arm64 signed-by=/usr/share/keyrings/mongodb-server-7.0.gpg ] https://repo.mongodb.org/apt/ubuntu jammy/mongodb-org/7.0 multiverse" | sudo tee /etc/apt/sources.list.d/mongodb-org-7.0.list
RUN apt-get update && apt-get install -y mongodb-org \
    && rm -rf /var/lib/apt/lists/*


# RUN apt install libzmq3-dev libboost-dev libncurses5-dev libncursesw5-dev
RUN git clone --branch v3.8 https://github.com/BehaviorTree/BehaviorTree.CPP.git \
    && cd BehaviorTree.CPP \
    && mkdir build; cd build \
    && cmake .. \
    && make && make install


RUN wget https://github.com/mongodb/mongo-c-driver/releases/download/1.24.4/mongo-c-driver-1.24.4.tar.gz \
    && tar -xzf mongo-c-driver-1.24.4.tar.gz \
    && cd mongo-c-driver-1.24.4 \
    && mkdir cmake-build \
    && cd cmake-build \
    && cmake -DENABLE_AUTOMATIC_INIT_AND_CLEANUP=OFF .. -DCMAKE_INSTALL_PREFIX=/usr/local \
    && make && make install

RUN curl -OL https://github.com/mongodb/mongo-cxx-driver/releases/download/r3.8.1/mongo-cxx-driver-r3.8.1.tar.gz \
    && tar -xzf mongo-cxx-driver-r3.8.1.tar.gz \
    && cd mongo-cxx-driver-r3.8.1/build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DBSONCXX_POLY_USE_BOOST=1 -DMONGOCXX_OVERRIDE_DEFAULT_INSTALL_PREFIX=OFF \
    && cmake --build . \
    && cmake --build . --target install


RUN export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH \
    && echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc \
    && cd && rm -rf mongo-c-driver-1.24.4 mongo-c-driver-1.24.4.tar.gz mongo-cxx-driver-r3.8.1 mongo-cxx-driver-r3.8.1.tar.gz

# source underlay for shell
RUN echo 'source "/opt/ros/humble/setup.bash"' >> /etc/bash.bashrc
RUN echo 'source "$OVERLAY_WS/install/setup.bash"' >> /etc/bash.bashrc

WORKDIR $OVERLAY_WS/src/ros2_tms_for_construction
