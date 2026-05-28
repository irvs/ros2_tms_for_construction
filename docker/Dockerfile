# syntax=docker/dockerfile:1.6
#
# ros2_tms_for_construction — X11 + MongoDB-separated dev image
#
# Base : osrf/ros:humble-desktop-full (Ubuntu 22.04 jammy)
# GUI  : X11 forward (no VNC)
# DB   : provided by separate `mongodb` service in compose.yaml
#
# Build context expected at the repository root (../). This Dockerfile is
# referenced as `dockerfile: docker/Dockerfile` from `docker/compose.yaml`.

FROM osrf/ros:humble-desktop-full

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# ---------------------------------------------------------------------------
# 1. APT dependencies
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git curl wget gnupg ca-certificates pkg-config \
        unzip xvfb gosu \
        python3-pip python3-vcstool python3-colcon-common-extensions \
        libzmq3-dev libboost-dev libncurses5-dev libncursesw5-dev libssl-dev \
        nlohmann-json3-dev rapidjson-dev \
        qtbase5-dev libqt5svg5-dev libdw-dev \
        ros-humble-robot-localization \
    && apt-get install -y --no-install-recommends \
        'ros-humble-*moveit*' \
        'ros-humble-*nav2*' \
        'ros-humble-*tf*' \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 2. MongoDB database tools (mongorestore / mongodump for `restore-db.sh`)
# ---------------------------------------------------------------------------
RUN curl -fsSL https://www.mongodb.org/static/pgp/server-6.0.asc \
      | gpg -o /usr/share/keyrings/mongodb-server-6.0.gpg --dearmor \
    && echo "deb [arch=amd64,arm64 signed-by=/usr/share/keyrings/mongodb-server-6.0.gpg] https://repo.mongodb.org/apt/ubuntu jammy/mongodb-org/6.0 multiverse" \
      > /etc/apt/sources.list.d/mongodb-org-6.0.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends mongodb-database-tools \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 3. BehaviorTree.CPP v3.8 (source build → /usr/local)
# ---------------------------------------------------------------------------
RUN cd /tmp \
    && git clone --depth 1 --branch v3.8 https://github.com/BehaviorTree/BehaviorTree.CPP.git \
    && cmake -S BehaviorTree.CPP -B BehaviorTree.CPP/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build BehaviorTree.CPP/build -j"$(nproc)" --target install \
    && rm -rf BehaviorTree.CPP

# ---------------------------------------------------------------------------
# 4. mongo-c-driver 1.24.4 + mongo-cxx-driver r3.8.1 (source build → /usr/local)
# ---------------------------------------------------------------------------
RUN cd /tmp \
    && wget -q https://github.com/mongodb/mongo-c-driver/releases/download/1.24.4/mongo-c-driver-1.24.4.tar.gz \
    && tar -xzf mongo-c-driver-1.24.4.tar.gz \
    && cmake -S mongo-c-driver-1.24.4 -B mongo-c-driver-1.24.4/build \
        -DENABLE_AUTOMATIC_INIT_AND_CLEANUP=OFF \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
    && cmake --build mongo-c-driver-1.24.4/build -j"$(nproc)" --target install \
    && rm -rf mongo-c-driver-1.24.4*

RUN cd /tmp \
    && wget -q https://github.com/mongodb/mongo-cxx-driver/releases/download/r3.8.1/mongo-cxx-driver-r3.8.1.tar.gz \
    && tar -xzf mongo-cxx-driver-r3.8.1.tar.gz \
    && cmake -S mongo-cxx-driver-r3.8.1 -B mongo-cxx-driver-r3.8.1/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DBSONCXX_POLY_USE_BOOST=1 \
        -DMONGOCXX_OVERRIDE_DEFAULT_INSTALL_PREFIX=OFF \
    && cmake --build mongo-cxx-driver-r3.8.1/build -j"$(nproc)" --target install \
    && rm -rf mongo-cxx-driver-r3.8.1*

ENV LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}

# ---------------------------------------------------------------------------
# 5. Python dependencies
#    Placed after the heavy C++ source builds so changes to requirements.txt
#    do not invalidate those cache layers.
# ---------------------------------------------------------------------------
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt && rm /tmp/requirements.txt

# ---------------------------------------------------------------------------
# 6. External ROS 2 packages via vcstool
#    (cloned into /workspace/src/, built at first container run)
# ---------------------------------------------------------------------------
RUN mkdir -p /workspace/src
COPY docker/src.repos /tmp/src.repos
RUN cd /workspace/src && vcs import < /tmp/src.repos && rm /tmp/src.repos

# ---------------------------------------------------------------------------
# 7. Non-root user matching host UID/GID for X11 + bind-mount permissions.
#    User creation only — privileges are dropped at runtime via `gosu` inside
#    entrypoint.sh. No sudoers NOPASSWD is granted.
# ---------------------------------------------------------------------------
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    && mkdir -p /workspace/build /workspace/install /workspace/log \
    && chown -R ${USERNAME}:${USERNAME} /workspace

RUN echo 'source /opt/ros/humble/setup.bash' >> /home/${USERNAME}/.bashrc \
    && echo '[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash' >> /home/${USERNAME}/.bashrc \
    && echo 'export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}' >> /home/${USERNAME}/.bashrc

# Container starts as root so entrypoint.sh can chown the named volumes on
# first mount; it then drops to `${USERNAME}` via gosu before exec'ing CMD.
# restore-db.sh is placed on PATH so it can be invoked as `restore-db.sh`
# from `docker compose exec tms ...` without typing the full path.
WORKDIR /workspace
COPY docker/scripts/entrypoint.sh /entrypoint.sh
COPY docker/scripts/restore-db.sh /usr/local/bin/restore-db.sh
RUN chmod +x /entrypoint.sh /usr/local/bin/restore-db.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
