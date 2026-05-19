FROM osrf/ros:humble-ros-base AS builder

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-dev \
    build-essential \
    libpcl-all-dev \
    libopencv-dev \
    libyaml-cpp-dev \
    git \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /b2_ws

COPY src/b2_thesis_fusion/ src/b2_thesis_fusion/

RUN cd src \
    && git clone https://github.com/PRBonn/kiss-icp.git \
    && git clone https://github.com/liangheming/FASTLIO2_ROS2.git

RUN apt-get update && rosdep init || true \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -y \
    && rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"


FROM osrf/ros:humble-ros-base

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-rosdep \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-robot-localization \
    ros-humble-nav2-amcl \
    ros-humble-nav2-map-server \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-velodyne \
    ros-humble-kiss-icp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /b2_ws

COPY --from=builder /b2_ws/src/ src/

RUN apt-get update && rosdep init || true && rosdep update \
    && rosdep install --from-paths src --ignore-src --dependency-types=exec -y \
    && rm -rf /var/lib/apt/lists/* \
    && rm -rf src

RUN pip3 install evo --upgrade

COPY --from=builder /b2_ws/install /b2_ws/install

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "source /b2_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
