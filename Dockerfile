FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

ENV ROS_DISTRO=humble \
    ROS_WS=/ws

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-yaml \
    python3-scipy \
    python3-pymap3d \
    ros-humble-mavros-msgs \
    ros-humble-foxglove-bridge \
  && rm -rf /var/lib/apt/lists/*

WORKDIR ${ROS_WS}
RUN mkdir -p ${ROS_WS}/src

ARG PX4_MSGS_BRANCH=main
RUN git clone -b ${PX4_MSGS_BRANCH} https://github.com/PX4/px4_msgs.git ${ROS_WS}/src/px4_msgs

ARG MICRO_XRCE_AGENT_VERSION=v2.4.3
RUN git clone -b ${MICRO_XRCE_AGENT_VERSION} https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/Micro-XRCE-DDS-Agent \
  && mkdir -p /tmp/Micro-XRCE-DDS-Agent/build \
  && cd /tmp/Micro-XRCE-DDS-Agent/build \
  && cmake .. \
  && make -j"$(nproc)" \
  && make install \
  && ldconfig /usr/local/lib/ \
  && rm -rf /tmp/Micro-XRCE-DDS-Agent

COPY . ${ROS_WS}/src/mavinsight

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
  && colcon build --symlink-install

COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
