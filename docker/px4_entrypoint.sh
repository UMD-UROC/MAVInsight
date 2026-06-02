#!/bin/sh
set -e

# Detect install prefix. SIH uses /opt/px4, Gazebo uses /opt/px4-gazebo.
if [ -d /opt/px4-gazebo ]; then
    PX4_PREFIX=/opt/px4-gazebo
else
    PX4_PREFIX=/opt/px4
fi

# On Docker Desktop, host.docker.internal lets PX4 send MAVLink packets to
# QGroundControl on the host without publishing the QGC UDP port from Docker.
DOCKER_HOST_IP=$(getent ahostsv4 host.docker.internal 2>/dev/null | awk '/STREAM/ {print $1; exit}')

if [ -n "$DOCKER_HOST_IP" ]; then
    sed -i "s/mavlink start -x -u/mavlink start -x -t $DOCKER_HOST_IP -u/g" \
        "$PX4_PREFIX/etc/init.d-posix/px4-rc.mavlink"
fi

# The ROS container shares this container's network namespace, so the
# MicroXRCEAgent is reachable from PX4 on localhost.
exec "$PX4_PREFIX/bin/px4" "$@"
