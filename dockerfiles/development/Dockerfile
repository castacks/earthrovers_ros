FROM osrf/ros:humble-desktop

# Install tmuxp
RUN apt-get update && apt-get install -y \
    tmux \
    tmuxp \
    && rm -rf /var/lib/apt/lists/*

# Install known dependencies of some of the ROS2 earth rovers packages.
# NOTE: Technically, this is not good practice in the name of flexibility for a
# development container--these should really be installed by rosdep once the
# container is already running. Practically, this makes our lives easier for the
# time being.
RUN apt-get update && apt-get install -y \
    python3-requests \
    python3-yaml \
    python3-transforms3d \
    python3-pyproj \
    ros-humble-geographic-info \
    && rm -rf /var/lib/apt/lists/*