FROM ros:humble-ros-base-jammy

# Clone the Earth Rover's ROS repository.
RUN mkdir -p /earthrovers_ws/src
WORKDIR /earthrovers_ws/src
RUN git clone https://github.com/castacks/earthrovers_ros.git

# Install the Earth Rover's ROS repository dependencies.
RUN sudo apt-get update && rosdep update --rosdistro $ROS_DISTRO
WORKDIR /earthrovers_ws
RUN rosdep install --from-paths src --ignore-src -r -y

# Build the Earth Rover's ROS packages with colcon.
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-install --packages-select earthrovers_ros
# Source the locally installed packages.
RUN source /earthrovers_ws/install/setup.bash

# Set the entrypoint to the Earth Rover's ROS launch file.
ENTRYPOINT ["ros2", "launch", "earthrovers_ros", "rover_bringup_launch.py"]