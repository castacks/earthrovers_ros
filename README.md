# earthrovers_ros
ROS 2 packages for interacting with the [FrodoBots](https://www.frodobots.ai/)
[Earth Rovers SDK](https://github.com/frodobots-org/earth-rovers-sdk).

![rviz](./images/earthrovers_rviz.png)

## Features
- Control Earth Rover Challenge FrodoBots with ROS [`Twist`](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html) messages
- Front and rear camera frames published as standard [`Image`](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html) messages
- GPS fixes published as [`NavSatFix`](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/NavSatFix.html) messages, IMU readings published as [`Imu`](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html)
  and [`MagneticField`](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/MagneticField.html) messages
- Basic
  [URDF](https://industrial-training-master.readthedocs.io/en/melodic/_source/session3/Intro-to-URDF.html)
  description of the FrodoBots
- Get mission checkpoints from SDK and track which checkpoints have been
  reached
- Docker image for standardized development/deployment environment

## ROS Packages
See each package's README for more details on its nodes and the interfaces for
each.
### [earthrovers_base](./earthrovers_base/)
Contains `base` node that publishes imu, gps, and other data from the Earth
Rover SDK's `/data` endpoint.
### [earthrovers_bringup](./earthrovers_bringup/)
Package to house launch files for bringing up the Earth Rovers ROS stack.
### [earthrovers_description](./earthrovers_description/)
Contains URDF description of the FrodoBot. Used to establish all the static
transforms between the rover's different links/frames.
### [earthrovers_interfaces](./earthrovers_interfaces/)
Contains custom service and message definitions.
### [earthrovers_navigation](./earthrovers_navigation/)
Mainly contains a node (`nav`) for producing "fake" odometry odometry messages derived
from the change in GPS positions. Also contains a node to publish consecutive
mission checkpoints as they are reached (`waypoint_receiver`). Note that any of
the other nodes in this package are experimental/not finished.
### [earthrovers_vision](./earthrovers_vision/)
Contains a node for grabbing front and rear camera frames from the SDK. Also
provides a basic framework for how you might publish camera parameters if you
have them.
### [earthrovers_viz](./earthrovers_viz/)
Houses an RVIZ configuration file for quick and easy visualization.

## Disclaimer
While these packages cover the basics and have pretty documentation, it is VERY
IMPORTANT to note that there are key missing capabilities and broken features.
For example:
- The "odometry" that is published by the `nav` node is derived from change in
  GPS positions, rather than from an inertial sensor or the wheel encoders. This
  results in very "choppy," noisy odometry.
- The camera node sometimes publishes screenshots of the map instead of actual
  camera frames.
- The dimensions specified in the URDF are not *exactly* correct--there are
  slight differences from the hardware spec that need fine-tuned.

Every node in this repo is provided as "best-effort" given how much time we had.
If you have time to add a feature or help resolve some of the issues, we will do
our best to monitor issues and pull requests, but cannot gaurantee a timely
resolution. Please, however, do still open issues for problems that you
encounter that are not yet documented!

<!-- # General Usage Instructions
This section assumes you have basic experience with ROS 2 and are familiar with
creating workspaces, building packages, running nodes, etc. If you are just
getting started with ROS 2, please spend some time with the [tutorials from the
ROS 2 docs](https://docs.ros.org/en/humble/Tutorials.html). If you are mainly
interested in using the packages in this repo, the [client libraries
section](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html)
may be the most immediately helpful.

If you *are* already familiar with ROS, using these packages is simply a matter
of cloning these into your workspace's `src` folder, installing dependencies,
building with `colcon`, and then running the nodes. For completeness, these
steps will begin with creating a fresh workspace, with the steps largely taking
after those outlined in [this
tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#creating-a-workspace).

## Prerequisites
- You already have ROS 2 Humble **Desktop** installed on a [ROS 2 Humble capable
system](https://www.ros.org/reps/rep-2000.html#humble-hawksbill-may-2022-may-2027)

## 1. Create a new workspace
Cd to wherever you want to place your 

## 2. Clone the repository
First, clone this repository containing all the packages into an existing
ROS 2 workspace's `src` folder.

```
cd path/to/your/workspaces/src/folder
git clone https://github.com/castacks/earthrovers_ros.git
```
## 2. Install package dependencies
Use [rosdep](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html#rosdep-operation) to install Python and system dependencies.
```
sudo apt-get update && \
rosdep update --rosdistro $ROS_DISTRO && \
rosdep install --from-paths src -y -r --ignore-src
```

## 3. Build with colcon

## 4. Launch the nodes

These packages can be used like any other "source packages" that you clone into
your workspace's `src`.

If you are new to ROS and unfamiliar with this pattern or are confused with how
this works, follow the 

If you are new to ROS but still want to use these nodes, all you need to know
is that these packages must be placed in the `src` folder of your workspace. In
this case, you can simply clone this entire repository into your workspace's
`src` folder. Then, use rosdep to install all the system dependencies required
by these packages. Finally, use colcon to build the workspace.

The above describes a general pattern of working with other ROS packages in your
ROS workspace. For context, the most simple example of this pattern is first
shown in the [Creating a workspace page](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#clone-a-sample-repo) of the ROS2 docs. -->

## Getting Started
### Set up the SDK
Before you can use any of these ROS packages to interface with the Earth Rovers
SDK, you need to make sure you have the Earth Rovers SDk set up! Follow
Santiag's great instructions
[here](https://github.com/frodobots-org/earth-rovers-sdk?tab=readme-ov-file#getting-started)
for getting started with that first. Basically, before you run any of these ROS
nodes, you'll want to make sure you already have the SDK connected to a robot
and have [**started the mission.**](https://github.com/frodobots-org/earth-rovers-sdk?tab=readme-ov-file#post-start-mission)

### Recommended Setup
If you are just getting started with ROS and are new to all this (**or** are a
pro and want the least-resistance path to getting the stack up and running), we
**highly recommend** you follow the [step-by-step instructions we provide for
running the Earth Rovers ROS stack in a Docker container outlined
below.](#running-in-a-docker-container-with-rocker). These steps include setting
up the Earth Rovers ROS packages in a Docker container with ROS 2 Humble Desktop
already installed.

### If You Are Already a ROS Pro
If you generally know your way around ROS / ROS packages, this should be pretty
familiar. Just clone this repository into your workspace's `src` folder, run
`colcon build`, and launch the nodes using the
[`rover_bringup_launch.py`](earthrovers_ros/earthrovers_bringup/launch/rover_bringup_launch.py)
launch file. Each package has pretty decent documentation with all of the topics
its nodes publish and subscribe to, and you will probably be able to piece
things together fairly quickly.


# Running in a Docker Container with Rocker
This repository contains Dockerfiles that should be useful if you're looking to
work on these ROS packages or deploy them in a container.

## Prerequisites and Assumptions
- Host machine with Ubuntu 20.04 or above (x86 or ARM)
- Docker engine ([installation
  instructions](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
  and [post-install instructions](https://docs.docker.com/engine/install/linux-postinstall/))
- OSRF's rocker ([installation
  instructions](https://github.com/osrf/rocker?tab=readme-ov-file#debians-recommended))

*NOTE: If you are familiar with ROS, note that none of those prerequisistes are
strictly necessary. We just highly recommended them for the easiest
getting-started experience.*

## Development Container Setup
If you are looking to make changes / contribute to this package, you can save
yourself a ton of headache by working in a development container.

For now, our development container is very straightforward and simply provides
an Ubuntu FS with ROS desktop installed. The basic idea is that you build our
docker image, create a new container from the image with your workspace added,
use rosdep to install dependencies, set up VSCODE, and you're ready to go.

### 1. Create a new ROS workspace on your host computer
Somewhere on your host, create a new workspace and source directory. Below, we
create a workspace called "earthrovers_ws" in your home directory--but any
folder name placed anywhere should work.
```
mkdir -p ~/earthrovers_ws/src && \
cd ~/earthrovers_ws/src
```

### 2. Clone the Earth Rovers ROS repository into your workspace's src folder
```
git clone https://github.com/castacks/earthrovers_ros.git &&\
cd earthrovers_ros
```

### 3. Build the development Docker image
In the root of the repository you just cloned, run the following command to
build the development Docker image based on the development Dockerfile.
```
docker build -t earthrovers_development dockerfiles/development
```
### 4. Create development container from the image
First, set an environment variable EARTHROVERS_WS to the path to your local
workspace.
```
export EARTHROVERS_WS=~/earthrovers_ws
```
Next, use rocker to create a new container using the docker image we just built:
```
rocker --x11 --user --ssh --privileged --network host --name earthrovers_ros --volume $EARTHROVERS_WS:/earthrovers_ws -- earthrovers_development
```
### 5. Install Earth Rovers ROS package dependencies in container
Next, in order that all the Earth Rovers ROS packages have all the libraries and
packages they need to work properly, we need to install those inside the
container.

To this, first navigate to `earthrovers_ws` directory, and then run the
following commands **in the container**:
```
cd /earthrovers_ws && \
sudo apt-get update && \
rosdep update --rosdistro $ROS_DISTRO && \
rosdep install --from-paths src -y -r --ignore-src
```

### 6. Connect VSCode to your development container
While you can edit the source files with your editor of choice on your host-OS,
your editor likely won't have a way to know how to resolve packages or header
files that your modules / source files are importing that are installed within
the container's filesystem, but not your hosts.

To resolve this, you can actually run a VSCode server from within your
development container and connect your VSCode client to this server. This is all
made very easy by the with the Dev Containers extension that is installed as a
part of the [Remote Development extension pack](https://code.visualstudio.com/docs/remote/remote-overview#_remote-development-extension-pack).

Then, follow [these
instructions](https://code.visualstudio.com/docs/devcontainers/attach-container)
from the [devcontainers
documentation](https://code.visualstudio.com/docs/devcontainers/containers) to
attach your VSCode client to the VSCode server running in your container.

## Running Stack in Development Container
Once you have your development environment set up according to the above steps,
you can build the packages in the workspace and then run the nodes.

### 1. Build the workspace
Once your workspace is set up with all the Earth Rovers ROS packages, run the
following to build the workspace.
```
source /opt/ros/humble/setup.bash && \
cd /earthrovers_ws && \
colcon build
```

### 2. Launch nodes with tmuxp
When developing a handful of connected ROS nodes, it can often be helpful to run
each one of them in a separate terminal window for easier debugging. You can run
all of the nodes of the stack in separate tmux windows using the provided tmuxp
script using the following command:

**NOTE: The earth rovers SDK needs to be running and mission must be started in
order for these ROS nodes to work as expected!** Follow [these instructions](https://github.com/frodobots-org/earth-rovers-sdk?tab=readme-ov-file#getting-started) here
to get your access token, bot slugs, and start the local SDK server.

```
tmuxp load /earthrovers_ws/src/earthrovers_ros/tmuxp_configs/dev.yaml
```
*Hint: To exit out of the whole tmux session, press ctrl+b, type
":kill-session, and then press enter.*

If you're not looking to debug the individual nodes of the stack, there is also
a provided launch file here (which you'll probably want to use + extend if
you're just using the stack for another application).
```
ros2 launch earthrovers_bringup rover_bringup_launch.py
```

## Teleoperating a Rover
Once you have the stack running in the development container using the steps
described above, you should have everything you need to remotely control the
FrodoBot via the ROS control interface.

If you successfully launched all the nodes using the `tmuxp` command, one of the
window panes opened should look something like this:

![teleop_keyboard](./images/teleop_keyboard.png)

Click on this pane to bring it into focus. Then, you can use the `i`, `j`, `k`,
and `l` keys to move around the earth rover. An `rviz` window should have
appeared--check out the front and camera image previews to see if its moving.
Rviz will also show the rover's position in 3D space as it gets GPS updates.