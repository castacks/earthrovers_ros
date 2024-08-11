# earthrovers_ros
ROS Wrapper for the FrodoBots Earth Rover SDK

# Running in a Docker Container with Rocker
This repository contains Dockerfiles that should be useful if you're looking to
work on these ROS packages or deploy them in a container.

## Prerequisites
- Docker engine ([installation
  instructions](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
  and [post-install instructions](https://docs.docker.com/engine/install/linux-postinstall/))
- OSRF's rocker ([installation instructions](https://github.com/osrf/rocker?tab=readme-ov-file#debians-recommended))

Note: rocker is not *absolutely* necessary--but **greatly** simplifies the
process of running a container with all the options you usually want.

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
git clone https://github.com/castacks/earthrovers_ros.git
```

### 3. Build the development docker image
In the root of the repository you just cloned, run the following command to
build the development Docker image based on the development Dockerfile.
```
docker build -t earthrovers_development dockerfiles/development
```
### 4. Create development container from the image
First, set an environment variable EARTHROVERS_WS to the path to your local
workspace.
```
export EARTHROVERS_WS=/path/to/your/workspace/folder
```
Next, use rocker to create a new container using the docker image we just built:
```
rocker --x11 --user --ssh --network host --name earthrovers_ros --volume $EARTHROVERS_WS:/earthrovers_ws -- earthrovers_desktop
```
### 5. Install Earth Rovers ROS package dependencies in container
Next, in order that all the Earth Rovers ROS packages have all the libraries and
packages they need to work properly, we need to install those inside the
container.

To this, first navigate to `earthrovers_ws` directory:
```
cd /earthrovers_ws
```

To do this, run the following commands **in the container**:
```
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

## Deployment Container Setup
If you only need to run the nodes of these packages without making any changes,
the deployment container is probably what you're looking for.

## Setup
### Build one of the earthrovers_ros docker images.
For the desktop image (includes useful visualization tools like rviz).
```
docker build -t earthrovers_desktop dockerfiles/desktop
```
For the base image (for just running the core nodes, you have debugging nodes
installed somewhere else).
```
docker build -t earthrovers_base dockerfiles/base
```
### Create a container from the image
First, set an environment variable EARTHROVERS_WS to the path to your local
workspace.
```
export EARTHROVERS_WS=/path/to/your/workspace/folder
```
Next, use rocker to create a new container using the earth rover's docker image.
```
rocker --x11 --user --ssh --network host --name earthrovers_ros --volume $EARTHROVERS_WS:/earthrovers_ws -- earthrovers_desktop
```

### Connecting to an existing container / creating a new terminal session
If you want to create an additional terminal session *inside* a container that
you have already spun up, you can create and attach to a new terminal session
with the following command:
```
docker exec -it earthrovers_ros /bin/bash
```
See the [documentation for docker
exec](https://docs.docker.com/reference/cli/docker/container/exec/) for more
details.

### Developing earthrovers_ros with docker in vscode
When working on ROS packages in Visual Studio Code, it can be very helpful to
make VS Code "aware" of the packages and environment created within your Docker
container (be it for typehints, resolving paths and dependencies, etc.).

To do this, you can connect your VS Code instance to your container using the
Dev Containers extension that is installed as a part of the [Remote Development
extension
pack](https://code.visualstudio.com/docs/remote/remote-overview#_remote-development-extension-pack).

Then, follow [these
instructions](https://code.visualstudio.com/docs/devcontainers/attach-container)
from the [devcontainers
documentation](https://code.visualstudio.com/docs/devcontainers/containers) to
attach your vscode client to your container.