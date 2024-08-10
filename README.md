# earthrovers_ros
ROS Wrapper for the FrodoBots Earth Rover SDK

# Running in a Docker Container with Rocker

## Prerequisites
- Docker engine ([installation
  instructions](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
  and [post-install instructions](https://docs.docker.com/engine/install/linux-postinstall/))
- OSRF's rocker ([installation instructions](https://github.com/osrf/rocker?tab=readme-ov-file#debians-recommended))

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
docker exec -it /bin/bash earthrovers_ros
```
See the [documentation for docker
exec](https://docs.docker.com/reference/cli/docker/container/exec/) for more
details.