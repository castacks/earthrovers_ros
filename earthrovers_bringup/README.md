# earthrovers_bringup
This package contains launch files for bringing up the Earth Rovers ROS nodes.

## `rover_bringup_launch.py`
This launch file brings up all the nodes in the stack, as well as a
preconfigured RVIZ instance for visualization.

## `rover_tf_launch.py`
This launch file only runs the `robot_state_publisher` and
`joint_state_publisher` nodes as a simple way to introduce the rover's static
and dynamic joint states / frame transformations based on the rover's URDF file.
This launch file is helpful when you are trying to debug other nodes in the
system but still need the body link transformations available.