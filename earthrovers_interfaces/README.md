# earthrovers_interfaces
Packages containing custom service and message types for interacting with the
Frodobots Earth Rovers SDK.

Custom service types created according to this official ROS2 Humble tutorial:
https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html.
Follow that tutorial for adding additional messages and services.

Custom service **server** implementations created by following another official ROS2 Humble
tutorial:
https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html

If we eventually choose to create the desired mission and waypoint management
behaviors using behavior trees, follow this tutorial to create clients that
serve as Action Nodes https://www.behaviortree.dev/docs/ros2_integration

NOTE: While there are a few service definitions hanging out in here, they are
not actively used by any working nodes and should be ignored until those nodes
get resurrected.