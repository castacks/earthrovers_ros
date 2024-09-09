"""Node with various navsat_transform like tasks and services.

NOTE: Some of the functionality of this node parallels what is going on in the
navsat_transform_node of robot_localization, but this node offers a few key
additional features that have not yet been added to navsat_transform_node (like
a service to transform GPS coordinates to UTM coordinates). So, is meant to be
used in conjunction with navsat_transform_node, not as a replacement for it.
"""

import rclpy
from rclpy.node import Node

class NavNode(Node):
    """Node that defines a set of services for interacting with the Earth Rovers
    SDK mission endpoints.
    """

    def __init__(self):
        super().__init__("mission_controller")

    # Rough overview of what I want this node to do / provide.

    # 1. Service to transform GPS coordinates to UTM coordinates. I.e., given a
    #    GeoPoint, we transform it to UTM coordinates. Can maybe use the exsting
    #    Geodessy functions to do this. This should be implemented as a function
    #    and then that function called by the service.
    # 2. Similar service to the above, but that transforms the coordinates
    #    directly to the odom frame. This should be implemented as a function
    #    and used by the service.
    # NOTE: The above two services should be implemented to operate on LISTs
    #    of GeoPoints/Poses, not just one (just a quick way to generalize that
    #    behavior). 
    # 2. There should be a set_datum service that accepts a GeoPose and uses
    #    this as the new datum for the UTM transform. This should also be a
    #    function that is then used by the service.
    # 3. The node should subscribe to GPS measurements, use the function to
    #    convert these to UTM, then transform these to the odom frame using the
    #    UTM-->odom transform, and then use those to publish an Odometry
    #    message. NOTE: THIS SHOULD NOT EXIST ONCE WE HAVE THE
    #    ROBOT_lOCALIZATION's EKF running--as this just gives us an infrequent,
    #    sorta crude position estimate. EKF with other sensor data can give us a
    #    smoother, real odometry estimate.
    # 4. Once the datum is set, this node should constantly be publishing a tf
    #    transform from the UTM frame to the odom frame (or whatever world-fixed
    #    frame the node is configured with). This is so that other nodes can
    #    take UTM coordinates and transform them to the odom frame if needed.
    #    Additionally this node will need that transform internally for the
    #    above services and topics.

    


def main(args=None):
    rclpy.init(args=args)
    nav_node = NavNode()
    rclpy.spin(nav_node)
    nav_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
