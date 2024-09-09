"""Node with various navsat_transform like tasks and services.

NOTE: Some of the functionality of this node parallels what is going on in the
navsat_transform_node of robot_localization, but this node offers a few key
additional features that have not yet been added to navsat_transform_node (like
a service to transform GPS coordinates to UTM coordinates). So, is meant to be
used in conjunction with navsat_transform_node, not as a replacement for it.
"""

import rclpy
from rclpy.node import Node
from robot_localization.srv import SetDatum
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

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

class NavNode(Node):
    """Node that defines a set of services for interacting with the Earth Rovers
    SDK mission endpoints.
    """

    def __init__(self):
        super().__init__("mission_controller")

        # Create parameter that this node will use as the world-fixed frame.
        # I.e., the frame whose origin we will anchor at the UTM location of the
        # GPS datum point (via the transform from UTM-->world_frame).
        self.declare_parameter("world_frame", "odom")
        self.declare_parameter("utm_to_world_frame_tf_broadcast_rate", 1.0)

        # Initialize datum to none.
        self._datum = None

        # Create service for setting the datum.
        self.create_service(SetDatum, "set_datum", self._set_datum_callback)

        # Create service for setting datum to most recently received GPS fix.
        self.create_service(Trigger, "set_datum_to_gps_fix", self._set_datum_to_gps_fix_callback)
        
        # Create service for transforming a list of GPS coordinates to UTM
        # coordinates.
        self.create_service(GpsToUtm, "transform_gps_to_utm", self._transform_gps_to_utm_callback)

        # Create service for transforming a list of GPS coordinates to the
        # specified world-fixed frame. 
        self.create_service(GpsToWorldFrame, "transform_gps_to_world_frame", self._transform_gps_to_world_frame_callback)

        # Create transform broadcaster for UTM-->world_frame. This will
        # effectively just be a static transform that only gets updated when
        # either a.) our datum changes or b.) the UTM grid changes according to
        # our GPS coordinates.
        self._tf_broadcaster = TransformBroadcaster(self)

        # Create GPS fix subscriber.
        self._gps_sub = self.create_subscription(NavSatFix, "gps", self._gps_callback, 10)
        # Create Odometry publisher.
        # NOTE: This should eventually be removed once we have the
        # robot_localization ekf running.
        self._odom_pub = self.create_publisher(Odometry, "odom", 10)

        # Create timer to publish the UTM-->world_frame transform.
        self.create_timer(1.0, self._publish_utm_to_world_frame)


def main(args=None):
    rclpy.init(args=args)
    nav_node = NavNode()
    rclpy.spin(nav_node)
    nav_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
