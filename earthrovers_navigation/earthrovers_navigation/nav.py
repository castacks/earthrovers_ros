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
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
from tf2_ros import TransformBroadcaster
from geodesy.utm import fromLatLong, UTMPoint
from geographic_msgs.msg import GeoPoint

from earthrovers_interfaces.srv import GpsToUtm, GpsToWorldFrame

def utm_to_world(current_utm_coords: UTMPoint,
                 gps_datum: GeoPoint) -> Point:
    """Express a given UTM position in the specified frame.

    Args:
        utm_coords (UTM): UTM coordinates to transform.

    Returns:
        Point: UTM position expressed in the frame specified by the datum.
    """
    # This is computed by taking the current position in the UTM frame (obtained
    # from the most recent GPS fix) and subtracting the datum position as
    # expressed in the UTM frame, as the datum defines where the origin of the
    # odometry frame is.
    # First, compute the UTM coordinates of the datum.
    datum_utm_coords = fromLatLong(gps_datum.latitude, gps_datum.longitude)
    # Now, subtract the datum from the current UTM coordinates to get the
    # position in the world frame.
    world_coords = Point()
    world_coords.x = current_utm_coords.easting - datum_utm_coords.easting
    world_coords.y = current_utm_coords.northing - datum_utm_coords.northing
    world_coords.z = current_utm_coords.altitude - datum_utm_coords.altitude
    return world_coords

def gps_to_world(current_gps_coords: GeoPoint,
                 gps_datum: GeoPoint) -> Point:
    """Express a given GPS position in the world-fixed frame whose origin is at
    the datum

    Args:
        current_gps_coords (GeoPoint): GPS coordinates to transform.
        gps_datum (GeoPoint): GPS coordinates that define the origin of the
            world-fixed frame.
    
    Returns:
        Point: GPS position expressed in the world-fixed frame.
    """
    # Compute this by first converting the current GPS coordinates to UTM, and
    # then computing the world-fixed frame position as the difference between
    # the current UTM coordinates and the datum UTM coordinates (computed by
    # utm_to_world).
    current_utm_coords = fromLatLong(current_gps_coords.latitude, current_gps_coords.longitude)
    return utm_to_world(current_utm_coords, gps_datum)

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
        self._gps_datum = None
        # self._utm_datum = None
        # Actually, we should compute this UTM datum value on the fly, as if we
        # end up in a different UTM grid zone by our GPS coordinates changing,
        # this value will now need to be expressed from the perspective of the
        # new UTM grid frame.

        # However, unless that gps_datum changes on the fly, then the
        # fromLatLong function of geodesy will always just be computing the same
        # value. However, I guess we are supporting the case where the datum
        # might be changed on the fly.

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
        self._gps_fix = None
        self._gps_sub = self.create_subscription(NavSatFix, "gps", self._gps_callback, 10)
        # Create orientation subscriber.
        # NOTE: This should also be removed, as the ekf should ultimately
        # provide us with a heading estimate based on magnetometer and IMU data.
        self._orientation = None
        self.create_subscription(Float32, "orientation", self._orientation_callback, 10)
        # Create Odometry publisher.
        # NOTE: This should eventually be removed once we have the
        # robot_localization ekf running.
        self._odom_pub = self.create_publisher(Odometry, "odom", 10)

        # Create timer to publish the UTM-->world_frame transform.
        self.create_timer(1.0, self._publish_utm_to_world_frame)

    def _set_datum_callback(self, request, response):
        pass

    def _set_datum_to_gps_fix_callback(self, request, response):
        """Set the datum to the most recently received GPS fix."""
        if self._gps_fix is None:
            self.get_logger().warn("Cannot set datum to GPS fix because no GPS fix has been received yet.")
            response.success = False
            response.message = "No GPS fix received yet."
            return response
        self._gps_datum = GeoPoint()
        self._gps_datum.latitude = self._gps_fix.latitude
        self._gps_datum.longitude = self._gps_fix.longitude
        self._gps_datum.altitude = self._gps_fix.altitude
        response.success = True
        response.message = f"Datum set to most recent GPS fix: {self._gps_datum}"
        return response

    def _transform_gps_to_utm_callback(self, request, response):
        pass

    def _transform_gps_to_world_frame_callback(self, request, response):
        """Transform a list of GPS coordinates to the specified world-fixed
        frame."""
        

    def _publish_utm_to_world_frame(self):
        """Publish the UTM-->world_frame transform. This will be a static
        transform that only gets updated when either a.) our datum changes or
        b.) the UTM grid changes according to our GPS coordinates.
        """

        # If we don't have a datum, we can't publish the transform.
        if self._datum is None:
            self.get_logger().warn("Cannot publish UTM-->world_frame transform because datum is not yet set.")
            return
        
        # If we do have the datum, once we compute where this lands in the UTM
        # frame, we basically have the UTM-->world_frame transform (as the datum
        # is what we subtract from the current state estimate in the world
        # frame).

    def _orientation_callback(self, orientation_msg: Float32):
        """Callback for the orientation subscriber. Will take the orientation
        message and store it for use in the Odometry message.
        """
        self._orientation = orientation_msg

    def _gps_callback(self, gps_msg):
        """Callback for the GPS fix subscriber. Will take the GPS fix and create
        an Odometry message from it.
        """
        self._gps_fix = gps_msg

        # NOTE: In the future, pretty much everything below should be removed,
        # as we won't need to publish Odometry messages from this node.

        # Also, for now, we will use the latest orientation we receive as our
        # Odometry estimate yaw.
        if self._orientation is None:
            self.get_logger().warn("No orientation received yet. Cannot publish Odometry message.")
            return

        # Express the GPS coordinates in the world-fixed frame.
        gps_point = GeoPoint()
        gps_point.latitude = gps_msg.latitude
        gps_point.longitude = gps_msg.longitude
        gps_point.altitude = gps_msg.altitude
        world_coords = gps_to_world(gps_point, self._gps_datum)

        # Create Odometry message from the UTM coordinates.
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = world_coords.x
        odom_msg.pose.pose.position.y = world_coords.y
        odom_msg.pose.pose.position.z = world_coords.z
        # For now, we will use the orientation we receive as our heading
        # estimate.
        odom_msg.pose.pose.orientation.z = self._orientation.data
        self._odom_pub.publish(odom_msg)


def main(args=None):

    rclpy.init(args=args)
    nav_node = NavNode()
    rclpy.spin(nav_node)
    nav_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
