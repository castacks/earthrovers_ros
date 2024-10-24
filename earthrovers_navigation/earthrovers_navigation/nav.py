
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32
from geographic_msgs.msg import GeoPath
from tf2_ros import TransformBroadcaster
import rclpy
import pyproj
from scipy.spatial.transform import Rotation as R
import numpy as np

def calculate_odometry(initial_gps_coords, current_gps_coords):
    """Calculates the odometry relative to the start GPS coordinates.

    Args:
        current_gps_coords (NavSatFix): The current GPS coordinates.
    """
    if initial_gps_coords is None:
        return None

    # Create a pyproj transformer to project the GPS coordinates.
    transformer = pyproj.Transformer.from_crs("epsg:4326", "epsg:3857", always_xy=True)

    # Project the start GPS coordinates.
    start_x, start_y = transformer.transform(initial_gps_coords.longitude, initial_gps_coords.latitude)

    # Project the current GPS coordinates.
    current_x, current_y = transformer.transform(current_gps_coords.longitude, current_gps_coords.latitude)

    odometry = [current_x-start_x, current_y - start_y]

    return odometry

class NavNode(Node):
    """Node that temporarily creates an odometry message from GPS data. Using in
    place of proper odometry for the time being.
    """

    def __init__(self):
        super().__init__("base")

        # Create GPS subscriber.
        self.create_subscription(NavSatFix, "gps", self.gps_to_odom_callback, 10)

        # Create orientation subscriber.
        self.create_subscription(Float32, "orientation", self.orientation_callback, 10)

        # Create checkpoint GPS subscriber, in callback we calculate the relative position to /odm frame and publish as /odom_waypoints
        self.create_subscription(GeoPath, "checkpoints_gps", self.checkpoint_gps_callback, 10)
        # Create odom publisher.
        self.__odom_publisher = self.create_publisher(Odometry, "odom", 10)

        self.__start_gps_coords = None
        self.orientation_float = None

        # Create a ROS service that provides a list of odometry frame waypoints
        # (converted from the GPS frame to UTM coordinates or directly to odom
        # frame). For now, just going to periodically publish these in place of a
        # service.
        self.__odom_waypoints_publisher = self.create_publisher(PoseArray, "odom_waypoints", 10)
        self.__next_waypoint_publisher = self.create_publisher(PoseArray, "next_waypoint", 10)
        self.get_logger().info("Created Nav Node.")

        # TODO: Create "set-datum" service that sets the start GPS coordinates.

        # TODO: Create a tf broadcaster that broadcasts the transform between
        # the base_link and odom frame.
        self._tf_broadcaster = TransformBroadcaster(self)

    def checkpoint_gps_callback(self, msg: GeoPath):
        """checkpoint_gps_callback

        Args:
            msg (GeoPath): Checkpoint from Frodobot SDK from waypoint_receiver node
        Publishes:
            PoseArray: The converted waypoints in odom frame
        """
        self.get_logger().info(f"Received checkpoint data: {msg}")
        if self.__start_gps_coords is None:
            return
        # Create a PoseArray message to store the waypoints.
        odom_waypoints = PoseArray()
        odom_waypoints.header.stamp = self.get_clock().now().to_msg()
        odom_waypoints.header.frame_id = "odom"

        # Create a pyproj transformer to project the GPS coordinates.
        transformer = pyproj.Transformer.from_crs("epsg:4326", "epsg:3857", always_xy=True)

        # Project the start GPS coordinates.
        start_x, start_y = transformer.transform(self.__start_gps_coords.longitude, self.__start_gps_coords.latitude)

        # For each of the GPS waypoints, project them to the odometry frame and
        # then add them to the PoseArray message.
        for waypoint in msg.poses:
            x, y = transformer.transform(waypoint.pose.position.longitude, waypoint.pose.position.latitude)
            pose = Pose()
            pose.position.x = x - start_x
            pose.position.y = y - start_y
            odom_waypoints.poses.append(pose)

        # Publish the PoseArray message.
        next_waypoint = PoseArray()
        next_waypoint.header = odom_waypoints.header
        next_waypoint.poses = [odom_waypoints.poses[0]]
        self.__next_waypoint_publisher.publish(next_waypoint)
        self.__odom_waypoints_publisher.publish(odom_waypoints)
        self.get_logger().info("Published odometry waypoints.")

    def orientation_callback(self, msg: Float32):
        """Callback for the orientation subscriber. Logs the orientation data to
        the console.

        Args:
            msg (Float32): The orientation data message.
        """
        self.get_logger().info(f"Received orientation data: {msg}")
        self.orientation_float = msg.data

    def gps_to_odom_callback(self, msg: NavSatFix):
        """Callback for the GPS subscriber. Logs the GPS data to the console.

        Args:
            msg (NavSatFix): The GPS data message.
        """
        # self.get_logger().info(f"Received GPS data: {msg}")

        # If we haven't received the start GPS coordinates yet, store them.
        if self.__start_gps_coords is None:
            self.__start_gps_coords = msg
            self.get_logger().info(f"Start GPS coordinates: {msg}")

        if self.orientation_float is not None:
            # Calculate current odometry relative to start GPS coordinates.


            # Call the above calculate_odometry function with the current GPS
            # coordinates and use the result to populate an odometry message.
            current_gps_coords = msg
            odometry = calculate_odometry(self.__start_gps_coords, current_gps_coords)

            # Populate an odometry message with the calculated odometry.
            timestamp = self.get_clock().now().to_msg()
            odometry_msg = Odometry()
            odometry_msg.header.stamp = timestamp
            odometry_msg.header.frame_id = "odom"
            odometry_msg.child_frame_id = "base_link"
            self.get_logger().info(f"Calculated odometry: {odometry}")
            odometry_msg.pose.pose.position.x = odometry[0]
            odometry_msg.pose.pose.position.y = odometry[1]

            # TODO: Populate the orientation field of the odometry message with orientation float
            # Float is between 0 and 360, so we can just use this as the yaw.
            # yaw to quaternion

            # Convert the yaw from degrees to radians
            yaw_radians = np.deg2rad(-1 * self.orientation_float + 90)


            # Create a rotation object for yaw around the Z-axis
            rotation = R.from_euler('z', yaw_radians)

            # Convert to quaternion
            quaternion = rotation.as_quat()
            odometry_msg.pose.pose.orientation.x = quaternion[0]
            odometry_msg.pose.pose.orientation.y = quaternion[1]
            odometry_msg.pose.pose.orientation.z = quaternion[2]
            odometry_msg.pose.pose.orientation.w = quaternion[3]


            # TODO: Subscribe to whatever topic the SDK-provided orientation is
            # given published on, as we can use this for the time being until we
            # have the magnometer data integrated.

            # Publish the odometry message.
            self.__odom_publisher.publish(odometry_msg)

            # Also, publish the base_link to odom transform. The transformation
            # is essentially the exact same as the post computed for the
            # odometry message. Eventually, a node from robot_localization
            # should take this role over or if you have another node fusing
            # various odometry sources.
            t = TransformStamped()
            t.header.stamp = timestamp
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation.x = odometry[0]
            t.transform.translation.y = odometry[1]
            t.transform.translation.z = 0.0
            t.transform.rotation.x = quaternion[0]
            t.transform.rotation.y = quaternion[1]
            t.transform.rotation.z = quaternion[2]
            t.transform.rotation.w = quaternion[3]
            self._tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    nav_node = NavNode()
    rclpy.spin(nav_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()