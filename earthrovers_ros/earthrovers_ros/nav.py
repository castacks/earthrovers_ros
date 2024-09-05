
from geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32
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
        # Create odom publisher.
        self.__odom_publisher = self.create_publisher(Odometry, "odom", 10)

        self.__start_gps_coords = None
        self.orientation_float = None

        # Create a ROS service that provides a list of odometry frame waypoints
        # (converted from the GPS frame) as 
        # For now, because I don't know how rosbridge handles service
        # definitions, I'm going to just publish the converted waypoints
        # continuously and the consumer can grab them as many times as needed.
        # Not permanent, just for now.
        self.__odom_waypoints_publisher = self.create_publisher(PoseArray, "odom_waypoints", 10)
        self.__odom_waypoints_timer = self.create_timer(1.0, self.publish_odom_waypoints)
        self.get_logger().info("Created Nav Node.")
        
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
            odometry_msg = Odometry()
            odometry_msg.header.stamp = self.get_clock().now().to_msg()
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
            

    def publish_odom_waypoints(self):
        """Publishes a list of odometry frame waypoints (converted from the GPS
        frame) as PoseArray messages.
        """
        if self.__start_gps_coords is None:
            return
        
        # TODO: Get these from a service in the base node. FOR NOW, we are just
        # hardcoding these waypoints. Just paste these from the result of
        # hitting the checkpoint-list endpoint in the SDK API.
        gps_waypoint_json = {"checkpoints_list":[{"id":6075,"sequence":1,"latitude":"10.03320789","longitude":"-84.21801758"},{"id":6076,"sequence":2,"latitude":"10.033322","longitude":"-84.217094"},{"id":6077,"sequence":3,"latitude":"10.03320789","longitude":"-84.21801758"}],"latest_scanned_checkpoint":0}
        gps_waypoints = [(float(waypoint["latitude"]), float(waypoint["longitude"])) for waypoint in gps_waypoint_json["checkpoints_list"]]

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
        for waypoint in gps_waypoints:
            x, y = transformer.transform(waypoint[1], waypoint[0])
            pose = Pose()
            pose.position.x = x - start_x
            pose.position.y = y - start_y
            odom_waypoints.poses.append(pose)

        # Publish the PoseArray message.
        self.__odom_waypoints_publisher.publish(odom_waypoints)
        self.get_logger().info("Published odometry waypoints.")
        
def main(args=None):
    rclpy.init(args=args)
    nav_node = NavNode()
    rclpy.spin(nav_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()