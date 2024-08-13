"""Module containing definition for the earthrover's control node responsible
for relaying commanded velocity messages to the Earth Rover SDK.
"""
import time
import requests
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from transforms3d.euler import euler2quat

class BaseNode(Node):
    """Node that subscribes to Twist messages on the cmd_vel topic and sends an
    HTTP POST request with the provided angular and linear velocities.
    """
    
    def __init__(self):
        super().__init__("base")

        # Per https://shop.frodobots.com/products/earthroverzero, the maximum
        # speed is ~0.94 m/s.
        self.declare_parameter("max_speed_ms", 0.94)
        # TODO: Need to determine this value. Guessing 1.57 rad/s for now.
        self.declare_parameter("max_angular_speed_rads", 1.57)
        self.declare_parameter("earthrover_sdk_url", "http://localhost:8000")
        self.declare_parameter("data_publish_rate_hz", 10.0)

        # Create a subscriber for cmd_vel Twist messages. Will parse these and
        # hit the control endpoint with the normalized values.
        self._cmd_vel_sub = self.create_subscription(msg_type=Twist,
                                                     topic="cmd_vel",
                                                     callback=self._cmd_vel_callback,
                                                     qos_profile=10)
        
        # Create a timer for periodically hitting /data endpoint for gps, imu,
        # battery, and other data.
        self._data_timer = self.create_timer(timer_period_sec=1.0 / self.get_parameter("data_publish_rate_hz").get_parameter_value().double_value,
                                             callback=self._get_and_publish_data)

        # Create publishers for the battery state, GPS, and odometry data.
        self._battery_state_pub = self.create_publisher(msg_type=BatteryState,
                                                        topic="battery_state",
                                                        qos_profile=10)
        self._gps_pub = self.create_publisher(msg_type=NavSatFix,
                                              topic="gps",
                                              qos_profile=10)
        self._odometry_pub = self.create_publisher(msg_type=Odometry,
                                                   topic="odom",
                                                   qos_profile=10)

    def _cmd_vel_callback(self, twist_msg: Twist) -> None:
        """Callback function for the cmd_vel topic. Receives a Twist message,
        normalizes the angular and linear velocities, and sends the normalized
        values to the Earth Rover SDK API via an HTTP POST request.

        Args:
            twist_msg (Twist): The received Twist message.
        """

        max_linear_vel_x = self.get_parameter("max_speed_ms").get_parameter_value().double_value
        min_linear_vel_x = -max_linear_vel_x
        max_angular_vel_z = self.get_parameter("max_angular_speed_rads").get_parameter_value().double_value
        min_angular_vel_z = -max_angular_vel_z

        # For the frodobots skid-steer platform, only considering the
        # x-component of the commanded linear velocity and the z-component
        # (yaw-rate) of the commanded angular velocity. Additionally, both the
        # linear and angular velocities are to be between [-1, 1] per the SDK:
        # https://github.com/frodobots-org/earth-rovers-sdk?tab=readme-ov-file#post-control
        # To convert the linear and angular values from m/s in the Twist message
        # to be in the [-1, 1] scale, we perform min-max normalization on the
        # linear and angular velocities. See the following link for the formula:
        # https://en.wikipedia.org/wiki/Feature_scaling#Rescaling_(min-max_normalization)
        # We also clamp the normalized values in case the received twist
        # messages exceed the maximum or minimum values velocities in m/s.
        SCALE_MIN = -1.0
        SCALE_MAX = 1.0
        normalized_linear_vel_x = min(SCALE_MIN + (twist_msg.linear.x - min_linear_vel_x)*(SCALE_MAX - SCALE_MIN) / (max_linear_vel_x - min_linear_vel_x), SCALE_MAX)
        normalized_angular_vel_z = min(SCALE_MIN + (twist_msg.angular.z - min_angular_vel_z)*(SCALE_MAX - SCALE_MIN) / (max_angular_vel_z - min_angular_vel_z), SCALE_MAX)
        self.get_logger().debug(f"NORM LINEAR VEL: {normalized_linear_vel_x}, NORM ANGULAR VEL: {normalized_angular_vel_z}")

        # Send the normalized linear and angular velocities to the SDK API.
        # First, grab the SDK API URL from the parameters.
        sdk_url = self.get_parameter("earthrover_sdk_url").get_parameter_value().string_value
        # Next, create the JSON payload to send to the SDK API.
        payload = {
            "command": {
                "linear": normalized_linear_vel_x,
                "angular": normalized_angular_vel_z
            }
        }
        # Finally, send the POST request to the SDK API.
        start = time.perf_counter()
        try:
            response = requests.post(f"{sdk_url}/control", json=payload)
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"POST request to SDK API failed: {e}")
            return
        else:
            end = time.perf_counter()
            self.get_logger().debug(f"POST response: {response.text}")
            self.get_logger().debug(f"POST request took {end - start} seconds.")
    
    # TODO: In the future, it may be wise to create a timer callback function
    # that sends the most recently received twist message to the SDK API at a
    # fixed rate, so as to prevent upstream nodes flooding the SDK API with more
    # messages than it can handle. Could also serve as a safe-guard, where if no
    # twist messages are received for a certain period of time, we could then
    # just request 0 velocities from the SDK API to stop the rover as a
    # failsafe. Could then also organize that timer callback to be implemented
    # as a state machine.

    def _get_and_publish_data(self) -> None:
        """Callback function for the data_timer. Sends an HTTP GET request to
        the Earth Rover SDK API to get the most recent data and publishes it to
        their respective topics.

        https://github.com/frodobots-org/earth-rovers-sdk?tab=readme-ov-file#get-data
        for more details on the data endpoint.
        """

        # Hit the data endpoint to get the most recent data.
        earthrover_sdk_url = self.get_parameter("earthrover_sdk_url").get_parameter_value().string_value
        start = time.perf_counter()
        try:
            response = requests.get(f"{earthrover_sdk_url}/data")
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Failed to get data: {e}")
            return
        end = time.perf_counter()
        self.get_logger().debug(f"data GET request took {end - start} seconds.")

        # Parse the response JSON.
        response_json = response.json()
        self.get_logger().debug(f"Data: {response_json}")

        """Example response:
        {
            "battery": 100,
            "signal_level": 5,
            "orientation": 128,
            "lamp": 0,
            "speed": 0,
            "gps_signal": 31.25,
            "latitude": 22.753774642944336,
            "longitude": 114.09095001220703,
            "vibration": 0.31
        }
        """

        # Publish the battery state.
        battery_state_msg = BatteryState()
        battery_state_msg.percentage = int(response_json["battery"]) / 100.0
        battery_state_msg.header.stamp = self.get_clock().now().to_msg()
        self._battery_state_pub.publish(battery_state_msg)

        # Publish the GPS data.
        gps_msg = NavSatFix()
        gps_msg.latitude = response_json["latitude"]
        gps_msg.longitude = response_json["longitude"]
        self._gps_pub.publish(gps_msg)

        # Populate and publish an Odometry message with the provided speed and
        # orientation.
        odometry_msg = Odometry()
        # AGAIN, need a timestamp for when these were measured.
        # odometry_msg.header.stamp = self.get_clock().now().to_msg()
        # TODO: parameterize these frame_ids.
        odometry_msg.header.frame_id = "odom"
        odometry_msg.child_frame_id = "base_link"
        odometry_msg.twist.twist.linear.x = float(response_json["speed"])
        # Convert the orientation from degrees to radians.
        orientation_rads = float(response_json["orientation"]) * (3.14159 / 180.0)
        # Convert the orientation from radians to a quaternion.
        orientation_quat = euler2quat(0, 0, orientation_rads)
        odometry_msg.pose.pose.orientation = Quaternion(x=orientation_quat[1],
                                                        y=orientation_quat[2],
                                                        z=orientation_quat[3],
                                                        w=orientation_quat[0])
        self._odometry_pub.publish(odometry_msg)

def main(args=None):
    rclpy.init(args=args)
    base_node = BaseNode()
    rclpy.spin(base_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()