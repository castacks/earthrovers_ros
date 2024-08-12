"""Module containing definition for the earthrover's control node responsible
for relaying commanded velocity messages to the Earth Rover SDK.
"""
import time
import requests
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from sensor_msgs.msg import BatteryState

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

        # Create a subscriber for cmd_vel Twist messages. Will parse these and
        # hit the control endpoint with the normalized values.
        self._cmd_vel_sub = self.create_subscription(msg_type=Twist,
                                                     topic="cmd_vel",
                                                     callback=self._cmd_vel_callback,
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

def main(args=None):
    rclpy.init(args=args)
    base_node = BaseNode()
    rclpy.spin(base_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()