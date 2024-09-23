"""Module containing definition for the earthrover's control node responsible
for relaying commanded velocity messages to the Earth Rover SDK.
"""
import time
import requests
import rclpy
from rclpy.time import Time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, MagneticField, NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from transforms3d.euler import euler2quat
# from conversions import degs_to_rads, gs_to_ms2, lsb_to_tesla
from .conversions import degs_to_rads, gs_to_ms2, lsb_to_tesla

# TODO: Update the basenode to be implemented as a LifecycleNode. This would
# allow us to control (via inherited services) when the different parts of this
# node start and stop. Why is this helpful? One example is that we don't want
# the node to begin hitting the /data endpoints until we've started a mission.
# Ideally, according to a behavior tree, if a mission is started successfully,
# we could then call the "on_configure" lifecycle node service to configure this
# node and set up its timer, and then "on_activate" to start the timer and begin
# hitting the /data endpoint.
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
        self.declare_parameter("earthrover_sdk_url", "http://127.0.0.1:8000")
        self.declare_parameter("data_publish_rate_hz", 1.0)
        # http://wiki.sunfounder.cc/images/7/72/QMC5883L-Datasheet-1.0.pdf
        self.declare_parameter("magnetometer_sensitivity_lsb_per_gauss", 3000)

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

        # Create publishers for IMU, Magnetic Field, Odometry, and GPS data.
        self._imu_pub = self.create_publisher(msg_type=Imu, topic="imu", qos_profile=10)
        self._magnetic_field_pub = self.create_publisher(msg_type=MagneticField, topic="magnetic_field", qos_profile=10)
        self._odometry_pub = self.create_publisher(msg_type=Odometry, topic="odom", qos_profile=10)
        self._gps_pub = self.create_publisher(msg_type=NavSatFix, topic="gps", qos_profile=10)
        self._ori_pub = self.create_publisher(msg_type=Float32, topic="orientation", qos_profile=10)

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
        the Earth Rover SDK API to get the most recent sensor data and publishes
        it to their respective topics.

        https://github.com/frodobots-org/earth-rovers-sdk?tab=readme-ov-file#get-data
        for more details on the data endpoint.
        """

        # Hit the data endpoint to get the most recent data.
        earthrover_sdk_url = self.get_parameter("earthrover_sdk_url").get_parameter_value().string_value
        start = time.perf_counter()
        try:
            self.get_logger().debug(f"Making GET request to {earthrover_sdk_url}/data")
            response = requests.get(f"{earthrover_sdk_url}/data")
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Failed to get data: {e}")
            return
        end = time.perf_counter()
        self.get_logger().debug(f"data GET request took {end - start} seconds.")

        # Parse the response JSON.
        try:
            response_json = response.json()
            self.get_logger().debug(f"Data: {response_json}")
        except ValueError as e:
            self.get_logger().error(f"Failed to parse response JSON: {e}")
            return
        except Exception as e:
            self.get_logger().error(f"Unknown error while trying to parse /data response: {e}")
            return

        # TODO: This functionality should not be present in the future if this
        # node re-implemented as a LifecycleNode. However, having a check like
        # this is still helpful in the name of fault tolerance/identification.
        try:
            # Parse the timestamp from the response JSON. This timestamp is
            # assocaited with the GPS coordinates, orientation.
            # NOTE: From Santiago: This is the timestamp at which the SDK REQUESTED
            # THE DATA--not the timestamp at which the data was actually measured.
            # This is *okay* for now, but could cause some unforseen issues down the
            # line--keep in mind if we start seeing weird behavior.
            timestamp = Time(nanoseconds=int(float(response_json["timestamp"]) * 1e9))
        except Exception as e:
            self.get_logger().error(f"Failed to parse timestamp from response JSON: {e}")
            return

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
            "vibration": 0.31,
            "timestamp": 1724189733.208559,
            "accel": [0.604, -0.853,0.076],
            "gyro": [3.595, -3.885,-0.557],
            "mag": [-75, 195,390],
            "rpm": [0, 0, 0, 0]
        }
        """

        # Parse, populate, and publish the IMU data.
        # NOTE: The Earth Rover SDK treats x as up, y as left, and z as
        # backward. Convert to ROS coordinate system where x is forward, y is
        # left, and z is up.
        try:
            imu_msg = Imu()
            latest_accel_sample = response_json["accels"][-1]
            latest_accel_sample_time = Time(nanoseconds=int(float(latest_accel_sample[3]) * 1e9))
            imu_msg.header.stamp = latest_accel_sample_time.to_msg()
            imu_msg.header.frame_id = "imu_link" # TODO: Parameterize this.
            imu_msg.linear_acceleration.x = gs_to_ms2(-latest_accel_sample[2])  # x = -z
            imu_msg.linear_acceleration.y = gs_to_ms2(latest_accel_sample[1])   # y = +y
            imu_msg.linear_acceleration.z = gs_to_ms2(latest_accel_sample[0])   # z = +x
            # TODO: Need linear acceleration covariance data from datasheet? For now,
            # just using nonzero placeholder values.
            imu_msg.linear_acceleration_covariance = [0.0] * 9
            imu_msg.linear_acceleration_covariance[0] = 0.1
            imu_msg.linear_acceleration_covariance[4] = 0.1
            imu_msg.linear_acceleration_covariance[8] = 0.1

            # Set up the angular velocity and its covariance matrix.
            latest_gyro_sample = response_json["gyros"][-1]
            imu_msg.angular_velocity.x = degs_to_rads(-latest_gyro_sample[2]) # x = -z  # TODO: Does this need inverted?
            imu_msg.angular_velocity.y = degs_to_rads(latest_gyro_sample[1]) # y = +y
            imu_msg.angular_velocity.z = degs_to_rads(latest_gyro_sample[0]) # z = +x
            # TODO: Find linear acceleration covariance data from datasheet? For now,
            # just using nonzero placeholder values.
            imu_msg.angular_velocity_covariance = [0.0] * 9
            imu_msg.angular_velocity_covariance[0] = 0.1
            imu_msg.angular_velocity_covariance[4] = 0.1
            imu_msg.angular_velocity_covariance[8] = 0.1
            # NOTE: Each gyro measurement has a slightly different timestamp from
            # the corresponding accelerometer measurement. This might be okay, but
            # maybe the more correct thing to do is to just create and publish a
            # separate IMU message that has empty accel readings and only gyro (but
            # with the correct timestamp).
            self._imu_pub.publish(imu_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to parse/publish IMU data: {e}")

        # Parse, populate, and publish the Magnetic Field data.
        try:
            latest_mag_sample = response_json["mags"][-1]
            magnetic_field_msg = MagneticField()
            latest_mag_sample_time = Time(nanoseconds=int(float(latest_mag_sample[3]) * 1e9))
            magnetic_field_msg.header.stamp = latest_mag_sample_time.to_msg()
            magnetic_field_msg.header.frame_id = "imu_link"
            lsb_per_gauss = self.get_parameter("magnetometer_sensitivity_lsb_per_gauss").get_parameter_value().integer_value
            magnetic_field_msg.magnetic_field.x = lsb_to_tesla(-latest_mag_sample[2], lsb_per_gauss)  # x = -z
            magnetic_field_msg.magnetic_field.y = lsb_to_tesla(latest_mag_sample[1], lsb_per_gauss)   # y = +y
            magnetic_field_msg.magnetic_field.z = lsb_to_tesla(latest_mag_sample[0], lsb_per_gauss)   # z = +x
            # TODO: Find magnetic field covariance data from datasheet? For now,
            # just using nonzero placeholder values.
            magnetic_field_msg.magnetic_field_covariance = [0.0] * 9
            magnetic_field_msg.magnetic_field_covariance[0] = 0.1
            magnetic_field_msg.magnetic_field_covariance[4] = 0.1
            magnetic_field_msg.magnetic_field_covariance[8] = 0.1
            self._magnetic_field_pub.publish(magnetic_field_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to parse/publish Magnetic Field data: {e}")

        # TODO: Write helper function to compute wheel odometry from RPMs.
        # Parse, populate, and publish the Odometry data.
        # Compute skid-steer odometry from each wheel's RPM.

        # Parse, populate, and publish the GPS data.
        # NOTE: Not sure of the consequences of publishing the GPS values
        # multiple times as if they were actually different measurements. This
        # could cause problems for downstream state-estimation systems...look
        # into this more later! My hunch is that a gps message should not be
        # published unless the GPS data from the endpoint has actually changed!
        try:
            gps_msg = NavSatFix()
            gps_msg.header.stamp = timestamp.to_msg()
            gps_msg.header.frame_id = "gps_link"
            gps_msg.latitude = response_json["latitude"]
            gps_msg.longitude = response_json["longitude"]
            # NOTE: Real covariance values not known, experimenting with this to
            # start.
            gps_msg.position_covariance = [0.0] * 9
            gps_msg.position_covariance[0] = 0.1
            gps_msg.position_covariance[4] = 0.1
            gps_msg.position_covariance[8] = 0.1
            gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            self._gps_pub.publish(gps_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to parse/publish GPS data: {e}")

        # Publish the orientation data.
        # NOTE: If we get robot_localization's ekf to work, it might be able to
        # use some kind of sensor model to compute a smoothed orientation from
        # the magnetometer and IMU data. For now, just publishing what is
        # computed by the SDK.
        orientation_msg = Float32()
        orientation_msg.data = float(response_json["orientation"])
        self._ori_pub.publish(orientation_msg)

def main(args=None):
    rclpy.init(args=args)
    base_node = BaseNode()
    rclpy.spin(base_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()