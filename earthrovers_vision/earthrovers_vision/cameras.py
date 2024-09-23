"""Node that periodically gets camera images from the Earth Rover SDK and
publishes them to their respective topic. Also publishes monocular camera
calibration parameters if specified.
"""

import base64
import time
import requests
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from rclpy.time import Time

import numpy as np
import cv2
from cv_bridge import CvBridge

from ament_index_python.packages import get_package_share_directory

def get_camera_params(filepath: str) -> CameraInfo:
    """Parses camera parameters from a YAML file and returns a CameraInfo
    message populated with those parameters.
    """
    camera_info = CameraInfo()
    with open(filepath, "r") as f:
        camera_params = yaml.safe_load(f)
        camera_info.width = camera_params["image_width"]
        camera_info.height = camera_params["image_height"]
        camera_info.distortion_model = camera_params["distortion_model"]
        camera_info.d = camera_params["distortion_coefficients"]["data"]
        camera_info.k = camera_params["camera_matrix"]["data"]
        camera_info.r = camera_params["rectification_matrix"]["data"]
        camera_info.p = camera_params["projection_matrix"]["data"]

    return camera_info

class CameraNode(Node):
    """Node that periodically gets camera images from the Earth Rover SDK and
    publishes them to their respective topic.
    """
    def __init__(self):
        super().__init__("camera")

        # Declare camera node parameters.
        self.declare_parameter("earthrover_sdk_url", "http://localhost:8000")
        self.declare_parameter("front_camera_framerate", 10)
        self.declare_parameter("rear_camera_framerate", 10)
        self.declare_parameter("front_camera_params_filepath", "/earthrovers_ws/src/earthrovers_ros/config/camera_calibration/front_camera.yaml")

        # Create publishers for front camera messages.
        self._front_camera_pub = self.create_publisher(msg_type=Image,
                                                       topic="front_camera/image_raw",
                                                       qos_profile=10)
        self._front_camera_info_pub = self.create_publisher(msg_type=CameraInfo,
                                                            topic="front_camera/camera_info",
                                                            qos_profile=10)

        # Create publishers for rear camera messages.
        self._rear_camera_pub = self.create_publisher(msg_type=Image,
                                                      topic="rear_camera/image_raw",
                                                      qos_profile=10)
        self._rear_camera_info_pub = self.create_publisher(msg_type=CameraInfo,
                                                           topic="rear_camera/camera_info",
                                                           qos_profile=10)

        # Create publisher for map image provided along with the camera images.
        self._map_image_pub = self.create_publisher(msg_type=Image,
                                                    topic="map_image",
                                                    qos_profile=10)

        # Create a timer for each camera to periodically get and publish images
        # from that camera (or the map screenshot endpoint).
        self._front_camera_timer = self.create_timer(timer_period_sec=1.0 / self.get_parameter("front_camera_framerate").get_parameter_value().integer_value,
                                                     callback=self._get_and_publish_front_camera_image)
        self._rear_camera_timer = self.create_timer(timer_period_sec=1.0 / self.get_parameter("rear_camera_framerate").get_parameter_value().integer_value,
                                                    callback=self._get_and_publish_rear_camera_image)
        # self._map_image_timer = self.create_timer(timer_period_sec=1.0 / 10,
        #                                           callback=self._get_and_publish_map_image)

        # Try to parse the front camera's calibration parameters.
        front_camera_params_filepath = f"{get_package_share_directory('earthrovers_vision')}/front_camera.yaml"
        try:
            self._front_camera_info = get_camera_params(front_camera_params_filepath)
        except Exception as e:
            self.get_logger().error(f"Failed to parse the front camera's calibration parameters from the provided file {front_camera_params_filepath} {e}")
            raise e

    def _get_and_publish_front_camera_image(self) -> None:
        """Hits the screenshot endpoint and requests only the front camera
        image.
        """
        earth_rover_sdk_url = self.get_parameter("earthrover_sdk_url").get_parameter_value().string_value
        start = time.perf_counter()
        try:
            response = requests.get(f"{earth_rover_sdk_url}/screenshot?view_types=front")
            response.raise_for_status()
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Failed to get front camera image: {e}")
            return
        end = time.perf_counter()
        self.get_logger().debug(f"Front camera screenshot GET request took {(end - start) * 1000:.0f} milliseconds.")

        # Parse the response JSON to get the camera frame and timestamp.
        response_json = response.json()

        # Prepare and publish the front camera frame.
        front_video_frame_b64 = response_json["front_frame"]
        image_as_array = np.fromstring(base64.b64decode(front_video_frame_b64), np.uint8)
        front_video_frame = cv2.imdecode(image_as_array, cv2.IMREAD_COLOR)
        bridge = CvBridge()
        front_camera_msg = bridge.cv2_to_imgmsg(front_video_frame, encoding="bgr8")
        # Grab the timestamp from the response JSON, convert it to nanoseconds,
        # create a new Time instance, and initialize the header's stamp with it.
        # NOTE: These timestamps are STILL NOT THE TIMESTAMP THAT THE IMAGE
        # FRAME ORIGINATED FROM THE ROBOT BASE. Instead, they are the time at
        # which the server took the screenshot. Waiting on an update to the SDK
        # to return the latency between the robot and the server so we can
        # compute its approximate acquisition time.
        timestamp = Time(nanoseconds=response_json["timestamp"] * 1e9)
        front_camera_msg.header.stamp = timestamp.to_msg()
        front_camera_msg.header.frame_id = "front_camera"
        self._front_camera_pub.publish(front_camera_msg)

        # Publish the front camera parameters.
        self._front_camera_info.header.stamp = timestamp.to_msg()
        self._front_camera_info.header.frame_id = "front_camera"
        self._front_camera_info_pub.publish(self._front_camera_info)

    def _get_and_publish_rear_camera_image(self) -> None:
        """Hits the screenshot endpoint and requests only the rear camera
        image.
        """
        earth_rover_sdk_url = self.get_parameter("earthrover_sdk_url").get_parameter_value().string_value
        start = time.perf_counter()
        try:
            response = requests.get(f"{earth_rover_sdk_url}/screenshot?view_types=rear")
            response.raise_for_status()
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Failed to get rear camera image: {e}")
            return
        end = time.perf_counter()
        self.get_logger().debug(f"Rear camera screenshot GET request took {(end - start) * 1000:.0f} milliseconds.")

        # Parse the response JSON to get the camera frame and timestamp.
        response_json = response.json()

        # Prepare and publish the rear camera frame.
        rear_video_frame_b64 = response_json["rear_frame"]
        image_as_array = np.fromstring(base64.b64decode(rear_video_frame_b64), np.uint8)
        rear_video_frame = cv2.imdecode(image_as_array, cv2.IMREAD_COLOR)
        bridge = CvBridge()
        rear_camera_msg = bridge.cv2_to_imgmsg(rear_video_frame, encoding="bgr8")
        # Grab the timestamp from the response JSON, convert it to nanoseconds,
        # create a new Time instance, and initialize the header's stamp with it.
        # NOTE: These timestamps are STILL NOT THE TIMESTAMP THAT THE IMAGE
        # FRAME ORIGINATED FROM THE ROBOT BASE. Instead, they are the time at
        # which the server took the screenshot. Waiting on an update to the SDK
        # to return the latency between the robot and the server so we can
        # compute its approximate acquisition time.
        timestamp = Time(nanoseconds=response_json["timestamp"] * 1e9)
        rear_camera_msg.header.stamp = timestamp.to_msg()
        rear_camera_msg.header.frame_id = "rear_camera"
        self._rear_camera_pub.publish(rear_camera_msg)

        # # Publish the rear camera parameters.
        # self._rear_camera_info.header.stamp = timestamp.to_msg()
        # self._rear_camera_info.header.frame_id = "rear_camera"
        # self._rear_camera_info_pub.publish(self._rear_camera_info)

    def _get_and_publish_map_image(self) -> None:
        """Hits the screenshot endpoint and requests only the map image.
        """
        earth_rover_sdk_url = self.get_parameter("earthrover_sdk_url").get_parameter_value().string_value
        start = time.perf_counter()
        try:
            response = requests.get(f"{earth_rover_sdk_url}/screenshot?view_types=map")
            response.raise_for_status()
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Failed to get map image: {e}")
            return
        end = time.perf_counter()
        self.get_logger().debug(f"Map screenshot GET request took {(end - start) * 1000:.0f} milliseconds.")

        # Parse the response JSON to get the camera frame and timestamp.
        response_json = response.json()

        # Prepare and publish the map image.
        map_image_b64 = response_json["map_frame"]
        image_as_array = np.fromstring(base64.b64decode(map_image_b64), np.uint8)
        map_image = cv2.imdecode(image_as_array, cv2.IMREAD_COLOR)
        bridge = CvBridge()
        map_image_msg = bridge.cv2_to_imgmsg(map_image, encoding="bgr8")
        # Grab the timestamp from the response JSON, convert it to nanoseconds,
        # create a new Time instance, and initialize the header's stamp with it.
        # NOTE: These timestamps are STILL NOT THE TIMESTAMP THAT THE IMAGE
        # FRAME ORIGINATED FROM THE ROBOT BASE. Instead, they are the time at
        # which the server took the screenshot. Waiting on an update to the SDK
        # to return the latency between the robot and the server so we can
        # compute its approximate acquisition time.
        timestamp = Time(nanoseconds=response_json["timestamp"] * 1e9)
        map_image_msg.header.stamp = timestamp.to_msg()
        map_image_msg.header.frame_id = "map"
        self._map_image_pub.publish(map_image_msg)

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()