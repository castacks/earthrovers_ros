

import base64
import time
import requests
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

import numpy as np
import cv2
from cv_bridge import CvBridge

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
        self.declare_parameter("front_camera_params_filepath", "config/front_camera_params.yaml")
        self.declare_parameter("rear_camera_params_filepath", "config/rear_camera_params.yaml")

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

        # Create a timer to periodically get camera images and publish them.
        # Because we get all camera images at the same time, we'll just hit the
        # endpoint at the maximum framerate between the front and rear cameras.
        max_framerate = max(self.get_parameter("front_camera_framerate").get_parameter_value().integer_value,
                            self.get_parameter("rear_camera_framerate").get_parameter_value().integer_value)
        self._screenshots_timer = self.create_timer(timer_period_sec=1.0 / max_framerate,
                                                    callback=self._get_and_publish_camera_images)

    def _get_and_publish_camera_images(self) -> None:

        # Hit the screenshots endpoint to get the most recent camera frames.
        earthrover_sdk_url = self.get_parameter("earthrover_sdk_url").get_parameter_value().string_value
        start = time.perf_counter()
        try:
            response = requests.get(f"{earthrover_sdk_url}/screenshot")
            response.raise_for_status()
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Failed to get camera images: {e}")
            return
        end = time.perf_counter()
        self.get_logger().debug(f"screenshots GET request took {end - start} seconds.")

        # Parse the response JSON to get the camera frames and timestamp.
        response_json = response.json()

        # TODO: Need to figure out how to translate this UTC timestamp from the
        # SDK into ROS time--if that's even the goal.
        
        timestamp_sec = response_json["timestamp"]

        # Prepare and publish the front camera frame.
        front_video_frame_b64 = response_json["front_video_frame"]
        nparr = np.fromstring(base64.b64decode(front_video_frame_b64), np.uint8)
        front_video_frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        bridge = CvBridge()
        front_camera_msg = bridge.cv2_to_imgmsg(front_video_frame, encoding="bgr8")
        # TODO: POTENTIAL PROBLEM: While we do have the UTC time in seconds, we
        # do not receive any time in nano seconds. Not sure if that's a standard
        # thing or not--but the point is, multiple image frames that are taken
        # in the same second will end up with the same timestamps.
        front_camera_msg.header.stamp.sec = timestamp_sec
        # TODO: Parameterize this frame_id.
        front_camera_msg.header.frame_id = "front_camera"
        self._front_camera_pub.publish(front_camera_msg)

        # Prepare and publish the rear camera frame.
        rear_video_frame = response_json["rear_video_frame"]
        nparr = np.fromstring(base64.b64decode(rear_video_frame), np.uint8)
        rear_video_frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        rear_camera_msg = bridge.cv2_to_imgmsg(rear_video_frame, encoding="bgr8")
        rear_camera_msg.header.stamp.sec = timestamp_sec
        rear_camera_msg.header.frame_id = "rear_camera"
        self._rear_camera_pub.publish(rear_camera_msg)

        # Prepare and publish the map image.
        map_frame = response_json["map_frame"]
        nparr = np.fromstring(base64.b64decode(map_frame), np.uint8)
        map_frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        map_msg = bridge.cv2_to_imgmsg(map_frame, encoding="bgr8")
        map_msg.header.stamp.sec = timestamp_sec
        self._map_image_pub.publish(map_msg)

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()