"""Module defining the a node that provides mission services."""

import time
import requests
import rclpy
from rclpy.node import Node

from earthrovers_interfaces.srv import StartMission, EndMission, GetCheckpoints, CheckpointReached

class MissionControl(Node):
    """Node that defines a set of services for interacting with the Earth Rovers
    SDK mission endpoints.
    """

    def __init__(self):
        super().__init__("mission_control")

        # Declare parameters.
        self.declare_parameter("earthrover_sdk_url", "http://127.0.0.1:8000")

        # Create mission management services.
        self._start_mission_srv = self.create_service(StartMission, "start_mission", self._start_mission_callback)
        self._end_mission_srv = self.create_service(EndMission, "end_mission", self._end_mission_callback)
        # self._checkpoint_list_srv = self.create_service(GetCheckpoints, "get_checkpoints", self._get_checkpoints_callback)
        # self._checkpoint_reached_srv = self.create_service(CheckpointReached,
        # "checkpoint_reached", self._checkpoint_reached_callback)
        
    def _start_mission_callback(self, request, response) -> StartMission.Response:
        """Callback function for the start_mission service. Sends an HTTP POST
        request to the Earth Rover SDK API to start a mission.

        Args:
            request (StartMission.Request): The request object. This is not used
            for this service and is simply a placeholder. TODO: Can we remove
            this? 
            response (StartMission.Response): The response object.

        Returns:
            StartMission.Response: The response object.
        """

        # Hit the start_mission endpoint.
        earthrover_sdk_url = self.get_parameter("earthrover_sdk_url").get_parameter_value().string_value
        start = time.perf_counter()
        try:
            self.get_logger().info("Making request to start mission.")
            self.get_logger().debug(f"Making POST request to {earthrover_sdk_url}/start_mission")
            post_response = requests.post(f"{earthrover_sdk_url}/start-mission")
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Failed to start mission: {e}")
            response.mission_started = False
            response.message = f"Failed to start mission: {e}"
            return response
        end = time.perf_counter()
        self.get_logger().debug(f"start_mission POST request took {end - start} seconds.")

        # According to the SDK README for version 4.3, a Code 200 response
        # implies a successful mission start, while a Code 400 response means
        # that the mission could not be started as the bot was unavailable.
        if post_response.status_code == 200:
            self.get_logger().info("Mission started successfully.")
            response.mission_started = True
            response.message = "Mission started successfully."
        elif post_response.status_code == 400:
            self.get_logger().error("Failed to start mission; Bot Unavailable. Response Code: 400")
            response.mission_started = False
            response.message = "Failed to start mission; Bot Unavailable. Response Code: 400"
        else:
            self.get_logger().error(f"Failed to start mission; Unknown Error. Response Code: {post_response.status_code}, Response Text: {post_response.text}")
            response.mission_started = False
            response.message = f"Failed to start mission with unknown error. Response Code: {post_response.status_code}, Response Text: {post_response.text}"

        return response
    
    def _end_mission_callback(self, request, response) -> EndMission.Response:
        """Callback function for the end_mission service. Sends an HTTP POST
        request to the Earth Rover SDK API to end a mission.

        Args:
            request (EndMission.Request): The request object. This is not used
            for this service and is simply a placeholder. TODO: Can we remove
            this? 
            response (EndMission.Response): The response object.

        Returns:
            EndMission.Response: The response object.
        """

        # Hit the end_mission endpoint.
        earthrover_sdk_url = self.get_parameter("earthrover_sdk_url").get_parameter_value().string_value
        start = time.perf_counter()
        try:
            self.get_logger().info("Making request to end mission.")
            self.get_logger().debug(f"Making POST request to {earthrover_sdk_url}/end_mission")
            post_response = requests.post(f"{earthrover_sdk_url}/end-mission")
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Failed to end mission: {e}")
            response.mission_ended = False
            response.message = f"Failed to end mission: {e}"
            return response
        end = time.perf_counter()
        self.get_logger().debug(f"end_mission POST request took {end - start} seconds.")

        # According to the SDK README for version 4.3, a Code 200 response
        # implies a successful mission end, while a Code 400 response means
        # that the mission could not be ended as the bot was unavailable.
        if post_response.status_code == 200:
            self.get_logger().info("Mission ended successfully.")
            response.mission_ended = True
            response.message = "Mission ended successfully."
        elif post_response.status_code == 400:
            self.get_logger().error("Failed to end mission; Bot Unavailable. Response Code: 400")
            response.mission_ended = False
            response.message = f"Failed to end mission; Bot Unavailable. Response Code: 400"
        else:
            self.get_logger().error(f"Failed to end mission; Unknown Error. Response Code: {post_response.status_code}, Response Text: {post_response.text}")
            response.mission_ended = False
            response.message = f"Failed to end mission with unknown error. Response Code: {post_response.status_code}, Response Text: {post_response.text}"

        return response
    
def main(args=None):
    """Main entry point for the mission services node."""
    rclpy.init(args=args)
    mission_control_node = MissionControl()
    rclpy.spin(mission_control_node)
    mission_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()