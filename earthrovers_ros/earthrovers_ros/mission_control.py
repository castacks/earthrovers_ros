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
        self._checkpoint_reached_srv = self.create_service(CheckpointReached, "checkpoint_reached", self._checkpoint_reached_callback)
        
    def _start_mission_callback(self, _, response) -> StartMission.Response:
        """Callback function for the start_mission service. Sends an HTTP POST
        request to the Earth Rover SDK API to start a mission.

        Args:
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
        # https://github.com/frodobots-org/earth-rovers-sdk?tab=readme-ov-file#post-start-mission
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
    
    def _end_mission_callback(self, _, response) -> EndMission.Response:
        """Callback function for the end_mission service. Sends an HTTP POST
        request to the Earth Rover SDK API to end a mission.

        Args:
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

        # Attempt to parse the end mission endpoint response.
        if post_response.status_code == 200:
            try:
                response_data = post_response.json()
                response_message = response_data["message"]
                if response_message == "Mission ended successfully":
                    self.get_logger().info("Mission ended successfully")
                    response.mission_ended = True
                    response.message = "Mission ended successfully"
                else:
                    self.get_logger().error(f"Failed to end mission; Unknown Error. Response Message: {response_message}")
                    response.mission_ended = False
                    response.message = f"Failed to end mission; Unknown Error. Response Message: {response_message}"
            except Exception as e:
                self.get_logger().error(f"Failed to parse end mission response: {e}")
                response.mission_ended = False
                response.message = f"Failed to parse end mission response: {e}"
        else:
            self.get_logger().error(f"Failed to end mission; Unknown Error. Response Code: {post_response.status_code}, Response Text: {post_response.text}")
            response.mission_ended = False
            response.message = f"Failed to end mission with unknown error. Response Code: {post_response.status_code}, Response Text: {post_response.text}"

        return response
    
    def _checkpoint_reached_callback(self, _, response) -> CheckpointReached.Response:
        """Callback function for the checkpoint_reached service. Sends an HTTP POST
        request to the Earth Rover SDK API to notify that a checkpoint has been
        reached.

        Args:
            response (CheckpointReached.Response): The response object.

        Returns:
            CheckpointReached.Response: The response object.
        """

        # Hit the checkpoint_reached endpoint.
        earthrover_sdk_url = self.get_parameter("earthrover_sdk_url").get_parameter_value().string_value
        start = time.perf_counter()
        try:
            self.get_logger().info("Making request to checkpoint reached.")
            self.get_logger().debug(f"Making POST request to {earthrover_sdk_url}/checkpoint-reached")
            post_response = requests.post(f"{earthrover_sdk_url}/checkpoint-reached", json={})
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Failed to check if checkpoint reached with error: {e}")
            response.checkpoint_reached = False
            response.approximate_distance_to_checkpoint = -1.0
            response.message = f"Failed to check if checkpoint reached with error: {e}"
            return response
        end = time.perf_counter()
        self.get_logger().debug(f"checkpoint_reached POST request took {end - start} seconds.")

        # Attempt to parse the checkpoint reached endpoint response.
        # According to the SDK README for version 4.3, a Code 200 response
        # indicates the checkpoint was reached successfully, while a Code 400
        # indicates the checkpoint was not reached.
        if post_response.status_code == 200:
            self.get_logger().info("Checkpoint reached successfully.")
            response.checkpoint_reached = True
            response.approximate_distance_to_checkpoint = 0.0
            response.message = "Checkpoint reached successfully"
        elif post_response.status_code == 400:
            response.checkpoint_reached = False
            # Try to parse the approximate distance to the checkpoint provided
            # in the response message.
            try:
                response_data = post_response.json()
                response_message = response_data["detail"]
                response.message = response_message["error"]
                approximate_distance_to_checkpoint = float(response_message["proximate_distance_to_checkpoint"])
                response.approximate_distance_to_checkpoint = approximate_distance_to_checkpoint
                self.get_logger().info(f"Bot is not within range of the current checkpoint. Approximate distance to checkpoint: {approximate_distance_to_checkpoint}")
            except Exception as e:
                self.get_logger().error(f"Bot is not within range of the current checkpoint. Failed to parse approximate distance to checkpoint from response: {e}")
                response.approximate_distance_to_checkpoint = -1.0
                response.message = "Bot is not within range of the current checkpoint. Failed to parse approximate distance to checkpoint from response."
        else:
            self.get_logger().error(f"Failed to check if checkpoint reached; Unknown Error. Response Code: {post_response.status_code}, Response Text: {post_response.text}")
            response.checkpoint_reached = False
            response.approximate_distance_to_checkpoint = -1.0
            response.message = f"Failed to check if checkpoint reached with unknown error. Response Code: {post_response.status_code}, Response Text: {post_response.text}"
        
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