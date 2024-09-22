"""Note that gets waypoints from the EarthRovers SDK and publishes them as a
GeoPath message. It also uses the SDK response to figure out which waypoint
should be followed next.
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import requests
from geographic_msgs.msg import GeoPoseStamped, GeoPath

class WaypointReceiverNode(Node):
    """Node that receives waypoints from the EarthRovers SDK to check for
    checkpoints and to continuously call checkpoint reached to scan it.
    """

    def __init__(self):
        super().__init__("waypoint_receiver")

        # Make publisher foor GeoPath 
        self.waypoints_pub = self.create_publisher(GeoPath, "checkpoints_gps", 10)

        # Make timers
        get_checkpoint_list_period = 1.0
        check_checkpoint_reached_period = 5.0
        self.create_timer(get_checkpoint_list_period, self.get_checkpoints_list)
        self.create_timer(check_checkpoint_reached_period, self.check_checkpoint_reached)
        self.earthrover_sdk_url = "http://127.0.0.1:8000" 
    
    def get_checkpoints_list(self):
        checkpoints_list = GeoPath()
        latest_scanned_checkpoint = -1
        start_time = time.time()
        try: 
            checkpoints_list_response = requests.get(f"{self.earthrover_sdk_url}/checkpoints-list")
            # Parse the checkpoints from the response and create a list of
            # of Waypoint messages. 
            if checkpoints_list_response.status_code == 200:
                checkpoints_list_json = checkpoints_list_response.json()
                # Get the checkpoints and create waypoint message from each 
                checkpoint_dicts = checkpoints_list_json["checkpoints_list"]
                print("Checkpoint dicts", checkpoint_dicts)
                
                for checkpoint_i, checkpoint_dict in enumerate(checkpoint_dicts):
                    latest_scanned_checkpoint = checkpoints_list_json["latest_scanned_checkpoint"]
                    if checkpoint_i >= latest_scanned_checkpoint:
                        geoposes = GeoPoseStamped()
                        geoposes.pose.position.latitude = float(checkpoint_dict["latitude"])
                        geoposes.pose.position.longitude = float(checkpoint_dict["longitude"])
                        print("Checkpoint: ", checkpoint_dict['id'], geoposes.pose.position.latitude, geoposes.pose.position.longitude)
                        geoposes.pose.position.altitude = 0.0
                        checkpoints_list.poses.append(geoposes)
                        
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Failed to get checkpoints list: {e}")
            raise Exception("Failed to get checkpoints list.")
        print("Published checkpoints list of length: ", len(checkpoints_list.poses))
        print([f"{pose.pose.position.latitude}, {pose.pose.position.longitude}" for pose in checkpoints_list.poses])
        self.waypoints_pub.publish(checkpoints_list)
        
    def check_checkpoint_reached(self):
        start_time = time.time()
        checkpoint_reached_response = requests.post(f"{self.earthrover_sdk_url}/checkpoint-reached", json={})
        checkpoint_reached_response_json = checkpoint_reached_response.json()
        print(f"Checkpoint reached response: {checkpoint_reached_response_json}")
        print(f"Time taken to get checkpoint reached response: {time.time() - start_time}")


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=8)
    waypoint_manager_node = WaypointReceiverNode()
    executor.add_node(waypoint_manager_node)
    executor.spin()
    executor.shutdown()
    waypoint_manager_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
