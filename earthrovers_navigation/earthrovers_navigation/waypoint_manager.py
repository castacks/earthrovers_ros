"""Node for figuring out which waypoint(s) should be tracked by a downstream
motion planner.

NOTE: This whole node should not be necessary once we have a behavior tree to
implement this functionality.
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray

from earthrovers_interfaces.srv import GetCheckpoints, GpsToWorldFrame

class WaypointManagerNode(Node):
    """Node that defines a set of services for interacting with the Earth Rovers
    SDK mission endpoints.
    """

    def __init__(self):
        super().__init__("waypoint_manager")

        # Create Reentrant callback group. Allows callbacks to be run in
        # parallel in different threads.
        # TODO: Will it be necessary to create separate callback groups to
        # separate the subscriber callbacks and service-related callbacks?
        self._reentrant_callback_group = ReentrantCallbackGroup()

        # Create odom subscriber.
        self._odom_sub = self.create_subscription(Odometry, "odom", self._odom_callback, 10, callback_group=self._reentrant_callback_group)

        # Create PoseArray publisher that we'll use to publish the waypoints
        # MPPI should navigate to / create a trajectory to.
        self._waypoint_pub = self.create_publisher(PoseArray, "next_waypoints", 10, callback_group=self._reentrant_callback_group)

        # Create timer to call the waypoint manager state machine at a fixed
        # rate.
        self.create_timer(1.0, self._waypoint_manager_state_machine, callback_group=self._reentrant_callback_group)

        # Initialize waypoint list to empty.
        self._waypoints = []

        # Initialize next waypoint to None.
        self._next_waypoint = None

        # Create service clients.
        self._get_checkpoints_client = self.create_client(GetCheckpoints, "get_checkpoints", callback_group=self._reentrant_callback_group)
        self._gps_to_world_frame_client = self.create_client(GpsToWorldFrame, "transform_gps_to_utm", callback_group=self._reentrant_callback_group)
        
        # TODO: Hit the service to get the GPS waypoint list.
        self._get_checkpoints_client.wait_for_service()
        checkpoints_future = self._get_checkpoints_client.call_async(GetCheckpoints.Request())
        while not checkpoints_future.done():
            time.sleep(0.5)
            self.get_logger().info("Waiting for checkpoints...")
        # Check the service result and store the received checkpoints.
        checkpoints_response = checkpoints_future.result()
        if checkpoints_response.success:
            self._waypoints = checkpoints_response.checkpoints
            self.get_logger().info(f"Received {len(self._waypoints)} waypoints: {self._waypoints}")
        else:
            self.get_logger().error("Failed to receive waypoints.")
            # TODO: In future, if implemented as a lifecycle node, could move
            # into a failure state. For now, raise an exception.
            raise Exception("Failed to receive waypoints.")

        # TODO: Hit the service to transform the GPS waypoints to the odom
        # frame.
        self._gps_to_world_frame_client.wait_for_service()
        self._transform_checkpoints_future = self._gps_to_world_frame_client.call_async(GpsToWorldFrame.Request())
        while not self._transform_checkpoints_future.done():
            time.sleep(0.5)
            self.get_logger().info("Waiting for transformed checkpoints...")
        # Check the service result and store the transformed checkpoints.
        transformed_checkpoints_response = self._transform_checkpoints_future.result()
        if transformed_checkpoints_response.success:
            self._waypoints = transformed_checkpoints_response.checkpoints
            self.get_logger().info(f"Transformed waypoints: {self._waypoints}")
        else:
            self.get_logger().error("Failed to transform waypoints.")
            raise Exception("Failed to transform waypoints.")
        
        # There must be something fundamentally incorrect about my understanding
        # of how you are supposed to call/use services in ROS2 nodes. Every
        # example I can find for a client is strictly to call a service once and
        # then terminate. At the very least, for running that checkpoint
        # service, we need to be able to do this repeatedly. Sure, I could hit
        # the endpoint from this node, but then what's the point of creating ROS
        # services??? For now, I think I am just going to synchronously call
        # these services.

        # Update: Apparently, not alone in this:
        # https://answers.ros.org/question/343279/ros2-how-to-implement-a-sync-service-client-in-a-node/
        # For now, using this guy's answer:
        # https://answers.ros.org/answers/354547/revisions/
        

    # What we roughly need from this node:
    # 1. On boot, needs to hit a service and get a waypoint list.
    # 2. Needs to hit a service from nas to transform those GeoPoints (GPS
    #    coordinates) to UTM coordinates (or maybe the service can internally
    #    use its UTM-->Odom transform to return the odom coordinates).

    # Then, as far as what the runtime waypoint manager should look like:
    # - Eventually, this should be deprecated an dreplaced with a behavior tree.
    #   In the meantime, maybe make this a very basic state machine, where a
    #   "next_state" function is called at each tick of a timer we set up.

    # Outputs:
    # - next_waypoint: The next waypoint that we should be tracking and that we
    #   will publish (as a PoseArray) for MPPI to solve for a trajectory to.

    # - States:
    #   - no_waypoints: If the waypoint list is empty, we should just idle.
    #   - waypoints_received: If our waypoint list is no longer empty,
    #     transition to this state. In this state, we will set the "next
    #     waypoint" output to the first waypoint that we (pop) from the list.
    #    - reached_next_waypoint: reached_next_waypoint comes back true,
    #      transition to this state and CALL THE SERVICE to check if we're
    #      actually in range. If sdk_checkpoint_reached comes back true and our
    #      waypoint list is not empty, transition to waypoints_received.
    #   - driving_to_waypoint: If we are not in range of the next waypoint, we
    #     just continue to publish this waypoint as the next waypoint to track
    #     and let MPPI do its thing.
    
    # Guard condition helper functions / inputs we check:
    # - Create a helper function called "reached_next_waypoint" that LOCALLY
    #   checks if we are within range of the next waypoint.
    # - Function to REMOTELY check if we are actually in range. I.e., hit the
    #   endpoint and make sure it also says we are in range.
    #   "sdk_checkpoint_reached"
    # - Function to check if our waypoint list is empty or not.
        


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=8)
    waypoint_manager_node = WaypointManagerNode()
    executor.add_node(waypoint_manager_node)
    executor.spin()
    executor.shutdown()
    waypoint_manager_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
