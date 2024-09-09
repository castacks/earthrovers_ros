"""Node for figuring out which waypoint(s) should be tracked by a downstream
motion planner.

NOTE: This whole node should not be necessary once we have a behavior tree to
implement this functionality.
"""

import rclpy
from rclpy.node import Node

class WaypointManagerNode(Node):
    """Node that defines a set of services for interacting with the Earth Rovers
    SDK mission endpoints.
    """

    def __init__(self):
        super().__init__("mission_controller")

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
    waypoint_manager_node = WaypointManagerNode()
    rclpy.spin(waypoint_manager_node)
    waypoint_manager_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
