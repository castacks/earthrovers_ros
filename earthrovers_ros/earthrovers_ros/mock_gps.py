
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from rclpy.node import Node
import rclpy
import pyproj

class MockGpsNode(Node):
    def __init__(self):
        super().__init__('mock_gps_node')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps', 10)
        self.timer_ = self.create_timer(1.0, self.publish_gps_fix)
        # self.gps_coordinates = [(30.48243713, 114.3026428), # First coord will be our initial GPS position.
        #                         (30.48268318, 114.3028047), 
        #                         (30.48283713, 114.3030428),
        #                         (30.48303713, 114.3032428),
        #                         (30.48323713, 114.3034428),
        #                         (30.48343713, 114.3036428)]
        
        """
        gps_waypoint_json = {"checkpoints_list":[{"id":697,"sequence":1,"latitude":"30.48239","longitude":"114.3026275635"},
                                                 {"id":698,"sequence":2,"latitude":"30.48272","longitude":"114.3026434"},
                                                 {"id":699,"sequence":3,"latitude":"30.48239","longitude":"114.30266"},
                                                 {"id":700,"sequence":4,"latitude":"30.48272","longitude":"114.30268"},
                                                 {"id":701,"sequence":5,"latitude":"30.48239","longitude":"114.3028"},
                                                 {"id":702,"sequence":6,"latitude":"30.48272","longitude":"114.3029"},
                                                 {"id":703,"sequence":7,"latitude":"30.48239","longitude":"114.303"}],
        """
        checkpoints = [(30.48239,114.3026275635), 
                        (30.48272,114.3026434),
                        (30.48239,114.30266),
                        (30.48272,114.30268),
                        (30.48239,114.3028),
                        (30.48272,114.3029),
                        (30.48239,114.303)]
        # Make a second version of the above list that contains the original gps
        # coordinate waypoints, but with five interpolated waypoints between
        # each pair of waypoints.
        interpolated_checkpoints = []
        for i in range(len(checkpoints) - 1):
            start = checkpoints[i]
            end = checkpoints[i + 1]
            for j in range(6):
                interpolated_checkpoints.append((
                    start[0] + j * (end[0] - start[0]) / 5,
                    start[1] + j * (end[1] - start[1]) / 5
                ))
        self.gps_coordinates = interpolated_checkpoints

    def publish_gps_fix(self):
        self.get_logger().info('Publishing GPS coordinates.')
        if self.gps_coordinates:
            latitude, longitude = self.gps_coordinates.pop(0)
            self.gps_coordinates.append((latitude, longitude))
            gps_fix = NavSatFix()
            gps_fix.latitude = latitude
            gps_fix.longitude = longitude
            # print out latitude and longitude before publishing.
            self.get_logger().info(f'Latitude: {latitude}, Longitude: {longitude}')
            self.publisher_.publish(gps_fix)
        else:
            self.get_logger().info('No more GPS coordinates to publish.')
            self.timer_.cancel()

def main(args=None):
    rclpy.init(args=args)
    mock_gps_node = MockGpsNode()
    rclpy.spin(mock_gps_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()