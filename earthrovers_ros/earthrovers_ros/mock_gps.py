
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from rclpy.node import Node
import rclpy
import pyproj

class MockGpsNode(Node):
    def __init__(self):
        super().__init__('mock_gps_node')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps_fix', 10)
        self.timer_ = self.create_timer(1.0, self.publish_gps_fix)
        self.gps_coordinates = [(30.48243713, 114.3026428), # First coord will be our initial GPS position.
                                (30.48268318, 114.3028047), 
                                (30.48283713, 114.3030428),
                                (30.48303713, 114.3032428),
                                (30.48323713, 114.3034428),
                                (30.48343713, 114.3036428)]

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