# """Module with helper functions to parse sensor data from the Earth Rover SDK
# API and create the appropriate ROS messages.
# """
# import rclpy
# from rclpy.time import Time
# from sensor_msgs.msg import Imu

# def parse_imu_data(accel_data: dict,
#                    gyro_data: dict) -> Imu:
#     """Parses IMU data from the Earth Rover SDK API into a ROS Imu message.

#     Args:
#         imu_data (dict): The IMU data from the Earth Rover SDK API.

#     Returns:
#         Imu: The parsed IMU data as a ROS Imu message.
#     """
#     imu_msg = Imu()
#     imu_msg.header.stamp = Time(nanoseconds=)
#     imu_msg.header.frame_id = "base_link"