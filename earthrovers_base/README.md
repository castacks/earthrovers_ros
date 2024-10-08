# earthrovers_base
This package mainly houses the `base` node, which is one of the core nodes
used to interface with the Earth Rovers SDK.

## `base` node
### ROS Topics Subscribed

| ROS Topic | Interface | Description |
| --- | --- | --- |
| `cmd_vel` | [geometry_msgs/Twist](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html) | Linear and Angular velocity command to send to the FrodoBot. |
### ROS Topics Published
| ROS Topic | Interface | Description |
| --- | --- | --- |
| `imu` | [sensor_messages/Imu](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html) | Most recent angular velocity and linear acceleration returned by the Earth Rovers SDK `/data` endpoint. |
| `magnetic_field` | [sensor_messages/MagneticField](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/MagneticField.html) | Most recent magnetic field measurement returned by the Earth Rovers SDK `/data` endpoint. |
| `gps` | [sensor_messages/NavSatFix](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/NavSatFix.html) | The newest GPS latitude and longitude available from the SDK `/data` endpoint. |
| `orientation` | [std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html) | Orientation computed by the SDK from the magnetometer data returned from the `/data` endpoint. |
| `battery` | [sensor_msgs/BatteryState ](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/BatteryState.html) | Battery message with only the percentage is populated from the battery level provided by the `/data` endpoint. |

### Notes/Disclaimers

- Eventually, the orientation field may not need be used, as the magnetic field
  data could be used directly by something like `imu_filter_madgwick` from
  [imu_tools](https://github.com/CCNYRoboticsLab/imu_tools) to produce
  orientation estimates.
- GPS messages are currently ONLY PUBLISHED when a change in the reported
  latitude/longitude is detected. This is done so that multiple, *identical* GPS
  fixes are not sent out as if they were two separate fixes / locations.


