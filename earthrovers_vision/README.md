# earthrovers_vision
This package mainly houses the `cameras` node, which is essentially just a node
for grabbing front+rear camera image frames and publishing them to their
respective topics.

## `cameras` node
### ROS Topics Subscribed
### ROS Topics Published
| ROS Topic | Interface | Description |
| --- | --- | --- |
| `front_camera/image_raw` | [sensor_messages/Image](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html) | Most recent front camera frame returned by the Earth Rovers SDK `/screenshot` endpoint. |
| `front_camera/camera_info` | [sensor_messages/CameraInfo](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html) | Front camera parameters. |
| `rear_camera/image_raw` | [sensor_messages/Image](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html) | Most recent rear camera frame returned by the Earth Rovers SDK `/screenshot` endpoint. |
| `rear_camera/camera_info` | [sensor_messages/CameraInfo](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html) | Rear camera parameters. |

### Notes/Disclaimers

- The camera_info being published is most definitely inaccurate / wrong. Those
  parameters are mostly being used as placeholders, and should be updated with
  new parameters obtained via calibration (perhaps by following the ["How to
  Calibrate a Monocular
  Camera"](https://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration)
  tutorial).
- Right now, the front and rear camera images are being queried via two separate
  HTTP `GET` requests to the SDK's `/screenshot` endpoint. However, because
  these requests are happening **within the same thread, the requests are
  sequential**. The upshot of this is that acquiring images takes longer in
  total and both the front and rear camera images get published at a lower rate
  than what might be possible. A **future improvement** would be to use a
  [MultiThreadedExecutor](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html#types-of-executors)
  to parallelize these io-bound, high-latency requests (io-bound, as the SDK
  currently writes the images to disk).


