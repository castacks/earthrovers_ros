# earthrovers_navigation
This package mainly houses the `nav` and `waypoint_receiver` nodes. The `nav`
node handles GPS-->UTM-->ODOM transformations, as well as publishing the "fake"
GPS-based odometry. The `waypoint_receiver` node gets mission checkpoints and
checks which checkpoints have been reached via the SDK.

This package is home to a few other nodes (`experimental_nav`,
`mission_controller`, and `waypoint_manager`) that are not finished and
should not be relied on.

## `nav` node
Node responsible for GPS-->UTM-->odom transformations, as well as publishing the
GPS based odometry. This note is sort of like the [`navsat_transform_node`](https://docs.ros.org/en/noetic/api/robot_localization/html/navsat_transform_node.html) from
the robot_localization package. It takes the first GPS location it receives,
projects it into the closest UTM grid zone, and uses that as its "datum" == the
UTM location that coincides with the robot's odometry frame origin. It can then
use this to express/resolve additional GPS locations/fixes in the odometry
frame, like the GPS checkpoint locations provided by the SDK. See
https://wiki.ros.org/geodesy and
https://docs.ros.org/en/noetic/api/robot_localization/html/integrating_gps.html#details
to learn more.


In the future, this node could/should be replaced by the navsat_transform_node
and ekf_localization_node provided by the
[robot_localization](https://docs.ros.org/en/noetic/api/robot_localization/html/index.html)
package.
### ROS Topics Subscribed

| ROS Topic | Interface | Description |
| --- | --- | --- |
| `gps` | [sensor_msgs/NavSatFix](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/NavSatFix.html) | GPS / fix location of the rover in [the WGS 84 frame](https://en.wikipedia.org/wiki/World_Geodetic_System). |
| `orientation` | [std_msgs/Float32](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html) | Orientation of the rover relative to its world-fixed odometry frame. |
| `checkpoints_gps` | [geographic_msgs/GeoPath](https://docs.ros.org/en/melodic/api/geographic_msgs/html/msg/GeoPath.html) | List of mission checkpoints expressed in the WGS 84 frame. I.e., GPS mission checkpoints returned from SDK. |

### ROS Topics Published
| ROS Topic | Interface | Description |
| --- | --- | --- |
| `odom` | [nav_msgs/Odometry](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) | Odometry estimated from consecutive GPS fixes and magnetometer-based orientation. |
| `odom_waypoints` | [geometry_msgs/PoseArray](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseArray.html) | List of mission checkpoints resolved in the rover's world-fixed odometry frame. I.e., this is a list of GPS-checkpoints that the node receives from its `checkpoints_gps` subscription and resolves them in the odometry frame via the WGS84-->UTM-->odom transform. |
| `next_waypoint` | [geometry_msgs/PoseArray](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseArray.html) | The next consecutive waypoint to navigate to from the list of mission checkpoints/waypoints. |

### Transforms Published
| Parent Frame | Child Frame | Description |
| --- | --- | --- |
| odom | base_link | A transformation from the `base_link` to `odom` frame simply based on the most recent gps-derived odometry estimate. |

### Notes/Disclaimers

- This node also publishes Odometry messages based on the change in consecutive
  GPS  positions and a magnetometer-derived orientation. It is important to note
  that this is not really a typical way to compute odometry, but is instead just a
  quick and dirty solution to get some robot poses in the odometry frame from
  the readily-available GPS data provided by the SDK.
- Long term, this node should **really not be publishing the base_link-->odom
  transform**--instead, this probably be done by some type of filtering node
  (like the
  [ekf_localization_node](https://docs.ros.org/en/noetic/api/robot_localization/html/state_estimation_nodes.html#ekf-localization-node))
  in order that whatever transform is being published is going through some kind
  of smoothing first. This is probably more necessary when you have multiple
  odometry sources that you want to reconcile/combine to create an overall
  better odometry estimate.



