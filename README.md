# waypoint based 2D navigation
2D waypoints will be predefined in ROS-based robots to navigate to the destination avoiding obstacles. A package that will buffer `move_base` goals until instructed to navigate to all waypoints in sequence.

<div align="center">
  <img src="media/waypoint_nav.png" width="600">
</div>

## Installation
Install the binaries needed for the package.
```
sudo apt-get install ros-melodic-follow-waypoints
```

#### The code can be run in this way:

```
rosrun follow_waypoints follow_waypoints.py
```

#### A wait_duration parameter. This sets wait duration in between waypoints. The default value is set to 0.0 sec.

```
rosparam set wait_duration 5.0
```

#### A distance threshold parameter. Issue the next goal target if the robot reaches within this distance. This has the effect of smoothing motion and not stopping at each waypoint. The default value is set to 0.0 distance which disables the feature.

```
rosparam set waypoint_distance_tolerance 0.5
```

#### Following waypoints will save the list of poses to a file in the following directory:

```
follow_waypoints/saved_path/pose.csv
```

#### To load the previously save path:

```
rostopic pub /start_journey std_msgs/Empty -1
```

![follow_waypoints](readme_images/follow_waypoint.gif "rviz")

## Acknowledgement
We sincerely thank the developers and contributors of [Follow waypoints by daniel snider](https://github.com/danielsnider/follow_waypoints), [Follow waypoints by SLAMCore](https://github.com/slamcore/follow_waypoints), [Follow waypoints by Qbotics Lab](https://github.com/qboticslabs/follow_waypoints), [Follow waypoints by Sugbuv](https://github.com/sugbuv/follow_waypoints), [Follow waypoints ROS Tutorial](http://wiki.ros.org/follow_waypoints), [Follow waypoints in ROS2 Tutorial](https://navigation.ros.org/configuration/packages/configuring-waypoint-follower.html) and [Neo Goal Sequence Driver](https://github.com/neobotix/neo_goal_sequence_driver)
