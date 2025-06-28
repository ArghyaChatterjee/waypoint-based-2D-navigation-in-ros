# Waypoint-Based 2D Navigation for ROS Robots

This ROS package enables robots to autonomously navigate through predefined 2D waypoints while effectively avoiding obstacles. It integrates seamlessly with the ROS `move_base` framework, buffering navigation goals and sequentially dispatching them upon command.

<div align="center">
  <img src="media/waypoint_nav_2d.gif" alt="Waypoint Navigation" width="800">
</div>

## Features

* **Waypoint Buffering:** Queue multiple navigation goals.
* **Sequential Navigation:** Automatically navigate through waypoints in order.
* **Obstacle Avoidance:** Utilize ROS `move_base` to avoid obstacles effectively.
* **Customizable Parameters:** Control waypoint wait duration and waypoint arrival tolerance.

---

## Repository Workflow

<div align="center">
  <img src="media/waypoint_nav.png" alt="Waypoint Navigation" width="800">
</div>

---

## Create Binary Map from Image File
Most of the time, the data is in the form of tiled image file. You can see in the [ArcGIS](https://pro.arcgis.com/en/pro-app/latest/help/data/imagery/raster-data-display-decisions.html) website for more info.

<div align="center">
  <img src="maps/mapa100-binary_raster_res0.0744m_org-start600x320pix.png" alt="Waypoint Navigation" width="400">
  <img src="maps/mapa100-binary_raster_res0.0744m_org-start600x320pix_inv.png" alt="Waypoint Navigation" width="400">
</div>

First, convert the tiled map to image file with color inverted. 
```
cd scripts
python3 convert_tiff_to_png.py
```
Then convert the image file to binary bit map.
```
python3 convert_png_to_bitmap.py
```

## Installation

First, install 2D laser odometry package from one of my [repo](https://github.com/ArghyaChatterjee/rf2o_laser_odometry). 

<div align="center">
  <img src="media/waypoint_nav.png" alt="Waypoint Navigation" width="800">
</div>

Install binary package using ROS package manager:

```bash
sudo apt-get install ros-melodic-follow-waypoints
```

---

## Usage

Launch the waypoint follower:

```bash
rosrun follow_waypoints follow_waypoints.py
```

### Parameters

#### Wait Duration Between Waypoints

Controls the pause duration between reaching one waypoint and navigating to the next. Default is `0.0` seconds (no wait).

```bash
rosparam set wait_duration 5.0
```

#### Waypoint Distance Tolerance

Triggers the robot to move to the next waypoint once it is within this threshold distance, resulting in smoother continuous navigation. Default is `0.0` (feature disabled).

```bash
rosparam set waypoint_distance_tolerance 0.5
```

---

## Saving and Loading Waypoints

The list of waypoints is automatically saved to:

```bash
follow_waypoints/saved_path/pose.csv
```

### Start Navigation with Previously Saved Waypoints

Publish to the `/start_journey` topic to initiate navigation using the previously saved waypoints:

```bash
rostopic pub /start_journey std_msgs/Empty -1
```

![follow_waypoints](readme_images/follow_waypoint.gif "rviz")

## Acknowledgement
We sincerely thank the developers and contributors of , [Follow waypoints by SLAMCore](https://github.com/slamcore/follow_waypoints), [Follow waypoints by Qbotics Lab](https://github.com/qboticslabs/follow_waypoints), [Follow waypoints by daniel snider](https://github.com/danielsnider/follow_waypoints), [Follow waypoints by Sugbuv](https://github.com/sugbuv/follow_waypoints), [Follow waypoints ROS Tutorial](http://wiki.ros.org/follow_waypoints), [Follow waypoints in ROS2 Tutorial](https://navigation.ros.org/configuration/packages/configuring-waypoint-follower.html) and [Neo Goal Sequence Driver](https://github.com/neobotix/neo_goal_sequence_driver).
