# Kinect and robot arm calibration scripts

## get_robot_marker_pose.py

To be used while robot is operating and precise kinect pose is required.

Computes compound marker location from 4 individual markers:
- /kinect_link -> /marker_robot from /ar_marker_10..13
- /kinect_link -> /marker_floor from /ar_marker_42..45

## calibrate_kinect_to_robot_base.py

Print transform between "/arips_base" and "/marker_robot" frames. 
"/arips_base" is derived from "/marker_floor" on the calibration floor plate.

Requires on-board calibration plate and floor plate to be visible inside kinect's view.
