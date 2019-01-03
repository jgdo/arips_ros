#!/bin/bash

rostopic pub --once /kinect_tilt_deg std_msgs/Float32 "data: 60"
rostopic pub --once /calc_transform_signal std_msgs/Empty "{}" 
sleep 6
rostopic pub --once /kinect_tilt_deg std_msgs/Float32 "data: 35"
rostopic pub --once /calc_transform_signal std_msgs/Empty "{}" 

