# ROS battery controller node for ARIPS battery controller

Connects to battery device serial port, can read battery state and turn on/off robot's power.

## Subscribed topics
``battery_command`` (std_msgs/Bool)  
Turn on/off power to robot.

## Published topics
``battery_state`` (sensor_msgs/BatteryState)  
Battery state

## Parameters
``~battery_device`` (string, default: "/dev/arips_battery_controller"):  
Path to battery controller device.

``~battery_on`` (bool, default: "false"):   
Whether robot power should be turned on/off on node startup.
