# ROS battery controller node for ARIPS battery controller

Connects to battery device serial port, can read battery state and turn on/off robot's power.

## Parameters
``~battery_device`` (string, default: "/dev/arips_battery_controller"):  
Path to battery controller device.

``~battery_on`` (bool, default: "false"):   
Whether battery should be turned on/off on node startup.
