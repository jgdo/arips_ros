* Install ROS noetic
* Install setup catkin workspace
* Install catkin_tools
* install sshfs
* copy over .catkin_config
* git clone https://github.com/jgdo/geduino-ros.git, checkout master
* copy over /etc/ros/env.sh
* copy over /usr/sbin/roslaunch, adjust path inside
* copy over /etc/systemd/system/roscore.service and roslaunch.service
* copy over /etc/udev/rules.d/... for kinect, arduino, battery_controller, md25, scs215, ydlidar
* adjust .bashrc
    alias shutdown='sudo shutdown now'

    source /home/jgdo/catkin_ws/install/setup.bash

    export ROS_HOSTNAME=arips
    export ROS_WORKSPACE=/home/jgdo/catkin_ws

* Setup chained catkin workspace
    * source ros setup.bash
    * source catkin_ws/install_desktop/setup.bash
    * catkin build

    This will make sure that packages from install_desktop/ are visible
* Install libraries for Kinect https://aibegins.net/2020/11/22/give-your-next-robot-3d-vision-kinect-v1-with-ros-noetic/
* copy over and install OpenNI (https://robots.uc3m.es/installation-guides/install-openni-nite.html), and also SensorKinect hw driver
* install cuda: https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=20.04&target_type=deb_local
* build opencv with cuda: https://gist.github.com/raulqf/f42c718a658cddc16f9df07ecc627be7
