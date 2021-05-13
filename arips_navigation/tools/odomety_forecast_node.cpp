#include <ros/ros.h>

#include <arips_navigation/odometry/OdometryBuffer.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry_forecast");

    OdometryBuffer odomBuffer;

    while(ros::ok()) {
        ros::spin();
    }
}

