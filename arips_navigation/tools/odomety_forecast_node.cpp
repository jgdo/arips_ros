#include <ros/ros.h>

#include <arips_navigation/odometry/OdometryForecaster.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry_forecast");

    OdometryForecaster odomBuffer;

    while(ros::ok()) {
        ros::spin();
    }
}

