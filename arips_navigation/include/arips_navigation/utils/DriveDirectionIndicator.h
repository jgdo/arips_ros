#pragma once

#include <ros/publisher.h>
#include <geometry_msgs/Twist.h>

class DriveDirectionIndicator {
public:
    DriveDirectionIndicator(const std::string& name, double robotRadius);

    void indicate(const geometry_msgs::Twist& twist);


private:
    ros::Publisher mPub;
    double mRobotRadius;
};

