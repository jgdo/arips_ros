#pragma once

#include <list>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

class OdometryBuffer {
public:
    struct Entry {
        nav_msgs::Odometry odom;
        geometry_msgs::Twist cmdVel;
    };

    OdometryBuffer();

    void saveBuffer(const std::string& filename);

private:
    std::list<Entry> mBuffer;
    geometry_msgs::Twist mLastCmdVel;
    uint32_t mMarkerIdCount = 0;

    ros::Publisher mForecastCmdVelPub;
    ros::Publisher mMarkerPub;

    ros::Subscriber mOdomSub;
    ros::Subscriber mCmdVelSub;
    ros::Timer mPublishTimer;


    void onOdom(const nav_msgs::Odometry& msg);
    void onCmdVel(const geometry_msgs::Twist& msg);

    void onTimer(const ros::TimerEvent& ev);

    void publishPoint(double x, double y, float r, float g, float b);
};
