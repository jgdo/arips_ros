#pragma once

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>

class NavigationContext {
public:
    tf2_ros::Buffer tf;

private:
    // It is important that the listener is instantiated before the costmaps, otherwise
    // they will stuck forever waiting for the transforms
    tf2_ros::TransformListener mTfListener{tf};

public:
    costmap_2d::Costmap2DROS globalCostmap{"global_costmap", tf};
    costmap_2d::Costmap2DROS localCostmap{"local_costmap", tf};

    explicit NavigationContext();

    void publishCmdVel(const geometry_msgs::Twist& cmd_vel);

private:
    ros::Publisher mCmdVelPub;
};
