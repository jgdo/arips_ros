//
// Created by jgdo on 1/3/21.
//

#pragma once

#include <costmap_2d/costmap_2d_ros.h>

#include <toponav_ros/TopoPlannerROS.h>

class Navigation {
public:
    Navigation();

    void poseCallbackNavGoal(const geometry_msgs::PoseStamped &msg);

private:
    tf2_ros::Buffer m_tfBuffer;
    tf2_ros::TransformListener tfListener {m_tfBuffer};
// costmap_2d::Costmap2DROS m_GlobalMap {"global", m_tfBuffer};

    toponav_ros::TopoPlannerROS m_TopoPlanner;

    ros::Subscriber psub_nav;
};
