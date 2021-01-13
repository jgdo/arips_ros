//
// Created by jgdo on 1/3/21.
//

#pragma once

#include <memory>

#include <costmap_2d/costmap_2d_ros.h>

#include <toponav_ros/TopoPlannerROS.h>
#include <nav_core/base_local_planner.h>

class Navigation {
public:
    Navigation();

    void poseCallbackNavGoal(const geometry_msgs::PoseStamped &msg);

    void timerCallback(const ros::TimerEvent& e);

private:
    enum class State {
        IDLE,
        NAVIGATING
    };

    State mState = State::IDLE;

    tf2_ros::Buffer m_tfBuffer;
    tf2_ros::TransformListener tfListener {m_tfBuffer};
    
    costmap_2d::Costmap2DROS m_LocalCostmap {"local_costmap", m_tfBuffer};
    toponav_ros::TopoPlannerROS m_TopoPlanner;

    std::unique_ptr<nav_core::BaseLocalPlanner> mLocalPlanner;

    ros::Publisher mCmdVelPub;
    ros::Subscriber psub_nav;

    ros::Timer mControlTimer;
};
