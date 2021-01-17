//
// Created by jgdo on 1/3/21.
//

#pragma once

#include <memory>

#include <costmap_2d/costmap_2d_ros.h>
#include <toponav_ros/TopoPlannerROS.h>
#include <arips_navigation/TopoExecuter.h>

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

    TopoExecuter m_TopoExec {m_tfBuffer, m_LocalCostmap};

    ros::Subscriber psub_nav;

    ros::Timer mControlTimer;
};
