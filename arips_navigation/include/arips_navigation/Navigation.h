//
// Created by jgdo on 1/3/21.
//

#pragma once

#include <memory>

#include <costmap_2d/costmap_2d_ros.h>
#include <toponav_ros/TopoPlannerROS.h>
#include <arips_navigation/DrivingState.h>
#include <arips_navigation/TopoExecuter.h>
#include <arips_navigation/AutoDocker.h>
#include <arips_navigation/local_planner/HPNav.h>
#include <arips_navigation/OpenDoor.h>

class Navigation {
public:
    Navigation();

private:
    tf2_ros::Buffer m_tfBuffer;
    tf2_ros::TransformListener tfListener {m_tfBuffer};
    
    costmap_2d::Costmap2DROS m_LocalCostmap {"local_costmap", m_tfBuffer};
    toponav_ros::TopoPlannerROS m_TopoPlanner;

    DrivingState* mDrivingState = nullptr;

    ros::Publisher mCmdVelPub;
    
    TopoExecuter m_TopoExec {m_tfBuffer, m_LocalCostmap, mCmdVelPub};
    AutoDocker mAutoDocker{ m_LocalCostmap, mCmdVelPub };
    HPNav mHPNav {&m_tfBuffer, mCmdVelPub};
    OpenDoor mOpenDoor {m_tfBuffer, mCmdVelPub};
     
    ros::Subscriber psub_nav, hp_sub, clicked_sub;

    ros::Timer mControlTimer;

    void poseCallbackNavGoal(const geometry_msgs::PoseStamped &msg);
    void poseCallbackHpGoal(const geometry_msgs::PoseStamped &msg);
    void onClickedPoint(const geometry_msgs::PointStamped& point);

    void timerCallback(const ros::TimerEvent& e);
};
