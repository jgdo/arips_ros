//
// Created by jgdo on 1/3/21.
//

#pragma once

#include <memory>

#include <arips_navigation/NavigationContext.h>

// #include <arips_navigation/AutoDocker.h>
#include <arips_navigation/CostsPlanners.h>
#include <arips_navigation/CrossDoor.h>
#include <arips_navigation/DriveTo.h>
#include <arips_navigation/DrivingState.h>
#include <arips_navigation/OpenDoor.h>
#include <arips_navigation/TopoExecuter.h>
#include <arips_navigation/local_planner/HPNav.h>
#include <toponav_ros/TopoPlannerROS.h>
#include <arips_navigation/DriveUntilCollision.h>

#include <arips_navigation/path_planning/Locomotion.h>

class Navigation {
public:
    Navigation();

private:
    NavigationContext mContext;

    Locomotion mLocomotion;

    toponav_ros::AripsFlatPlanner mAripsPlanner{mContext, mLocomotion};

    toponav_ros::TopoPlannerROS m_TopoPlanner;

    DrivingState* mDrivingState = nullptr;

    ros::Publisher mActivePub;

    std::unique_ptr<TopoExecuter> m_TopoExec;
    // AutoDocker mAutoDocker{m_LocalCostmap, mCmdVelPub};
    //  HPNav mHPNav{&m_tfBuffer, mCmdVelPub};
    DriveUntilCollision mDriveUntilCollision {mContext};
    OpenDoor mOpenDoor{mContext, mDriveUntilCollision};
    std::unique_ptr<DriveTo> mDriveTo;
    std::unique_ptr<CrossDoor> mCrossDoor;

    ros::Subscriber psub_nav, hp_sub, clicked_sub, door_info_sub;

    ros::Timer mControlTimer;

    void poseCallbackNavGoal(const geometry_msgs::PoseStamped& msg);
    // void poseCallbackHpGoal(const geometry_msgs::PoseStamped& msg);
    void onClickedPoint(const geometry_msgs::PointStamped& point);
    void onDoorInfoReceived(const arips_navigation::CrossDoorInformation& doorInfo);

    void timerCallback(const ros::TimerEvent& e);
};
