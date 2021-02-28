//
// Created by jgdo on 2/28/21.
//

#pragma once

#include "DrivingState.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <arips_navigation/local_planner/HPNav.h>

class OpenDoor: public DrivingStateProto
{
public:
    OpenDoor(tf2_ros::Buffer& tf, ros::Publisher& cmdVelPub, HPNav& nav);

    void init(const geometry_msgs::PointStamped& approxDoorPose);

    bool isActive() override;

    void runCycle() override;

private:
    enum class State {
        Idle,
        WaitingDoorPose,
        RotatingStart,
        DrivingDoor,
        RotatingFinal,
    } mState = State::Idle;

    void onDoorHandleReceived(const geometry_msgs::PoseStamped& pose);

    geometry_msgs::PoseStamped mDoorPose;

    ros::Publisher mEnablePub, mApproxDoorPub;
    ros::Subscriber mDoorHandleSub;

    HPNav& mNav;

    void rotateAtStart();

    void driveToDoor();

    void rotateFinal();

    void setState(State newState);
};



