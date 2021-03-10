//
// Created by jgdo on 2/28/21.
//

#include "arips_navigation/OpenDoor.h"

#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

OpenDoor::OpenDoor(tf2_ros::Buffer &tf, ros::Publisher &cmdVelPub, HPNav& nav) :
    DrivingStateProto(tf, cmdVelPub),
    mNav(nav)
{
    ros::NodeHandle nh;
    mEnablePub = nh.advertise<std_msgs::Bool>("enable_door_handle", 1, true);
    mApproxDoorPub = nh.advertise<geometry_msgs::PointStamped>("approx_door_handle_pos", 1, true);

    mDoorHandleSub = nh.subscribe("door_handle_pose", 1, &OpenDoor::onDoorHandleReceived, this);
    mDoorHandleVisualSub = nh.subscribe("/door_handle/poses", 1, &OpenDoor::onDoorHandleVisualReceived, this);
}

void OpenDoor::init(const geometry_msgs::PointStamped &approxDoorPose) {
    mApproxDoorPub.publish(approxDoorPose);

    std_msgs::Bool enable;
    enable.data = true;
    mEnablePub.publish(enable);

    setState(State::WaitingDoorPose);

    geometry_msgs::Twist cmd_vel;
    mCmdVelPub.publish(cmd_vel);
}

bool OpenDoor::isActive() {
    return mState != State::Idle;
}

void OpenDoor::runCycle() {
    switch (mState) {
        case State::Idle:
        // case State::WaitingDoorPose:
            break;

        case State::WaitingDoorPose:
        case State::RotatingStart:
            rotateAtStart();
            break;

        case State::DrivingDoor:
            driveToDoor();
            break;

        case State::RotatingFinal:
            rotateFinal();
            break;
    }
}

void OpenDoor::rotateAtStart() {
    setState(State::DrivingDoor);

#if 0
    // rotate away such that goal is at robot's back side
    const geometry_msgs::TransformStamped transform = mTfBuffer.lookupTransform("arips_base", mDoorPose.header.frame_id,  ros::Time(0));
    // get goal position relative to robot
    geometry_msgs::Point gaolPos;
    tf2::doTransform(mDoorPose.pose.position, gaolPos, transform);

    const float headingAngle = std::atan2(gaolPos.y, gaolPos.x);

    const float angleTolerance = 30.0 / 180.0 * M_PI;

    geometry_msgs::Twist cmd_vel;

    if(std::abs(headingAngle) < M_PI - angleTolerance) {
        if(headingAngle > 0) {
            cmd_vel.angular.z = -0.3;
        } else {
            cmd_vel.angular.z = 0.3;
        }
    } else {
        mNav.setGoal(mDoorPose);
        setState(State::DrivingDoor);
    }

    mCmdVelPub.publish(cmd_vel);
#endif
}

void OpenDoor::driveToDoor() {
    geometry_msgs::Twist cmd_vel;
    if(mLastVisualHandlePoses.poses.size() > 0) {
        const float x = mLastVisualHandlePoses.poses[1].position.x;

        if(x > 0.1) {
            cmd_vel.angular.z = 0.2;
        } else if(x < -0.1) {
            cmd_vel.angular.z = -0.2;
        }
    }

    mCmdVelPub.publish(cmd_vel);

#if 0
    if(mNav.isActive()) {
        mNav.runCycle();
    } else {
        setState(State::RotatingFinal);
    }
#endif
}

void OpenDoor::rotateFinal() {
    // TODO actually rotate
    setState(State::Idle);
}

void OpenDoor::setState(OpenDoor::State newState) {
    mState = newState;
    ROS_INFO_STREAM("OpenDoor: new state " << (int)newState);
}

void OpenDoor::onDoorHandleReceived(const geometry_msgs::PoseStamped &pose) {
    if(mState == State::WaitingDoorPose) {
        mDoorPose = pose;
        setState(State::RotatingStart);
        std_msgs::Bool enable;
        enable.data = false;
        mEnablePub.publish(enable);
    }
}

void OpenDoor::onDoorHandleVisualReceived(const geometry_msgs::PoseArray &poses) {
    mLastVisualHandlePoses = poses;
}
