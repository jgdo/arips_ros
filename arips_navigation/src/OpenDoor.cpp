//
// Created by jgdo on 2/28/21.
//

#include "arips_navigation/OpenDoor.h"

#include <angles/angles.h>
#include <geometry_msgs/Twist.h>
#include <optional>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <arips_navigation/OpenDoorConfig.h>
#include <arips_navigation/local_planner/VelocityPlanner.h>
#include <dynamic_reconfigure/server.h>

struct OpenDoor::Pimpl {
    enum class State {
        Idle,
        WaitingDoorPose,
        RotatingStart,
        DrivingDoor,
        RotatingFinal,
        PullHandle,
        DriveOpen,
        DriveAway,
    } mState = State::Idle;

    geometry_msgs::PoseStamped mDoorApproachPose;

    ros::Publisher mEnablePub, mApproachPosePub, mServoPub;
    ros::Subscriber mDoorHandleSub, mServoSub;

    OpenDoor &mParent;

    VelocityPlanner mVelPlanner{0.01, 0.03};

    const float DoorAngleOffset = asin(0.25 / 0.7);

    float mLastServoPos = 0;

    arips_navigation::OpenDoorConfig mConfig;
    dynamic_reconfigure::Server<arips_navigation::OpenDoorConfig> mConfigServer {ros::NodeHandle {"~/OpenDoor"}};

    explicit Pimpl(OpenDoor &parent) : mParent(parent) {
        ros::NodeHandle nh;
        mEnablePub = nh.advertise<std_msgs::Bool>("enable_door_handle", 1, true);
        mApproachPosePub = nh.advertise<geometry_msgs::PoseStamped>("door_approach_pose", 1, true);
        mServoPub = nh.advertise<std_msgs::Float32>("/servo_7/setpoint_deg", 1, true);

        mDoorHandleSub = nh.subscribe("door_handle/pose", 1, &Pimpl::onDoorHandleReceived, this);
        mServoSub = nh.subscribe("/servo_7/pos_deg", 1, &Pimpl::onServoPosReceived, this);

        dynamic_reconfigure::Server<arips_navigation::OpenDoorConfig>::CallbackType cb =
            boost::bind(&Pimpl::onDynamicReconfigure, this, _1, _2);
        mConfigServer.setCallback(cb);
    }

    void onDynamicReconfigure(arips_navigation::OpenDoorConfig &config, uint32_t level) {
        mConfig = config;
    }

    void init() {
        std_msgs::Bool enable;
        enable.data = true;
        mEnablePub.publish(enable);

        setServo(0);

        setState(State::WaitingDoorPose);

        geometry_msgs::Twist cmd_vel;
        mParent.mCmdVelPub.publish(cmd_vel);
    }

    void runCycle() {
        switch (mState) {
        case State::Idle:
        case State::WaitingDoorPose:
            break;

        case State::RotatingStart:
            rotateAtStart();
            break;

        case State::DrivingDoor:
            driveToDoor();
            break;

        case State::RotatingFinal:
            rotateFinal();
            break;

        case State::PullHandle:
            pullHandle();
            break;

        case State::DriveOpen:
            driveOpen();
            break;

        case State::DriveAway:
            driveAway();
            break;
        }
    }

    void setServo(float val) {
        std_msgs::Float32 servo;
        servo.data = val;
        mServoPub.publish(servo);
    }

    std::optional<geometry_msgs::Pose> getApproachPoseFromBase() const {
        try {
            geometry_msgs::PoseStamped currentPose = mParent.mTfBuffer.transform(
                mDoorApproachPose, "arips_base", ros::Time::now(), "odom", ros::Duration(0.1));

            return currentPose.pose;
        } catch (const tf2::TransformException &ex) {
            ROS_WARN_STREAM("getApproachPoseFromBase() error: " << ex.what());
            return {};
        }
    }

    void onDoorHandleReceived(const geometry_msgs::PoseStamped &pose) {
        try {
            tf2::Stamped<tf2::Transform> poseTransform;
            tf2::fromMsg(pose, poseTransform);
            // poseTransform.setOrigin(poseTransform(tf2::Vector3(distFromDoorOffset,
            // 0, 0)));
            const tf2::Vector3 approachPos = poseTransform(tf2::Vector3(
                mConfig.door_handle_approach_x_offset, mConfig.door_handle_approach_y_offset, 0));

            mDoorApproachPose.header = pose.header;
            tf2::toMsg(approachPos, mDoorApproachPose.pose.position);
            mDoorApproachPose.pose.position.z = 0;

            tf2::Quaternion handleQuat;
            tf2::fromMsg(pose.pose.orientation, handleQuat);
            const tf2::Quaternion newHandleQuat =
                tf2::Quaternion(tf2::Vector3(0, 0, 1), DoorAngleOffset) * handleQuat;
            mDoorApproachPose.pose.orientation = tf2::toMsg(newHandleQuat);

            mApproachPosePub.publish(mDoorApproachPose);

            if (mState == State::WaitingDoorPose) {
                setState(State::RotatingStart);
                std_msgs::Bool enable;
                enable.data = false;
                mEnablePub.publish(enable);
            }
        } catch (const tf2::TransformException &ex) {
            ROS_WARN_STREAM("onDoorHandleReceived() error: " << ex.what());
        }
    }

    void onServoPosReceived(const std_msgs::Float32 &msg) { mLastServoPos = msg.data; }

    void rotateAtStart() {
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

    void driveToDoor() {
        geometry_msgs::Twist cmd_vel;

        const auto approachPose = getApproachPoseFromBase();
        if (approachPose) {
            const auto handleStart = approachPose->position;

            const float angleDiff = std::atan2(handleStart.y, -handleStart.x);
            const float floatAngleThres = angles::from_degrees(20);

            ROS_INFO_STREAM("angleDiff: " << angleDiff);

            if (angleDiff > floatAngleThres) {
                cmd_vel.angular.z = -0.2;
            } else if (angleDiff < -floatAngleThres) {
                cmd_vel.angular.z = 0.2;
            } else if (!mVelPlanner.computeVelocity(handleStart.x, handleStart.y, cmd_vel)) {
                setState(State::RotatingFinal);
            }
        }

        mParent.mCmdVelPub.publish(cmd_vel);

#if 0
        if(mNav.isActive()) {
        mNav.runCycle();
    } else {
        setState(State::RotatingFinal);
    }
#endif
    }

    void rotateFinal() {
        geometry_msgs::Twist cmd_vel;

        const auto approachPose = getApproachPoseFromBase();
        if (approachPose) {
            tf2::Quaternion handleQuat;
            tf2::fromMsg(approachPose->orientation, handleQuat);
            const auto handleDirVec = tf2::quatRotate(handleQuat, tf2::Vector3(1, 0, 0));
            const float handleAngle = std::atan2(handleDirVec.y(), handleDirVec.x());

            const float angleDiff = angles::normalize_angle(handleAngle);
            const float angleThres = angles::from_degrees(5);

            ROS_INFO_STREAM("angleDiff: " << angleDiff);

            if (angleDiff > angleThres) {
                cmd_vel.angular.z = 0.2;
            } else if (angleDiff < -angleThres) {
                cmd_vel.angular.z = -0.2;
            } else {
                setState(State::PullHandle);
            }
        }

        mParent.mCmdVelPub.publish(cmd_vel);
    }

    void driveOpen() {
        static ros::Time lastTime(0);

        if (lastTime.isZero()) {
            lastTime = ros::Time::now();
        }

        geometry_msgs::Twist cmd_vel;

        const auto duration = ros::Time::now() - lastTime;
        if (duration.toSec() < 3) {
            cmd_vel.linear.x = 0.2;
            cmd_vel.angular.z = 0.3;
        } else {
            setServo(0);
            lastTime = ros::Time(0);
            setState(State::DriveAway);
        }

        mParent.mCmdVelPub.publish(cmd_vel);
    }

    void driveAway() {
        static ros::Time lastTime(0);

        if (lastTime.isZero()) {
            lastTime = ros::Time::now();
        }

        geometry_msgs::Twist cmd_vel;

        const auto duration = ros::Time::now() - lastTime;
        if (duration.toSec() < 3.5) {

        } else if (duration.toSec() < 5.0) {
            cmd_vel.linear.x = 0.1;
            cmd_vel.angular.z = -0.4;
        } else {
            lastTime = ros::Time(0);
            setState(State::Idle);
        }

        mParent.mCmdVelPub.publish(cmd_vel);
    }

    void pullHandle() {
        setServo(100);
        if (mLastServoPos > 90.0) {
            setState(State::DriveOpen);
        }
    }

    void setState(State newState) {
        mState = newState;
        ROS_INFO_STREAM("OpenDoor: new state " << (int)newState);
    }
};

OpenDoor::OpenDoor(tf2_ros::Buffer &tf, ros::Publisher &cmdVelPub)
    : DrivingStateProto(tf, cmdVelPub), pimpl(std::make_unique<Pimpl>(*this)) {}

OpenDoor::~OpenDoor() = default;

void OpenDoor::init() { pimpl->init(); }

bool OpenDoor::isActive() { return pimpl->mState != Pimpl::State::Idle; }

void OpenDoor::runCycle() { pimpl->runCycle(); }
