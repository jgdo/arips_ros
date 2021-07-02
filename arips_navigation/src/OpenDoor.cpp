#include "arips_navigation/OpenDoor.h"
#include <arips_navigation/StateExecutor.h>

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

struct OpenDoor::Pimpl : public StateExecutor<Pimpl, DrivingStateProto> {
    geometry_msgs::PoseStamped mDoorApproachPose;

    ros::Publisher mEnablePub, mApproachPosePub, mServoPub;
    ros::Subscriber mDoorHandleSub, mServoSub;

    VelocityPlanner mVelPlanner{0.01, 0.03};

    const float DoorAngleOffset = std::asin(0.25F / 0.7F);

    float mLastServoPos = 0;

    DriveUntilCollision& mDriveUntilCollision;

    arips_navigation::OpenDoorConfig mConfig;
    dynamic_reconfigure::Server<arips_navigation::OpenDoorConfig> mConfigServer{
        ros::NodeHandle{"~/OpenDoor"}};

    explicit Pimpl(NavigationContext& context, DriveUntilCollision& driveUntilCollision)
        : StateExecutor(context), mDriveUntilCollision{driveUntilCollision} {
        ros::NodeHandle nh;
        mEnablePub = nh.advertise<std_msgs::Bool>("enable_door_handle", 1, true);
        mApproachPosePub = nh.advertise<geometry_msgs::PoseStamped>("door_approach_pose", 1, true);
        mServoPub = nh.advertise<std_msgs::Float32>("/servo_7/setpoint_deg", 1, true);

        mDoorHandleSub = nh.subscribe("door_handle/pose", 1, &Pimpl::onDoorHandleReceived, this);
        mServoSub = nh.subscribe("/servo_7/pos_deg", 1, &Pimpl::onServoPosReceived, this);

        dynamic_reconfigure::Server<arips_navigation::OpenDoorConfig>::CallbackType cb =
            [this](auto&& PH1, auto&& PH2) {
                onDynamicReconfigure(std::forward<decltype(PH1)>(PH1),
                                     std::forward<decltype(PH2)>(PH2));
            };
        mConfigServer.setCallback(cb);
    }

    void onDynamicReconfigure(arips_navigation::OpenDoorConfig& config, uint32_t level) {
        mConfig = config;
    }

    void init() {
        std_msgs::Bool enable;
        enable.data = true;
        mEnablePub.publish(enable);

        setServo(0);

        setState(&Pimpl::waitingDoorPose);

        geometry_msgs::Twist cmd_vel;
        publishCmdVel(cmd_vel);
    }

    // NOLINTNEXTLINE
    void setServo(float val) {
        std_msgs::Float32 servo;
        servo.data = val;
        mServoPub.publish(servo);
    }

    std::optional<geometry_msgs::Pose> getApproachPoseFromBase() const {
        try {
            geometry_msgs::PoseStamped currentPose = tf().transform(
                mDoorApproachPose, "arips_base", ros::Time::now(), "odom", ros::Duration(0.1));

            return currentPose.pose;
        } catch (const tf2::TransformException& ex) {
            ROS_WARN_STREAM("getApproachPoseFromBase() error: " << ex.what());
            return {};
        }
    }

    void waitingDoorPose() {}

    void onDoorHandleReceived(const geometry_msgs::PoseStamped& pose) {
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

            if (isStateFunc(&Pimpl::waitingDoorPose)) {
                setState(&Pimpl::rotateAtStart);
                std_msgs::Bool enable;
                enable.data = false;
                mEnablePub.publish(enable);
            }
        } catch (const tf2::TransformException& ex) {
            ROS_WARN_STREAM("onDoorHandleReceived() error: " << ex.what());
        }
    }

    void onServoPosReceived(const std_msgs::Float32& msg) { mLastServoPos = msg.data; }

    void rotateAtStart() {
        setState(&Pimpl::driveToDoor);

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
        setState(State::drivingDoor);
    }

    mCmdVelPub.publish(cmd_vel);
#endif
    }

    void driveToDoor() {
        geometry_msgs::Twist cmd_vel;

        const auto approachPose = getApproachPoseFromBase();
        if (approachPose) {
            const auto handleStart = approachPose->position;

            const auto angleDiff = std::atan2(handleStart.y, -handleStart.x);
            const auto floatAngleThres = angles::from_degrees(20);

            ROS_INFO_STREAM("angleDiff: " << angleDiff);

            if (angleDiff > floatAngleThres) {
                cmd_vel.angular.z = -0.2;
            } else if (angleDiff < -floatAngleThres) {
                cmd_vel.angular.z = 0.2;
            } else if (!mVelPlanner.computeVelocity(handleStart.x, handleStart.y, cmd_vel)) {
                setState(&Pimpl::rotateFinal);
            }
        }

        publishCmdVel(cmd_vel);

#if 0
        if(mNav.isActive()) {
        mNav.runCycle();
    } else {
        setState(&Pimpl::RotatingFinal);
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
            const auto handleAngle = std::atan2(handleDirVec.y(), handleDirVec.x());

            const auto angleDiff = angles::normalize_angle(handleAngle);
            const auto angleThres = angles::from_degrees(5);

            ROS_INFO_STREAM("angleDiff: " << angleDiff);

            if (angleDiff > angleThres) {
                cmd_vel.angular.z = 0.2;
            } else if (angleDiff < -angleThres) {
                cmd_vel.angular.z = -0.2;
            } else {
                setState(&Pimpl::pullHandle);
            }
        }

        publishCmdVel(cmd_vel);
    }

    ros::Time mLastTime{0};

    void clearAfterPulling() {
        setServo(0);
        if (mLastServoPos  < 5.0) {
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0.1;
            mDriveUntilCollision.activate(cmd_vel, mConfig.drive_open_wall_dist/3, ros::Duration{5});
            execState(&mDriveUntilCollision, nullptr);
        }
    }

    void pullHandle() {
        setServo(100);
        if (mLastServoPos > 90.0) {
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0.2;
            cmd_vel.angular.z = 0.3;
            mDriveUntilCollision.activate(cmd_vel, mConfig.drive_open_wall_dist, ros::Duration{5});
            execState(&mDriveUntilCollision, &Pimpl::clearAfterPulling);
        }
    }
};

OpenDoor::OpenDoor(NavigationContext& context, DriveUntilCollision& driveUntilCollision)
    : pimpl(std::make_unique<Pimpl>(context, driveUntilCollision)) {}

OpenDoor::~OpenDoor() = default;

void OpenDoor::init() { pimpl->init(); }

bool OpenDoor::isActive() { return pimpl->isActive(); }

void OpenDoor::runCycle() { pimpl->runCycle(); }
