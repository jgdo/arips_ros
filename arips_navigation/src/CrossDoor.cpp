#include <arips_navigation/CrossDoor.h>
#include <arips_navigation/StateExecutor.h>

#include <arips_navigation/utils/VariableSubscriber.h>
#include <arips_navigation/utils/transforms.h>

#include <angles/angles.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

struct CrossDoor::Pimpl : public StateExecutor<Pimpl, DrivingStateProto> {
    constexpr static auto fixedFrame = "map";
    constexpr static auto robotFrame = "arips_base";
    const double closedDoorAngleThres = angles::from_degrees(30);
    const double fullyOpenDoorAngleThres = angles::from_degrees(80);

    Pimpl(NavigationContext& ctx, DriveTo& driveTo, OpenDoor& openDoor)
        : StateExecutor{ctx}, mDriveTo{driveTo}, mOpenDoor{openDoor} {}

    ~Pimpl() override = default;

    void activate(arips_navigation::CrossDoorInformation const& doorInfo) {
        mDoorInfo = doorInfo;
        mDoorPoseSub.receivedOnce = false;
        mDoorPivotPub.publish(doorInfo.pivotPose);

        initialRobotPose = *tryLookupTransform(tf(), fixedFrame, robotFrame);

        setState(&Pimpl::checkDoor);
    }

    void setKinectAngleDeg(float angle_deg) const {
        std_msgs::Float32 msg;
        msg.data = angle_deg;
        mKinectServoPub.publish(msg);
    }

    void checkDoor() {
        static const auto doorApproachDistFromPivot = 0.72;
        static const auto doorApproachDistFromHandle = 0.3;
        static const auto doorPushDistFromPivot = 0.6;
        static const auto doorPushApproachDistFromDoor = -0.27;

        if (isStateInit()) {
            setKinectAngleDeg(0);

            // TODO find better method
            mDoorInfo.pivotPose.header.stamp = ros::Time::now();
            mDoorInfo.approachPose.header.stamp = ros::Time::now();
        }

        if (!mDoorPoseSub.receivedOnce) {
            ROS_INFO_STREAM("Door pose not received yet");
            return;
        }

        const auto optDoorPose = tryTransformPoseMsgToTransform(tf(), mDoorPoseSub.msg,
                                                                robotFrame, ros::Duration(0.1));
        const auto optDoorPivot = tryTransformPoseMsgToTransform(tf(), mDoorInfo.pivotPose,
                                                                 robotFrame, ros::Duration(0.1));

        if (!optDoorPose || !optDoorPivot) {
            ROS_INFO_STREAM("Could not transform pivot or door pose");
            return;
        }

        const auto doorVector = optDoorPose->getOrigin() - optDoorPivot->getOrigin();
        const auto doorOpenAngle =
            std::abs(doorVector.angle(optDoorPivot->getBasis().getColumn(0)));
        ROS_INFO_STREAM("Detected door angle is " << angles::to_degrees(doorOpenAngle) << " deg");
        if (doorOpenAngle < closedDoorAngleThres) {
            // consider door as closed => hook open door

            // compute direction of door to open
            tf2::Vector3 doorOpenDirection = optDoorPose->getBasis().getColumn(0).normalized();
            if (doorOpenDirection.x() > 0) {
                doorOpenDirection = -doorOpenDirection;
            }

            const tf2::Vector3 handleApproachPoint =
                optDoorPivot->getOrigin() + doorVector.normalized() * doorApproachDistFromPivot +
                doorOpenDirection * doorApproachDistFromHandle;

            tf2::Quaternion handleApproachRot;
            handleApproachRot.setRPY(0, 0,
                                     std::atan2(doorOpenDirection.y(), doorOpenDirection.x()));

            const tf2::Stamped<tf2::Transform> handleApproachPoseRobot{
                tf2::Transform{handleApproachRot, handleApproachPoint}, optDoorPose->stamp_,
                optDoorPose->frame_id_};

            if (!mDriveTo.driveTo(handleApproachPoseRobot)) {
                ROS_WARN("CrossDoor: could not plan to handleApproachPoseRobot");
                return;
            }
            setKinectAngleDeg(-110);

            execState(&mDriveTo, &Pimpl::openDoorHook);
        } else if (doorOpenAngle > fullyOpenDoorAngleThres) {
            // consider door as fully opened => just do approach pose
            setState(&Pimpl::approachCross);
        } else {
            // consider door as half open => push door fully open
            // compute direction of door to open
            tf2::Vector3 doorOpenDirection = optDoorPose->getBasis().getColumn(0).normalized();
            if (doorOpenDirection.x() > 0) {
                doorOpenDirection = -doorOpenDirection;
            }

            const tf2::Vector3 handleApproachPoint =
                optDoorPivot->getOrigin() + doorVector.normalized() * doorPushDistFromPivot +
                doorOpenDirection * doorPushApproachDistFromDoor;

            tf2::Quaternion handleApproachRot;
            handleApproachRot.setRPY(0, 0,
                                     std::atan2(-doorOpenDirection.y(), -doorOpenDirection.x()));

            const tf2::Stamped<tf2::Transform> handleApproachPoseRobot{
                tf2::Transform{handleApproachRot, handleApproachPoint}, optDoorPose->stamp_,
                optDoorPose->frame_id_};

            if (!mDriveTo.driveTo(handleApproachPoseRobot)) {
                return;
            }

            setState(&Pimpl::openDoorPush);
        }
    }

    void approachCross() {
        if (isStateInit()) {
            mDriveTo.driveTo(fromMsg<tf2::Stamped<tf2::Transform>>(mDoorInfo.approachPose));
        }

        if (mDriveTo.isActive()) {
            mDriveTo.runCycle();
        } else {
            setState(nullptr);
        }
    }

    void openDoorHook() {
        if(isStateInit()) {
            mOpenDoor.init();
        }

        if (mOpenDoor.isActive()) {
            mOpenDoor.runCycle();
        } else {
            if (!mDriveTo.driveTo(initialRobotPose)) {
                return;
            }
            setState(&Pimpl::returnInitial);
        }
    }

    void openDoorPush() {
        if (mDriveTo.isActive()) {
            mDriveTo.runCycle();
            return;
        }

        const auto robotPoseInMap = tryLookupTransform(tf(), fixedFrame, robotFrame);
        if (!robotPoseInMap) {
            return;
        }

        auto optDoorPivotMap = tryTransformPoseMsgToTransform(tf(), mDoorInfo.pivotPose,
                                                              fixedFrame, ros::Duration(0.1));
        if (!optDoorPivotMap) {
            return;
        }

        const auto distToOpen = (robotPoseInMap->getOrigin() - optDoorPivotMap->getOrigin())
                                    .dot(optDoorPivotMap->getBasis().getColumn(0));

        ROS_INFO_STREAM("distToOpen: " << distToOpen);

        geometry_msgs::Twist cmd_vel;
        if (distToOpen > 0.2) {
            cmd_vel.linear.x = -0.1;
        } else {
            setState(&Pimpl::clearAfterPush);
        }
        publishCmdVel(cmd_vel);
    }

    void clearAfterPush() {
        static ros::Time lastTime(0);

        if (isStateInit()) {
            lastTime = ros::Time::now();
        }

        geometry_msgs::Twist cmd_vel;

        const auto duration = ros::Time::now() - lastTime;
        if (duration.toSec() < 1.0) {
            cmd_vel.linear.x = 0.1;
            cmd_vel.angular.z = 0.5;
        } else {
            setState(&Pimpl::approachCross);
        }

        publishCmdVel(cmd_vel);
    }

    void returnInitial() {
        if (mDriveTo.isActive()) {
            mDriveTo.runCycle();
        } else {
            setState(&Pimpl::checkDoor);
        }
    }

    DriveTo& mDriveTo;
    OpenDoor& mOpenDoor;

    arips_navigation::CrossDoorInformation mDoorInfo;
    tf2::Stamped<tf2::Transform> initialRobotPose;
    VariableSubscriber<geometry_msgs::PoseStamped> mDoorPoseSub{"detected_door_pose", 1};
    ros::Publisher mDoorPivotPub{
        mNodeHandle.advertise<geometry_msgs::PoseStamped>("door_pivot_pose", 1)};
    ros::Publisher mKinectServoPub{
        mNodeHandle.advertise<std_msgs::Float32>("/servo_6/setpoint_deg", 1)};
};

void CrossDoor::activate(arips_navigation::CrossDoorInformation const& doorInfo) {
    pimpl->activate(doorInfo);
}

bool CrossDoor::isActive() { return pimpl->isActive(); }

void CrossDoor::runCycle() { return pimpl->runCycle(); }

CrossDoor::CrossDoor(NavigationContext& ctx, DriveTo& driveTo,
                     OpenDoor& openDoor)
    : pimpl{std::make_unique<Pimpl>(ctx, driveTo, openDoor)} {}

CrossDoor::~CrossDoor() = default;
