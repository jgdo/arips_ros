#include <arips_navigation/CrossDoor.h>

#include <arips_navigation/utils/VariableSubscriber.h>
#include <arips_navigation/utils/transforms.h>

#include <angles/angles.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

struct CrossDoor::Pimpl : public DrivingStateProto {
    constexpr static auto fixedFrame = "map";
    constexpr static auto robotFrame = "arips_base";
    const double closedDoorAngleThres = angles::from_degrees(30);
    const double fullyOpenDoorAngleThres = angles::from_degrees(90);

    Pimpl(tf2_ros::Buffer &tf, ros::Publisher &cmdVelPub,
          TopoExecuter &topoExec, OpenDoor &openDoor)
        : DrivingStateProto(tf, mCmdVelPub), mTopoExec{topoExec},
          mOpenDoor{openDoor} {}

    ~Pimpl() override = default;

    void activate(arips_navigation::CrossDoorInformation const &doorInfo) {
        mDoorInfo = doorInfo;
        mDoorPoseSub.receivedOnce = false;
        mDoorPivotPub.publish(doorInfo.pivotPose);
        setState(State::CheckDoor);

        setKinectAngleDeg(0);
    }

    void setKinectAngleDeg(float angle_deg) {
        std_msgs::Float32 msg;
        msg.data = angle_deg;
        mKinectServoPub.publish(msg);
    }

    bool isActive() override { return mState != State::Idle; }
    void runCycle() override {
        mStateCount++;

        switch (mState) {
        case State::Idle:
            break;

        case State::CheckDoor:
            checkDoor();
            break;

        case State::Approach:
            ROS_INFO_STREAM("State::Approach not implemented");
            setState(State::Idle);
            break;

        case State::ApproachHook:
            approachHook();
            break;

        case State::OpenDoorHook:
            openDoorHook();
            break;

        case State::OpenDoorPush:
            ROS_INFO_STREAM("State::OpenDoorPush not implemented");
            setState(State::Idle);
            break;

        default:
            ROS_INFO_STREAM("State # " << (int)mState << "  not implemented");
            setState(State::Idle);
            break;
        }
    }

    void checkDoor() {
        static const auto doorApproachDistFromPivot = 0.7;
        static const auto doorApproachDistFromHandle = 0.3;

        if (!mDoorPoseSub.receivedOnce) {
            ROS_INFO_STREAM("Door pose not received yet");
            return;
        }

        const auto optDoorPose = tryTransformPoseMsgToTransform(
            mTfBuffer, mDoorPoseSub.msg, robotFrame, ros::Duration(0.1));
        const auto optDoorPivot = tryTransformPoseMsgToTransform(
            mTfBuffer, mDoorInfo.pivotPose, robotFrame, ros::Duration(0.1));

        if (!optDoorPose || !optDoorPivot) {
            ROS_INFO_STREAM("Could not transform pivot or door pose");
            return;
        }

        const auto doorVector =
            optDoorPose->getOrigin() - optDoorPivot->getOrigin();
        const auto doorOpenAngle =
            std::abs(doorVector.angle(optDoorPivot->getBasis().getColumn(0)));
        ROS_INFO_STREAM("Detected door angle is "
                        << angles::to_degrees(doorOpenAngle) << " deg");
        if (doorOpenAngle < closedDoorAngleThres) {
            // consider door as closed => hook open door

            // compute direction of door to open
            tf2::Vector3 doorOpenDirection =
                optDoorPose->getBasis().getColumn(0).normalized();
            if (doorOpenDirection.x() > 0) {
                doorOpenDirection = -doorOpenDirection;
            }

            const tf2::Vector3 handleApproachPoint =
                optDoorPivot->getOrigin() +
                doorVector.normalized() * doorApproachDistFromPivot +
                doorOpenDirection * doorApproachDistFromHandle;

            tf2::Quaternion handleApproachRot;
            handleApproachRot.setRPY(
                0, 0, std::atan2(doorOpenDirection.y(), doorOpenDirection.x()));

            geometry_msgs::PoseStamped handleApproachPoseRobot;
            handleApproachPoseRobot.header.stamp = optDoorPose->stamp_;
            handleApproachPoseRobot.header.frame_id = optDoorPose->frame_id_;
            tf2::toMsg(handleApproachPoint,
                       handleApproachPoseRobot.pose.position);
            handleApproachPoseRobot.pose.orientation =
                tf2::toMsg(handleApproachRot);

            const auto handleApproachPoseGlobal =
                tryTransform(mTfBuffer, handleApproachPoseRobot, fixedFrame,
                             ros::Duration(0.1));
            if (!handleApproachPoseGlobal) {
                ROS_ERROR_STREAM("Could not transform handleApproachPoseRobot "
                                 "to fixed frame "
                                 << fixedFrame);
                // setState(State::OpenDoorHook);
                return;
            }

            mTopoExec.activate(*handleApproachPoseGlobal);
            setState(State::ApproachHook);
        } else if (doorOpenAngle > fullyOpenDoorAngleThres) {
            // consider door as fully opened => just do approach pose
            setState(State::Approach);
        } else {
            // consider door as half open => push door fully open
            setState(State::OpenDoorPush);
        }
    }

    void approachHook() {
        if (stateInit()) {
            setKinectAngleDeg(-110);
        }

        if (mTopoExec.isActive()) {
            mTopoExec.runCycle();
        } else {
            mOpenDoor.init();
            setState(State::OpenDoorHook);
        }
    }

    void openDoorHook() {
        if (mOpenDoor.isActive()) {
            mOpenDoor.runCycle();
        } else {
            setState(State::Idle);
        }
    }

    TopoExecuter &mTopoExec;
    OpenDoor &mOpenDoor;

    enum class State {
        Idle,
        CheckDoor,
        ApproachHook,
        OpenDoorHook,
        OpenDoorPush,
        Approach
    } mState = State::Idle;

    int mStateCount = 0; // count of cycles in current state

    arips_navigation::CrossDoorInformation mDoorInfo;
    VariableSubscriber<geometry_msgs::PoseStamped> mDoorPoseSub{
        "detected_door_pose", 1};
    ros::Publisher mDoorPivotPub{
        mNodeHandle.advertise<geometry_msgs::PoseStamped>("door_pivot_pose",
                                                          1)};
    ros::Publisher mKinectServoPub{
        mNodeHandle.advertise<std_msgs::Float32>("/servo_6/setpoint_deg", 1)};

    void setState(State newState) {
        mState = newState;
        mStateCount = -1;
        ROS_INFO_STREAM("CrossDoor::setState: " << (int)newState);
    }

    bool stateInit() const { return mStateCount == 0; }
};

void CrossDoor::activate(
    arips_navigation::CrossDoorInformation const &doorInfo) {
    pimpl->activate(doorInfo);
}

bool CrossDoor::isActive() { return pimpl->isActive(); }

void CrossDoor::runCycle() { return pimpl->runCycle(); }

CrossDoor::CrossDoor(tf2_ros::Buffer &tf, ros::Publisher &cmdVelPub,
                     TopoExecuter &topoExec, OpenDoor &openDoor)
    : pimpl{std::make_unique<Pimpl>(tf, cmdVelPub, topoExec, openDoor)} {}

CrossDoor::~CrossDoor() = default;
