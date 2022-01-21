#include <arips_navigation/DriveTo.h>

#include <arips_navigation/path_planning/Locomotion.h>
#include <arips_navigation/path_planning/PotentialMapVisualizer.h>

#include "arips_navigation/path_planning/Costmap2dView.h"
#include <nav_msgs/Odometry.h>

struct DriveTo::Pimpl {
    DriveTo& mParent;
    Locomotion& mLocomotion;
    PotentialMapVisualizer mMapViz;

    ros::Subscriber mTwistSub;
    nav_msgs::Odometry mLastOdom;

    explicit Pimpl(DriveTo& parent, Locomotion& locomotion)
        : mParent{parent}, mLocomotion{locomotion} {

        mTwistSub = ros::NodeHandle().subscribe<nav_msgs::Odometry>(
            "/odom", 1, [this](const nav_msgs::OdometryConstPtr& msg) { mLastOdom = *msg; });
    }

    bool driveTo(const tf2::Stamped<tf2::Transform>& goal) {
        geometry_msgs::PoseStamped robotPose;
        if (!mParent.mContext.globalCostmap.getRobotPose(robotPose)) {
            ROS_WARN_STREAM("Could not get robot pose");
            mLocomotion.cancel(); // make sure robot stops instead of using the old goal
            return false;
        }

        try {
            const auto poseOnFloor = mParent.mContext.tf.transform(
                goal, mParent.mContext.globalCostmap.getGlobalFrameID());

            const auto planOk =
                mLocomotion.setGoal(Costmap2dView{mParent.mContext.globalCostmap, Pose2D::fromMsg(robotPose.pose)},
                                    Pose2D::fromMsg(robotPose.pose), Pose2D::fromTf(poseOnFloor));

            if (!planOk) {
                ROS_WARN("Could not plan path to goal, clearing global costmap...");
                mParent.mContext.globalCostmap.resetLayers();
                if (!mLocomotion.setGoal(Costmap2dView{mParent.mContext.globalCostmap, Pose2D::fromMsg(robotPose.pose)},
                                         Pose2D::fromMsg(robotPose.pose),
                                         Pose2D::fromTf(poseOnFloor))) {
                    ROS_ERROR("Could not plan path to goal even after clearing the global costmap");
                    return false;
                }
            }
        } catch (const tf2::TransformException& ex) {
            ROS_WARN("DriveTo::driveTo(): %s", ex.what());
            return false;
        }

        return true;
    }

    void runCycle() {
        geometry_msgs::PoseStamped robotPoseMsg;
        if (mParent.mContext.globalCostmap.getRobotPose(robotPoseMsg)) {
            const auto robotPose = Pose2D::fromMsg(robotPoseMsg.pose);
            const auto optTwist = mLocomotion.computeVelocityCommands(
                Costmap2dView{mParent.mContext.globalCostmap, robotPose},
                {robotPose, Pose2D::fromMsg(mLastOdom.twist.twist)});

            if (optTwist) {
                mParent.mContext.publishCmdVel(optTwist->toTwistMsg());
                const auto* potmap = mLocomotion.potentialMap();
                if(potmap) {
                    mMapViz.showMap(*potmap, robotPose);
                }
            }
        }

        ROS_WARN("Could not get robot pose or robot stuck, cancelling");
        mLocomotion.cancel();
        mParent.mContext.publishCmdVel({});
    }
};

DriveTo::DriveTo(NavigationContext& context, Locomotion& locomotion)
    : DrivingStateProto(context), pimpl{std::make_unique<Pimpl>(*this, locomotion)} {}

bool DriveTo::driveTo(const tf2::Stamped<tf2::Transform>& goal) { return pimpl->driveTo(goal); }

bool DriveTo::isActive() { return pimpl->mLocomotion.hasGoal(); }

void DriveTo::runCycle() { return pimpl->runCycle(); }

DriveTo::~DriveTo() = default;
