#include <utility>

#include <arips_navigation/DriveTo.h>
#include <arips_navigation/local_planner/arips_planner_ros.h>

#include <arips_navigation/FlatGroundModule.h>
#include <arips_navigation/utils/FixedPosition.h>

using toponav_core::TopoMap;
using toponav_ros::FlatGroundModule;

DriveTo::DriveTo(NavigationContext& context, Locomotion& locomotion)
    : DrivingStateProto(context), mLocomotion{locomotion} {

    dynamic_reconfigure::Server<arips_navigation::FlatNavigationConfig>::CallbackType cb =
        boost::bind(&DriveTo::onDynamicReconfigure, this, _1, _2);
    mConfigServer.setCallback(cb);
}

bool DriveTo::driveTo(const tf2::Stamped<tf2::Transform>& goal) {
    geometry_msgs::PoseStamped robotPose;
    if (!mContext.globalCostmap.getRobotPose(robotPose)) {
        ROS_WARN_STREAM("Could not get robot pose");
        mLocomotion.cancel(); // make sure robot stops instead of using the old goal
        return false;
    }

    try {
        const auto poseOnFloor =
            mContext.tf.transform(goal, mContext.globalCostmap.getGlobalFrameID());
        const auto planOk =
            mLocomotion.setGoal(Pose2D::fromMsg(robotPose.pose), Pose2D::fromTf(poseOnFloor));

        if (!planOk) {
            ROS_WARN("Could not plan path to goal, clearing global costmap...");
            mContext.globalCostmap.resetLayers();
            if (!mLocomotion.setGoal(Pose2D::fromMsg(robotPose.pose),
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

bool DriveTo::isActive() { return mLocomotion.hasGoal(); }

void DriveTo::runCycle() {
    geometry_msgs::Twist cmd_vel;

    geometry_msgs::PoseStamped robotPoseMsg;
    if (mContext.globalCostmap.getRobotPose(robotPoseMsg)) {
        const auto robotPose = Pose2D::fromMsg(robotPoseMsg.pose);

        if (mLocomotion.goalReached(robotPose)) {
            mLocomotion.cancel();
        } else {
            const auto optTwist = mLocomotion.computeVelocityCommands(robotPose);
            if (optTwist) {
                cmd_vel = optTwist->toTwistMsg();
                mLastControllerSuccessfulTime = ros::Time::now();
            } else {
                ROS_WARN("Could not compute velocity command");
                if (ros::Time::now() >
                    mLastControllerSuccessfulTime + ros::Duration(mConfig.controller_patience)) {
                    doRecovery();
                    mLastControllerSuccessfulTime = ros::Time::now();
                }
            }
        }

    } else {
        ROS_WARN_STREAM("Could not get robot pose");
        mLocomotion.cancel();
    }

    mContext.publishCmdVel(cmd_vel);
}
void DriveTo::onDynamicReconfigure(arips_navigation::FlatNavigationConfig& config, uint32_t level) {
    mConfig = config;
}

void DriveTo::doRecovery() { mContext.localCostmap.resetLayers(); }
