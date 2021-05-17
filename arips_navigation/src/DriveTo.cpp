#include <utility>

#include <arips_navigation/DriveTo.h>
#include <arips_navigation/local_planner/arips_planner_ros.h>

#include <arips_navigation/FlatGroundModule.h>
#include <arips_navigation/utils/FixedPosition.h>

using toponav_core::TopoMap;
using toponav_ros::FlatGroundModule;

DriveTo::DriveTo(tf2_ros::Buffer& tf, ros::Publisher& cmdVelPub, toponav_core::TopoMapPtr topoMap,
                 costmap_2d::Costmap2DROS& localCostmap)
    : DrivingStateProto(tf, cmdVelPub), mTopoMap{std::move(topoMap)}, mLocalCostmap{localCostmap} {

    ros::NodeHandle nh;
    mLocalPlanner = std::make_unique<arips_local_planner::AripsPlannerROS>();
    mLocalPlanner->initialize("AripsPlannerROS", &mTfBuffer, &mLocalCostmap);

    dynamic_reconfigure::Server<arips_navigation::FlatNavigationConfig>::CallbackType cb =
        boost::bind(&DriveTo::onDynamicReconfigure, this, _1, _2);
    mConfigServer.setCallback(cb);
}

bool DriveTo::driveTo(const tf2::Stamped<tf2::Transform>& goal) {
    mCurrentPath = planTo(goal);
    return followPath(mCurrentPath);
}

std::vector<geometry_msgs::PoseStamped> DriveTo::planTo(const tf2::Stamped<tf2::Transform>& goal) {
    std::vector<geometry_msgs::PoseStamped> path;

    if (goal.frame_id_.empty()) {
        ROS_ERROR("Pose cannot have an empty frame_id");
        return path;
    }

    auto* planner = FlatGroundModule::getMapData(mTopoMap.get()).planner;

    tf2::Stamped<tf2::Transform> poseOnFloor;

    try {
        poseOnFloor = mTfBuffer.transform(goal, planner->getMap().getGlobalFrameID());
    } catch (const tf2::TransformException& ex) {
        ROS_WARN("DriveTo::driveTo(): %s", ex.what());
        return path;
    }

    geometry_msgs::PoseStamped start;
    if (!planner->getMap().getRobotPose(start)) {
        ROS_ERROR("Could not get current robot pose");
        return path;
    }

    if (!planner->makePlan(start, toponav_ros::FixedPosition::create(goal), path) || path.empty()) {
        ROS_WARN("Could not plan path to goal, clearing global costmap...");
        planner->getMap().resetLayers();
        if (!planner->makePlan(start, toponav_ros::FixedPosition::create(goal), path) ||
            path.empty()) {
            ROS_ERROR("Could not plan path to goal even after clearing the global costmap");
            path.clear();
        }
    }
    return path;
}

bool DriveTo::followPath(const std::vector<geometry_msgs::PoseStamped>& path) {
    mCurrentPath = path;

    if (mCurrentPath.empty()) {
        ROS_WARN("Cannot follow empty path");
        return false;
    }

    mLocalPlanner->setPlan(mCurrentPath);
    mLastControllerSuccessfulTime = ros::Time::now();
    return true;
}

bool DriveTo::isActive() {
    return !mCurrentPath.empty();
}

void DriveTo::runCycle() {
    geometry_msgs::Twist cmd_vel;

    const bool finished = mLocalPlanner->isGoalReached();

    if (finished) {
        mCurrentPath.clear();
    } else {
        const auto canDrive = mLocalPlanner->computeVelocityCommands(cmd_vel);
        if (canDrive) {
            mLastControllerSuccessfulTime = ros::Time::now();
        } else if (ros::Time::now() >
                   mLastControllerSuccessfulTime + ros::Duration(mConfig.controller_patience)) {
            doRecovery();
            mLastControllerSuccessfulTime = ros::Time::now();
        }
    }
    mCmdVelPub.publish(cmd_vel);
}
void DriveTo::onDynamicReconfigure(arips_navigation::FlatNavigationConfig& config, uint32_t level) {
    mConfig = config;
}

void DriveTo::doRecovery() { mLocalCostmap.resetLayers(); }
