#include <arips_navigation/DriveTo.h>
#include <arips_navigation/local_planner/arips_planner_ros.h>

#include <arips_navigation/FlatGroundModule.h>
#include <arips_navigation/utils/FixedPosition.h>

#include <utility>

using toponav_core::TopoMap;
using toponav_ros::FlatGroundModule;

DriveTo::DriveTo(tf2_ros::Buffer& tf, ros::Publisher& cmdVelPub, toponav_core::TopoMapPtr topoMap,
                 costmap_2d::Costmap2DROS& localCostmap)
    : DrivingStateProto(tf, cmdVelPub), mTopoMap{std::move(topoMap)}, mLocalCostmap{localCostmap} {

    ros::NodeHandle nh;
    mLocalPlanner = std::make_unique<arips_local_planner::AripsPlannerROS>();
    mLocalPlanner->initialize("AripsPlannerROS", &mTfBuffer, &mLocalCostmap);
}

bool DriveTo::driveTo(const tf2::Stamped<tf2::Transform>& goal) {
    mCurrentPath = planTo(goal);
    if (mCurrentPath.empty()) {
        return false;
    }

    mLocalPlanner->setPlan(mCurrentPath);
    return true;
}

std::vector<geometry_msgs::PoseStamped> DriveTo::planTo(const tf2::Stamped<tf2::Transform>& goal) {
    std::vector<geometry_msgs::PoseStamped> path;

    if (goal.frame_id_.empty()) {
        ROS_ERROR("Pose cannot have an empty frame_id");
        return path;
    }

    tf2::Stamped<tf2::Transform> poseOnFloor;
    TopoMap::Node const* realNode = nullptr;

    FlatGroundModule::CostsPlanner* planner = nullptr;

    for (auto& mapEntry : FlatGroundModule::getMapData(mTopoMap.get())) {
        const auto& globalCostmap = mapEntry.second.planner->getMap();

        try {
            poseOnFloor = mTfBuffer.transform(goal, globalCostmap.getGlobalFrameID());
        } catch (const tf2::TransformException& ex) {
            ROS_WARN("DriveTo::driveTo(): %s", ex.what());
            continue;
        }

        // FIXME: better check if this is the costmap
        if (std::abs(poseOnFloor.getOrigin().z()) < 0.5) {
            // Found planner for this floor
            planner = mapEntry.second.planner.get();
            break;
        }
    }

    if (!planner) {
        ROS_ERROR("Could not find planner for goal pose");
        return path;
    }

    geometry_msgs::PoseStamped start;
    if (!planner->getMap().getRobotPose(start)) {
        ROS_ERROR("Could not get current robot pose");
        return path;
    }

    if (!planner->makePlan(start, toponav_ros::FixedPosition::create(goal), path) || path.empty()) {
        ROS_ERROR("Could not plan path to goal");
        path.clear();
    }

    return path;
}

void DriveTo::followPath(const std::vector<geometry_msgs::PoseStamped>& path) {
    mCurrentPath = path;

    if (mCurrentPath.empty()) {
        ROS_WARN("Cannot follow empty path");
        return;
    }

    mLocalPlanner->setPlan(mCurrentPath);
}

bool DriveTo::isActive() { return !mCurrentPath.empty(); }

void DriveTo::runCycle() {
    geometry_msgs::Twist cmd_vel;

    const bool finished = mLocalPlanner->isGoalReached();

    if (finished) {
        mCurrentPath.clear();
    } else {
        mLocalPlanner->computeVelocityCommands(cmd_vel);
    }
    mCmdVelPub.publish(cmd_vel);
}
