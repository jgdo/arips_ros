//
// Created by jgdo on 1/16/21.
//

#include "arips_navigation/TopoExecuter.h"

#include <memory>

#include <arips_navigation/local_planner/arips_planner_ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace toponav_ros;
using namespace toponav_core;

// from https://github.com/strawlab/navigation/blob/master/move_base/src/move_base.cpp
static bool isQuaternionValid(const tf2::Quaternion& tf_q) {
    // first we need to check if the quaternion has nan's or infs
    if (!std::isfinite(tf_q.x()) || !std::isfinite(tf_q.y()) || !std::isfinite(tf_q.z()) ||
        !std::isfinite(tf_q.w())) {
        ROS_ERROR("Quaternion has nans or infs... discarding pose");
        return false;
    }

    // next, we need to check if the length of the quaternion is close to zero
    if (tf_q.length2() < 1e-6) {
        ROS_ERROR("Quaternion has length close to zero... discarding pose");
        return false;
    }

    return true;
}

TopoExecuter::TopoExecuter(tf2_ros::Buffer& tfBuffer, DriveTo& driveTo, ros::Publisher& cmdVelPub,
                           toponav_ros::TopoPlannerROS& topoPlanner)
    : mDriveTo{driveTo}, mCmdVelPub(cmdVelPub), mTopoPlanner(topoPlanner), mTfBuffer{tfBuffer} {
}

void TopoExecuter::activate(const geometry_msgs::PoseStamped& goalMsg) {
    tf2::Stamped<tf2::Transform> pose;
    tf2::fromMsg(goalMsg, pose);

    if (!isQuaternionValid(pose.getRotation()))
        return;

    try {
        geometry_msgs::TransformStamped startTransform;
        startTransform =
            mTfBuffer.lookupTransform("map", "arips_base", ros::Time(0), ros::Duration(0.5));
        geometry_msgs::PoseStamped startPose;
        startPose.header = startTransform.header;
        startPose.pose.position.x = startTransform.transform.translation.x;
        startPose.pose.position.y = startTransform.transform.translation.y;
        startPose.pose.position.z = startTransform.transform.translation.z;
        startPose.pose.orientation = startTransform.transform.rotation;

        GlobalPosition start =
            mTopoPlanner.getContext()
                .poseService->findGlobalPose(startPose, *mTopoPlanner.getContext().topoMap)
                .first;
        ROS_INFO_STREAM("found start node for robot pose: "
                        << (start.node ? start.node->getName() : std::string("<NOT FOUND>")));

        GlobalPosition goal =
            mTopoPlanner.getContext()
                .poseService->findGlobalPose(pose, *mTopoPlanner.getContext().topoMap)
                .first;
        ROS_INFO_STREAM("found goal node for pose: " << (goal.node ? goal.node->getName()
                                                                   : std::string("<NOT FOUND>")));

        if (start.node && goal.node) {
            try {
                TopoPath lastPlan;
                if (mTopoPlanner.getContext().pathPlanner->plan(
                        mTopoPlanner.getContext().topoMap.get(), start, goal, &lastPlan, nullptr)) {
                    mTopoPlanner.getPathViz().visualizePath(lastPlan);
                    setNewPlan(lastPlan);
                    return;
                }
            } catch (const std::exception& e) {
                ROS_ERROR_STREAM("Exception when calling plan: " << e.what());
            }
        }

    } catch (const tf2::TransformException& ex) {
        ROS_WARN("poseCallbackNavGoal(): %s", ex.what());
    }

    // on success the function already returned
    mCurrentPlan.reset();
}

void TopoExecuter::setNewPlan(const toponav_core::TopoPath& plan) {
    // TODO make sure that stopped
    mCurrentPlan = std::make_unique<toponav_core::TopoPath>(plan);
    mCurrentPlanIter = mCurrentPlan->pathElements.begin();
    (*mCurrentPlanIter)->visitPlanVisitor(this);
}

void TopoExecuter::safeStop() { emergencyStop(); }

void TopoExecuter::emergencyStop() {
    // TODO just send stop for now
    geometry_msgs::Twist msg;
    mCmdVelPub.publish(msg);

    mCurrentPlan.reset();
}

void TopoExecuter::runCycle() {
    if (mSegmentExec) {
        const bool segmentDone = mSegmentExec->runCycle(this);
        if (segmentDone) {
            mSegmentExec.reset();

            ++mCurrentPlanIter;
            if (mCurrentPlanIter == mCurrentPlan->pathElements.end()) {
                // full plan is done
                mCurrentPlan.reset();
            } else {
                // visit new segment for crating new segment executor
                // cycle will be run next time
                (*mCurrentPlanIter)->visitPlanVisitor(this);
            }
        }
    } else {
        ROS_WARN("Nothing to execute.");
    }
}

bool TopoExecuter::isActive() { return !!mCurrentPlan; }

void TopoExecuter::visitRegionMovement(const toponav_core::TopoPath::RegionMovement* mov) {
    assert(mov->start.node->getRegionType() == "flat");

    const auto pathPtr = boost::any_cast<std::vector<geometry_msgs::PoseStamped>>(&mov->pathData);
    mDriveTo.followPath(*pathPtr);
    mSegmentExec = std::make_unique<MovementExecuter>(pathPtr);
}

void TopoExecuter::visitTransition(const toponav_core::TopoPath::Transition* transition) {
    assert(transition->topoEdge->getTransitionType() == "step");

    mSegmentExec = std::make_unique<TransitionExecuter>();
}

bool TopoExecuter::MovementExecuter::runCycle(TopoExecuter* parent) {
    parent->mDriveTo.runCycle();
    return !parent->isActive();
}

bool TopoExecuter::TransitionExecuter::runCycle(TopoExecuter* parent) {
    const ros::Time currentTime = ros::Time::now();
    const double sec = (currentTime - m_StartTime).toSec();
    const bool finished = sec > 2.0;

    geometry_msgs::Twist cmd_vel;
    if (!finished) {
        cmd_vel.linear.x = 0.4;
    }
    parent->mCmdVelPub.publish(cmd_vel);
    return finished;
}
