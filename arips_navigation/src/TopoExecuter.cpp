//
// Created by jgdo on 1/16/21.
//

#include "arips_navigation/TopoExecuter.h"

#include <memory>
#include <base_local_planner/trajectory_planner_ros.h>
#include <arips_local_planner/arips_planner_ros.h>


TopoExecuter::TopoExecuter(tf2_ros::Buffer &tfBuffer, costmap_2d::Costmap2DROS &costmap) {
    ros::NodeHandle nh;
    mCmdVelPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1, false);
    //mLocalPlanner = std::make_unique<base_local_planner::TrajectoryPlannerROS>();
    //mLocalPlanner->initialize("TrajectoryPlannerROS", &tfBuffer, &costmap);
    mLocalPlanner = std::make_unique<arips_local_planner::AripsPlannerROS>();
    mLocalPlanner->initialize("AripsPlannerROS", &tfBuffer, &costmap);
}


void TopoExecuter::setNewPlan(const toponav_core::TopoPath &plan) {
    // TODO make sure that stopped
    mCurrentPlan = std::make_unique<toponav_core::TopoPath>(plan);
    mCurrentPlanIter = mCurrentPlan->pathElements.begin();
    (*mCurrentPlanIter)->visitPlanVisitor(this);
}

void TopoExecuter::safeStop() {
    emergencyStop();
}

void TopoExecuter::emergencyStop() {
    // TODO just send stop for now
    geometry_msgs::Twist msg;
    mCmdVelPub.publish(msg);

    mCurrentPlan.reset();
}

void TopoExecuter::runControlCycle() {
    if(mSegmentExec) {
        const bool segmentDone = mSegmentExec->runCycle(this);
        if(segmentDone) {
            mSegmentExec.reset();

            ++mCurrentPlanIter;
            if(mCurrentPlanIter == mCurrentPlan->pathElements.end()) {
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

bool TopoExecuter::isRunning() {
    return !!mCurrentPlan;
}

void TopoExecuter::visitRegionMovement(const toponav_core::TopoPath::RegionMovement *mov) {
    assert(mov->start.node->getRegionType() == "flat");

    const auto pathPtr = boost::any_cast<std::vector<geometry_msgs::PoseStamped>>(&mov->pathData);
    mLocalPlanner->setPlan(*pathPtr);
    mSegmentExec = std::make_unique<MovementExecuter>(pathPtr);
}

void TopoExecuter::visitTransition(const toponav_core::TopoPath::Transition *transition) {
    assert(transition->topoEdge->getTransitionType() == "step");

    mSegmentExec = std::make_unique<TransitionExecuter>();
}

bool TopoExecuter::MovementExecuter::runCycle(TopoExecuter * parent) {
    geometry_msgs::Twist cmd_vel;

    const bool finished = parent->mLocalPlanner->isGoalReached();

    if(!finished) {
        parent->mLocalPlanner->computeVelocityCommands(cmd_vel);
    }
    parent->mCmdVelPub.publish(cmd_vel);
    return finished;
}

bool TopoExecuter::TransitionExecuter::runCycle(TopoExecuter* parent) {
    const ros::Time currentTime = ros::Time::now();
    const double sec = (currentTime - m_StartTime).toSec();
    const bool finished = sec > 2.0;

    geometry_msgs::Twist cmd_vel;
    if(!finished) {
        cmd_vel.linear.x = 0.4;
    }
    parent->mCmdVelPub.publish(cmd_vel);
    return finished;
}
