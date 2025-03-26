//
// Created by jgdo on 1/16/21.
//

#include "arips_navigation/TopoExecuter.h"

#include "arips_navigation/path_planning/Costmap2dView.h"

#include <arips_navigation/StepEdgeModule.h>

#include <arips_navigation/utils/transforms.h>

#include <memory>

#include <arips_navigation/utils/FlatPathData.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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

TopoExecuter::TopoExecuter(NavigationContext& context, DriveTo& driveTo,
                           SemanticTopoPlanner& topoPlanner, SemanticMapTracker& mapTracker,
                           CrossDoor& crossDoor, CrossFloorStep& crossStep)
    : DrivingStateProto{context}, mTopoPlanner{topoPlanner}, mMapTracker{mapTracker},
      mDriveTo{driveTo}, mCrossDoor{crossDoor}, mCrossStep{crossStep} {
    ros::NodeHandle nh;
    mTopoPathPub = nh.advertise<nav_msgs::Path>("new_topo_path", 1);
}

void TopoExecuter::activate(const geometry_msgs::PoseStamped& goalMsg) {
    mCurrentPlan.reset();

    geometry_msgs::PoseStamped robotPoseMsg;
    if (!mContext.globalCostmap.getRobotPose(robotPoseMsg)) {
        ROS_ERROR("Could not get robot pose. Cannot execute topo plan");
    }

    const Pose2D startPose = Pose2D::fromMsg(robotPoseMsg.pose);

    try {
        const auto robotPose =
            mContext.tf.transform(goalMsg, mContext.globalCostmap.getGlobalFrameID());
        const Pose2D goalPose = Pose2D::fromMsg(robotPose.pose);

        const auto optPlan =
            mTopoPlanner.plan(Costmap2dView(mContext.globalCostmap),
                              mMapTracker.getLastSemanticMap(), startPose, goalPose);

        if (optPlan) {
            ROS_INFO("Found topo plan");
            visualizePath(*optPlan);
            setNewPlan(*optPlan);
        } else {
            ROS_WARN("Could not find topo plan");
        }
    } catch (const tf2::TransformException& ex) {
        ROS_WARN("poseCallbackNavGoal(): %s", ex.what());
    }
}

void TopoExecuter::setNewPlan(const TopoPath& plan) {
    // TODO make sure that stopped
    mCurrentPlan = std::make_unique<TopoPath>(plan);
    mCurrentPlanIter = mCurrentPlan->pathElements.begin();
    (*mCurrentPlanIter)->visitPlanVisitor(this);
}

void TopoExecuter::safeStop() { emergencyStop(); }

void TopoExecuter::emergencyStop() {
    // TODO just send stop for now
    geometry_msgs::Twist msg;
    publishCmdVel(msg);

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

bool TopoExecuter::isActive() { return mCurrentPlan.operator bool(); }

void TopoExecuter::visitMovement(const TopoPath::Movement* mov) {
    const tf2::Stamped<tf2::Transform> tfGoal{mov->goal.pose.toTf(), ros::Time::now(),
                                              mContext.globalCostmap.getGlobalFrameID()};
    mDriveTo.driveTo(tfGoal);
    mSegmentExec = std::make_unique<MovementExecuter>();
}

void TopoExecuter::visitTransition(const TopoPath::Transition* transition) {
    /*
    const auto diff = stepInfo.end - stepInfo.start;
    const double yaw = atan2(diff.y(), diff.x());

    const tf2::Transform startTrans(createQuaternionFromYaw(yaw), stepInfo.start);

    arips_navigation::CrossDoorInformation doorInfo;
    tf2::toMsg(startTrans, doorInfo.pivotPose.pose);
    doorInfo.pivotPose.header.frame_id = stepInfo.start.frame_id_;

    auto closeApproach =
    toponav_ros::StepEdgeModule::getApproachData(transition->topoEdge)->getCenter();
    closeApproach.setOrigin(closeApproach.getOrigin() * 0.25 + (stepInfo.start + stepInfo.end) *0.5
    * 0.75); tf2::toMsg(closeApproach, doorInfo.approachPose);

    mCrossDoor.activate(doorInfo);
     */

    const auto trans = tryLookupTransform(tf(), localCostmap().getGlobalFrameID(),
                                          globalCostmap().getGlobalFrameID());
    if (!trans) {
        ROS_WARN_STREAM("TopoExecuter::visitTransition cannot transform global to local frame");
        return;
    }

    const auto stepA =
        (*trans)(tf2::Vector3{transition->doorPivot.x(), transition->doorPivot.y(), 0});
    const auto stepB =
        (*trans)(tf2::Vector3{transition->doorExtent.x(), transition->doorExtent.y(), 0});

    mCrossStep.activate({Point2d{stepA.x(), stepA.y()}, Point2d{stepB.x(), stepB.y()}});

    mSegmentExec = std::make_unique<TransitionExecuter>();
}

bool TopoExecuter::MovementExecuter::runCycle(TopoExecuter* parent) {
    parent->mDriveTo.runCycle();
    return !parent->mDriveTo.isActive();
}

bool TopoExecuter::TransitionExecuter::runCycle(TopoExecuter* parent) {
    /*
    if(parent->mCrossDoor.isActive()) {
        parent->mCrossDoor.runCycle();
        return false;
    } else {
        if(m_StartTime.isZero()) {
            m_StartTime = ros::Time::now();
        }

        const ros::Time currentTime = ros::Time::now();
        const double sec = (currentTime - m_StartTime).toSec();
        const bool finished = sec > 2.1;

        geometry_msgs::Twist cmd_vel;
        if (!finished) {
            cmd_vel.linear.x = 0.5;
        }

        parent->publishCmdVel(cmd_vel);
        return finished;
    }
     */

    if (parent->mCrossStep.isActive()) {
        parent->mCrossStep.runCycle();
        return false;
    } else {
        parent->publishCmdVel({});
        return true;
    }
}

void TopoExecuter::visualizePath(const TopoPath& path) const {
    nav_msgs::Path navPath;

    navPath.header.frame_id = mContext.globalCostmap.getGlobalFrameID();
    navPath.header.stamp = ros::Time::now();

    ::LambdaPlanVisitor visitor(
        [&, this](::TopoPath::Movement const* mov) {
            for (const auto& p : mov->pathPoints) {
                geometry_msgs::PoseStamped poseStampedMsg;
                poseStampedMsg.pose = p.toPoseMsg();
                poseStampedMsg.header = navPath.header;
                navPath.poses.push_back(poseStampedMsg);
            }
        },
        [&, this](::TopoPath::Transition const* trans) {

        });

    path.visitPlan(visitor);

    mTopoPathPub.publish(navPath);
}
