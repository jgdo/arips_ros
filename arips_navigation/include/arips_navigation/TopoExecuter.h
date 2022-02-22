//
// Created by jgdo on 1/16/21.
//

#pragma once

#include <memory>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <toponav_core/TopoPath.h>

#include <arips_navigation/CrossDoor.h>
#include <arips_navigation/DriveTo.h>
#include <arips_navigation/DrivingState.h>
#include <toponav_ros/TopoPlannerROS.h>
#include "CrossFloorStep.h"

/**
 * Responsible for executing a planned topo path
 */
class TopoExecuter : public DrivingStateProto, private toponav_core::TopoPath::PathVisitor {
public:
    TopoExecuter(NavigationContext& context, DriveTo& driveTo,
                 toponav_ros::TopoPlannerROS& topoPlanner, CrossDoor& crossDoor, CrossFloorStep& crossStep);

    void activate(const geometry_msgs::PoseStamped& goalMsg);

    /**
     * Set new plan for execution. Assumes that current state is idle.
     * @param plan
     */
    void setNewPlan(const toponav_core::TopoPath& plan);

    /**
     * Safely stop the robot. This might need a view control cycles to finish.
     */
    void safeStop();

    /**
     * Stops robot immediately, even if this will result in an inconvenient physical state.
     */
    void emergencyStop();

    void runCycle() override;
    bool isActive() override;

private:
    struct SegmentExecuter {
        virtual bool runCycle(TopoExecuter*) = 0;
        virtual ~SegmentExecuter() = default;
    };

    struct MovementExecuter : public SegmentExecuter {
        bool runCycle(TopoExecuter*) override;
    };

    struct TransitionExecuter : public SegmentExecuter {
        ros::Time m_StartTime;

        bool runCycle(TopoExecuter*) override;
    };

    DriveTo& mDriveTo;

    std::unique_ptr<toponav_core::TopoPath> mCurrentPlan;
    std::vector<toponav_core::TopoPath::PathSegment::Ptr>::iterator
        mCurrentPlanIter; /// only valid if mCurrentPlan valid

    void visitRegionMovement(toponav_core::TopoPath::RegionMovement const* movement) override;
    void visitTransition(toponav_core::TopoPath::Transition const* transition) override;

    std::unique_ptr<SegmentExecuter> mSegmentExec;

    toponav_ros::TopoPlannerROS& mTopoPlanner;

    CrossDoor& mCrossDoor;
    CrossFloorStep& mCrossStep;
};
