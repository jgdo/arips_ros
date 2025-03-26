//
// Created by jgdo on 1/16/21.
//

#pragma once

#include <memory>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include "CrossFloorStep.h"
#include "topo_nav/SemanticMapTracker.h"
#include <arips_navigation/CrossDoor.h>
#include <arips_navigation/DriveTo.h>
#include <arips_navigation/DrivingState.h>
#include <arips_navigation/topo_nav/SemanticTopoPlanner.h>

/**
 * Responsible for executing a planned topo path
 */
class TopoExecuter : public DrivingStateProto, private TopoPath::PathVisitor {
public:
    TopoExecuter(NavigationContext& context, DriveTo& driveTo, SemanticTopoPlanner& topoPlanner,
                 SemanticMapTracker& mapTracker, CrossDoor& crossDoor, CrossFloorStep& crossStep);

    void activate(const geometry_msgs::PoseStamped& goalMsg);

    /**
     * Set new plan for execution. Assumes that current state is idle.
     * @param plan
     */
    void setNewPlan(const TopoPath& plan);

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

    void visualizePath(const TopoPath& path) const;

    DriveTo& mDriveTo;

    std::unique_ptr<TopoPath> mCurrentPlan;
    std::vector<TopoPath::PathSegment::Ptr>::iterator
        mCurrentPlanIter; /// only valid if mCurrentPlan valid

    void visitMovement(TopoPath::Movement const* movement) override;
    void visitTransition(TopoPath::Transition const* transition) override;

    std::unique_ptr<SegmentExecuter> mSegmentExec;

    SemanticTopoPlanner& mTopoPlanner;
    SemanticMapTracker& mMapTracker;

    CrossDoor& mCrossDoor;
    CrossFloorStep& mCrossStep;

    ros::Publisher mTopoPathPub;
};
