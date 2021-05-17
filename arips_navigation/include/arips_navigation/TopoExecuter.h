//
// Created by jgdo on 1/16/21.
//

#pragma once

#include <memory>

#include <ros/ros.h>
#include <toponav_core/TopoPath.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>


#include <arips_navigation/DrivingState.h>
#include <arips_navigation/DriveTo.h>
#include <toponav_ros/TopoPlannerROS.h>


/**
 * Responsible for executing a planned topo path
 */
class TopoExecuter: public DrivingState, private toponav_core::TopoPath::PathVisitor  {
public:

    TopoExecuter(tf2_ros::Buffer& tfBuffer, DriveTo& driveTo,
        ros::Publisher& cmdVelPub, toponav_ros::TopoPlannerROS& topoPlanner);

    void activate(const geometry_msgs::PoseStamped &goalMsg);

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
        virtual bool runCycle(TopoExecuter* ) = 0;
        virtual ~SegmentExecuter() = default;
    };

    struct MovementExecuter: public SegmentExecuter {
        explicit MovementExecuter(const std::vector<geometry_msgs::PoseStamped>* path):
                mPath(path) {}

        bool runCycle(TopoExecuter* ) override;

        const std::vector<geometry_msgs::PoseStamped>* const mPath;
    };

    struct TransitionExecuter: public SegmentExecuter {
        ros::Time m_StartTime = ros::Time::now();

        bool runCycle(TopoExecuter* ) override;
    };

    DriveTo& mDriveTo;

    std::unique_ptr<toponav_core::TopoPath> mCurrentPlan;
    std::vector<toponav_core::TopoPath::PathSegment::Ptr>::iterator mCurrentPlanIter; /// only valid if mCurrentPlan valid

    ros::Publisher& mCmdVelPub;

    void visitRegionMovement(toponav_core::TopoPath::RegionMovement const* movement) override;
    void visitTransition(toponav_core::TopoPath::Transition const* transition) override;

    std::unique_ptr<SegmentExecuter> mSegmentExec;

    toponav_ros::TopoPlannerROS& mTopoPlanner;
    tf2_ros::Buffer &mTfBuffer;
};



