#pragma once

#include <ros/ros.h>
#include <tf2_ros/buffer.h>

/**
 * Represents the execution of something, e.g. navigation or docking.
 */
class DrivingState{
public:
    inline virtual ~DrivingState() = default;
    
    /**
     * @return true iff currently following a plan.
     */
    virtual bool isActive() = 0;

    /**
     * Run one control cycle given current plan and robot state.
     */
    virtual void runCycle() = 0;
};

class DrivingStateProto: public DrivingState {
public:
    DrivingStateProto(tf2_ros::Buffer& tf, ros::Publisher& cmdVelPub)
        : mTfBuffer(tf)
        , mCmdVelPub(cmdVelPub)
    {}

protected:
    ros::NodeHandle mNodeHandle;
    tf2_ros::Buffer& mTfBuffer;
    ros::Publisher& mCmdVelPub;

    virtual void init() {} // overrideable ctor substitute to avoid writing params for actual ctor
};

/*
template<class Parent, class Pimpl>
class DrivingStatePimpl: public DrivingState {
public:
    DrivingStatePimpl(tf2_ros::Buffer& tf, ros::Publisher& cmdVelPub)
    : pimpl {tf, cmdVelPub}
    {}

    bool isActive() override {
        return Parent::pimpl->isActive();
    }

    void runCycle() override {
        return Parent::pimpl->runCycle();
    }

protected:

};
*/