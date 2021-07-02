#pragma once

#include <arips_navigation/NavigationContext.h>

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

/**
 * Represents the execution of something, e.g. navigation or docking.
 */
class DrivingStateProto: public DrivingState {
public:
    explicit DrivingStateProto(NavigationContext& ctx) : mContext{ctx} {}

    inline void publishCmdVel(const geometry_msgs::Twist& cmd_vel) {
        mContext.publishCmdVel(cmd_vel);
    }

    [[nodiscard]] const tf2_ros::Buffer& tf() const { return mContext.tf; }
    [[nodiscard]] costmap_2d::Costmap2DROS& globalCostmap() { return mContext.globalCostmap; }
    [[nodiscard]] costmap_2d::Costmap2DROS& localCostmap() { return mContext.localCostmap; }

protected:
    NavigationContext& mContext;
    ros::NodeHandle mNodeHandle;
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