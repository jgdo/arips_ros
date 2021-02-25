//
// Created by jgdo on 2/23/21.
//

#pragma once

#include <tf2_ros/buffer.h>

#include <arips_navigation/DrivingState.h>
#include <geometry_msgs/PoseStamped.h>

/**
 * High-Precision planner
 */
class HPNav: public DrivingState{
public:
    HPNav(tf2_ros::Buffer* tf, ros::Publisher& cmdVelPub);
    ~HPNav() = default;

    bool isActive() override;

    void runCycle() override;

    void setGoal(const geometry_msgs::PoseStamped& global_pose);

private:
    tf2_ros::Buffer* mTfBuffer;
    ros::Publisher& mCmdVelPub;

    geometry_msgs::PoseStamped mGoal;
    bool mGoalReached = true;
};



