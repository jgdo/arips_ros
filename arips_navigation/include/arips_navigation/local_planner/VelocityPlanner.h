#pragma once

#include <geometry_msgs/Twist.h>

class VelocityPlanner {
public:
    explicit VelocityPlanner(float toleranceX, float toleranceY);

    bool computeVelocity(float target_x, float target_y, geometry_msgs::Twist& cmd_vel) const;

private:
    float mGoalToleranceX, mGoalToleranceY;

    const float mLookaheadStart = 0.3;
    const float mLookaheadLimit = 0.15;

    const float mSpeedMax = 0.15;
    const float mSpeedMin = 0.05;

    void limitSpeed(geometry_msgs::Twist& cmd_vel, float goalDistance) const;
    float scaleSpeed(float goalDistance) const;
};
