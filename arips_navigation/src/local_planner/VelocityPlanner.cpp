//
// Created by jgdo on 22.03.21.
//

#include <arips_navigation/local_planner/VelocityPlanner.h>

void VelocityPlanner::limitSpeed(geometry_msgs::Twist& cmd_vel, float goalDistance) const {
    double maxSpeed = scaleSpeed(goalDistance);
    const double robotRadius = 0.33/2;

    const double wheelSpeed = std::abs(cmd_vel.linear.x) + std::abs(cmd_vel.angular.z * robotRadius);
    if(wheelSpeed < 0.0001) {
        return;
    }

    const double scalingFactor = maxSpeed / wheelSpeed;

    cmd_vel.linear.x *= scalingFactor;
    cmd_vel.angular.z *= scalingFactor;
}

float VelocityPlanner::scaleSpeed(float goalDistance) const {
    if(goalDistance > mLookaheadStart) {
        return mSpeedMax;
    } else if(goalDistance < mLookaheadLimit) {
        return mSpeedMin;
    } else {
        const float lambda = (goalDistance - mLookaheadLimit) / (mLookaheadStart - mLookaheadLimit);
        return lambda*(mSpeedMax - mSpeedMin) + mSpeedMin;
    }
}

VelocityPlanner::VelocityPlanner(float toleranceX, float toleranceY)
    :mGoalToleranceX(toleranceX)
    ,mGoalToleranceY(toleranceY)
{
}

bool VelocityPlanner::computeVelocity(float x, float y, geometry_msgs::Twist &cmd_vel) const {
    // std::cout << "x: " << x << ", y:" << y << std::endl;

    cmd_vel = geometry_msgs::Twist();

    const float goalDistance = sqrt(x*x +y*y);
    std::cout << "goalDistance: " << goalDistance << std::endl;

    if(std::abs(x) < mGoalToleranceX && std::abs(y) < mGoalToleranceY) {
        return false;
    }

    cmd_vel.linear.x = 1;
    if(std::abs(y) > 0.001) {
        const double r = (x*x + y*y)/(2*y);
        const double rot = 2.0 / r; // 4.0 to keep the robot in line
        //  std::cout << "r = " << r << "rot: " << rot << std::endl;

        cmd_vel.angular.z = rot;
    }

    if(x < 0) {
        cmd_vel.linear.x *= -1;
        cmd_vel.angular.z *= -1;
    }

    // std::cout << "before scaling: vel: " << cmd_vel.linear.x << ", rot: " << cmd_vel.angular.z << std::endl;
    limitSpeed(cmd_vel, goalDistance);

    std::cout << "Final vel: " << cmd_vel.linear.x << ", rot: " << cmd_vel.angular.z << std::endl;

    return true;
}
