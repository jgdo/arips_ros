#include "arips_navigation/local_planner/HPNav.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Twist.h>

HPNav::HPNav(tf2_ros::Buffer *tf, ros::Publisher& cmdVelPub):
        mTfBuffer(tf), mCmdVelPub(cmdVelPub)
{

}

void HPNav::setGoal(const geometry_msgs::PoseStamped &global_pose) {
    mGoal = global_pose;
    mGoalReached = false;
}

bool HPNav::isActive() {
    return !mGoalReached;
}

static float scaleSpeed(float goalDistance) {
    const float lookaheadStart = 0.3;
    const float lookaheadLimit = 0.15;

    const float speedMax = 0.15;
    const float speedMin = 0.05;

    if(goalDistance > lookaheadStart) {
        return speedMax;
    } else if(goalDistance < lookaheadLimit) {
        return speedMin;
    } else {
        const float lambda = (goalDistance - lookaheadLimit) / (lookaheadStart - lookaheadLimit);
        return lambda*(speedMax-speedMin) + speedMin;
    }
}

static void limitSpeed(geometry_msgs::Twist& cmd_vel, float goalDistance) {
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

void HPNav::runCycle() {
    const geometry_msgs::TransformStamped transform = mTfBuffer->lookupTransform("arips_base", mGoal.header.frame_id,  ros::Time(0));
    geometry_msgs::Point gaolPoint;
    tf2::doTransform(mGoal.pose.position, gaolPoint, transform);
    tf2::Vector3 goalPose;
    tf2::fromMsg(gaolPoint, goalPose);

    const double x = goalPose.x(), y = goalPose.y();

    std::cout << "x: " << x << ", y:" << y << std::endl;

    geometry_msgs::Twist cmd_vel;

    const float distTolerance = 0.01;

    const float goalDistance = sqrt(x*x +y*y);
    std::cout << "goalDistance: " << goalDistance << std::endl;

    if(goalDistance < distTolerance) {
        mGoalReached = true;
    } else {
        cmd_vel.linear.x = 1;
        if(std::abs(y) > 0.003) {
            const double r = (x*x + y*y)/(2*y);
            const double rot = 2.0 / r; // 4.0 to keep the robot in line
            std::cout << "r = " << r << "rot: " << rot << std::endl;

            cmd_vel.angular.z = rot;
        }
    }

    if(x < 0) {
        cmd_vel.linear.x *= -1;
        cmd_vel.angular.z *= -1;
    }

    std::cout << "before scaling: vel: " << cmd_vel.linear.x << ", rot: " << cmd_vel.angular.z << std::endl;
    limitSpeed(cmd_vel, goalDistance);

    std::cout << "vel: " << cmd_vel.linear.x << ", rot: " << cmd_vel.angular.z << std::endl;

    mCmdVelPub.publish(cmd_vel);
}

