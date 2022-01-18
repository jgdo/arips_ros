#pragma once

#include <ros/ros.h>

#include <arips_navigation/path_planning/PotentialMap.h>

#include <arips_navigation/MotionControllerConfig.h>

#include "PlanningMath.h"

#include "Costmap.h"

// Stateless motion controller
class MotionController {
public:
    explicit MotionController();
    ~MotionController();

    bool goalReached(const Pose2D& robotPose, const Pose2D& gaolPose);

    std::optional<Twist2D> computeVelocity(const NavMapView& costmap, const Odom2D& robotPose, const Pose2D& goalPose);

private:
    struct Pimpl;
    std::unique_ptr<Pimpl> mPimpl;
};

Trajectory generateTrajectory(Pose2D currentPose, const Pose2D& vel, double duration, double dt);

Trajectories sampleTrajectories(const Pose2D& currentPose, double goalDistance,
                                const CostFunction& costFunction,
                                const arips_navigation::MotionControllerConfig& config);