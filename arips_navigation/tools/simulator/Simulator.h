#pragma once

#include <memory>

#include <tf2_ros/buffer.h>

#include <arips_navigation/path_planning/PlanningMath.h>

class Simulator {
public:
    explicit Simulator(tf2_ros::Buffer& tf, bool runInOwnThread = true);
    ~Simulator();

    [[nodiscard]] Pose2D getRobotPose() const;

    // dt in seconds
    void makeStep(double dt, ros::Time now);

    void setSpeed(const Pose2D& twist);

private:
    struct Pimpl;
    std::unique_ptr<Pimpl> mPimpl;
};
