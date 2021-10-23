#pragma once

#include <Eigen/Core>

#include <angles/angles.h>

using Vector2d = Eigen::Vector2d;

struct Pose2D;
using Twist2D = Pose2D;

struct Pose2D {
    Vector2d point = {0, 0};
    double theta = 0;

    [[nodiscard]] double x() const { return point.x(); }

    [[nodiscard]] double y() const { return point.y(); }

    [[nodiscard]] Pose2D moved(const Twist2D& vel, double dt) const {
        const Vector2d dir = {vel.point.x() * cos(theta) + vel.point.y() * cos(M_PI_2 + theta),
                              vel.point.x() * sin(theta) + vel.point.y() * cos(M_PI_2 + theta)};
        return {point + dir * dt, angles::normalize_angle(theta + vel.theta * dt)};
    }

    /*
    [[nodiscard]] Pose2D operator+(const Pose2D& rhs) const {
        return {point + rhs.point, angles::normalize_angle(theta + rhs.theta)};
    }

    [[nodiscard]] Pose2D& operator+=(const Pose2D& rhs) {
        point += rhs.point;
        theta = angles::normalize_angle(theta + rhs.theta);
    } */

    Pose2D operator*(double scale) const { return {point * scale, theta * scale}; }

    [[nodiscard]] double distance(const Pose2D& other) const {
        return (point - other.point).norm();
    }

    // TODO conversion functions from geometry_msgs::PoseStamped and similar
};

struct TrajectoryPoint {
    Pose2D pose;
    Twist2D velocity;
    double timeFromStart = 0;
};

using Trajectory = std::vector<TrajectoryPoint>;
using Trajectories = std::vector<Trajectory>;