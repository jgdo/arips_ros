#pragma once

#include <Eigen/Core>

#include <angles/angles.h>
#include <geometry_msgs/Twist.h>

#include <tf2/utils.h>

#include <arips_navigation/utils/transforms.h>

using Vector2d = Eigen::Vector2d;

struct Pose2D;
using Twist2D = Pose2D;

struct Pose2D {
    Vector2d point = {0, 0};
    double theta = 0;

    Pose2D() = default;
    Pose2D(const Pose2D& other) = default;
    Pose2D(const Vector2d& point, double theta) : point{point}, theta{theta} {}

    static Pose2D fromMsg(const geometry_msgs::Twist& msg) {
        return {{msg.linear.x, msg.linear.y}, msg.angular.z};
    }

    static Pose2D fromMsg(geometry_msgs::Pose const& msg) {
        return {{msg.position.x, msg.position.y}, getYawFromQuaternion(msg.orientation)};
    }

    static Pose2D fromTf(const tf2::Transform& trans) {
        return {{trans.getOrigin().x(), trans.getOrigin().y()}, tf2::getYaw(trans.getRotation())};
    }


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

    geometry_msgs::Twist toTwistMsg() const {
        geometry_msgs::Twist msg;
        msg.linear.x = point.x();
        msg.linear.y = point.y();
        msg.angular.z = theta;
        return msg;
    }

    geometry_msgs::Pose toPoseMsg() const {
        geometry_msgs::Pose pose;
        pose.position.x = point.x();
        pose.position.y = point.y();
        pose.orientation = createQuaternionMsgFromYaw(theta);
        return pose;
    }
};

struct TrajectoryPoint {
    Pose2D pose;
    Twist2D velocity;
    double timeFromStart = 0;
};

using Trajectory = std::vector<TrajectoryPoint>;
using Trajectories = std::vector<Trajectory>;
