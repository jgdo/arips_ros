#pragma once

#include <Eigen/Core>

#include <angles/angles.h>

#include <tf2/utils.h>

#include <arips_tf/transforms.h>

using Vector2d = Eigen::Vector2d;
using Point2d = Eigen::Vector2d;

struct Pose2D {
    Vector2d point = {0, 0};
    double theta = 0;

    Pose2D() = default;
    Pose2D(const Pose2D& other) = default;
    Pose2D(const Vector2d& point, double theta) : point{point}, theta{theta} {}

    static Pose2D fromMsg(geometry_msgs::Pose const& msg) {
        return {{msg.position.x, msg.position.y}, getYawFromQuaternion(msg.orientation)};
    }

    static Pose2D fromTf(const tf2::Transform& trans) {
        return {{trans.getOrigin().x(), trans.getOrigin().y()}, tf2::getYaw(trans.getRotation())};
    }


    [[nodiscard]] double x() const { return point.x(); }

    [[nodiscard]] double y() const { return point.y(); }

  

    /*
    [[nodiscard]] Pose2D operator+(const Pose2D& rhs) const {
        return {point + rhs.point, angles::normalize_angle(theta + rhs.theta)};
    }

    [[nodiscard]] Pose2D& operator+=(const Pose2D& rhs) {
        point += rhs.point;
        theta = angles::normalize_angle(theta + rhs.theta);
    } */

    Pose2D operator*(double scale) const { return {point * scale, theta * scale}; }

    [[nodiscard]] double distance(const Point2d & other) const {
        return (point - other).norm();
    }

    geometry_msgs::Pose toPoseMsg() const {
        geometry_msgs::Pose pose;
        pose.position.x = point.x();
        pose.position.y = point.y();
        pose.orientation = createQuaternionMsgFromYaw(theta);
        return pose;
    }
};
