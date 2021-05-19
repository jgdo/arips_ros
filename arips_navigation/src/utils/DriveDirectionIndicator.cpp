#include <arips_navigation/utils/DriveDirectionIndicator.h>

#include <ros/ros.h>

#include <tf2/LinearMath/Transform.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

DriveDirectionIndicator::DriveDirectionIndicator(const std::string& name, double robotRadius)
    : mRobotRadius{robotRadius} {
    ros::NodeHandle nh;
    mPub = nh.advertise<visualization_msgs::Marker>(name, 1, true);
}

void DriveDirectionIndicator::indicate(geometry_msgs::Twist const& twist) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "velocity_marker";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.color.r = 1;
    marker.color.a = 1;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.01;

    auto toPointMsg = [](const tf2::Vector3& v) {
      geometry_msgs::Point msg;
      msg.x = v.x();
      msg.y = v.y();
      msg.z = v.z();
      return msg;
    };

    // transform between individual steps
    const double forward_time = 2.0;
    const int steps = 8;
    const auto stepTrans = tf2::Transform{
        tf2::Quaternion(tf2::Vector3{0, 0, 1}, twist.angular.z * forward_time / steps),
        tf2::Vector3{twist.linear.x, 0, 0} * forward_time / steps};

    const tf2::Vector3 leftOffset = {0, mRobotRadius, 0};
    const tf2::Vector3 rightOffset = {0, -mRobotRadius, 0};

    // running robot center pose
    auto robotPos = tf2::Transform::getIdentity();
    auto left = leftOffset;
    auto right = rightOffset;
    // iteratively move robot and add sides to marker
    for (int i = 0; i < steps; i++) {
        robotPos = stepTrans * robotPos;
        const auto nextLeft = robotPos * leftOffset;
        const auto nextRight = robotPos * rightOffset;

        marker.points.push_back(toPointMsg(left));
        marker.points.push_back(toPointMsg(nextLeft));
        marker.points.push_back(toPointMsg(right));
        marker.points.push_back(toPointMsg(nextRight));

        left = nextLeft;
        right = nextRight;
    }

    mPub.publish(marker);
}
