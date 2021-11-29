#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Transform.h>

struct FlatPathData {
    std::vector<geometry_msgs::PoseStamped> plan;
    tf2::Stamped<tf2::Transform> actualApproachPose;
};
