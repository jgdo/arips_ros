#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <global_planner/planner_core.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <arips_navigation/path_planning/PotentialMap.h>
#include <arips_navigation/path_planning/PotentialMapVisualizer.h>
#include <visualization_msgs/MarkerArray.h>

#include <arips_navigation/utils/transforms.h>
#include <geometry_msgs/Twist.h>

#include <angles/angles.h>

class MotionController {
public:
    explicit MotionController(PotentialMap& potentialMap);
    ~MotionController();

    bool goalReached(const geometry_msgs::PoseStamped& gaolPose);

    std::optional<geometry_msgs::Twist> computeVelocity(const geometry_msgs::PoseStamped& goalPose);

private:
    struct Pimpl;
    std::unique_ptr<Pimpl> mPimpl;
};