#pragma once

#include <ros/ros.h>

namespace toponav_ros {

struct MoveGoalProperties {
  double position_tolerance = 0.0;
  double angle_tolerance = 0.0;
  double rotate_to_goal = 0.0;
  
  void readFromParams(std::string const& ns) {
    ros::NodeHandle nh(ns);
    nh.param("position_tolerance", position_tolerance, 0.0);
    nh.param("angle_tolerance", angle_tolerance, 0.0);
    nh.param("rotate_to_goal", rotate_to_goal, 0.0);
  }
};
  
} // namespace topo_nav

