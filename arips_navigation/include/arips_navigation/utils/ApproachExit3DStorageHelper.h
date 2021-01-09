#pragma once

#include <arips_navigation/utils/ApproachLineArea.h>
#include <arips_navigation/utils/FixedPosition.h>
#include <XmlRpcValue.h>

#include <yaml-cpp/yaml.h>
#include <toponav_ros/utils/YamlTools.h>

// See YAML tutorial on https://github.com/jbeder/yaml-cpp/wiki/Tutorial
namespace YAML {
template<>
struct convert<toponav_ros::ApproachExit3DPtr> {
  static Node encode(const toponav_ros::ApproachExit3DPtr &ptr) {
    Node node;
  
    if(toponav_ros::ApproachLineAreaPtr approachLine = std::dynamic_pointer_cast<toponav_ros::ApproachLineArea>(ptr)) {
      auto line = *approachLine;
      node["type"] = "LineArea";
      node["start"] = line.start;
      node["end"] = line.end;
      node["orientation"] = (tf2::Quaternion&)line.orientation;
      node["frame_id"] = line.orientation.frame_id_; // TODO tf2::strip_leading_slash(line.orientation.frame_id_);
    } else if(toponav_ros::FixedPositionPtr approachPose = std::dynamic_pointer_cast<toponav_ros::FixedPosition>(ptr)) {
      node["type"] = "FixedPosition";
      node["pose"] = approachPose->pose;
    } else {
      ROS_ERROR_STREAM("Could not save ApproachLineAreaPtr into a YAML node since concrete subtype is unknown");
    }
    
    return node;
  }
  
  // decoding though ApproachExit3DStorageHelper::parseApproachExit()
};
  
} // namespace YAML

namespace toponav_ros {

struct ApproachExit3DStorageHelper {
  static ApproachExit3DPtr parseApproachExit(YAML::Node const &config, std::string const &errorMessage);
};

} // namespace topo_nav
