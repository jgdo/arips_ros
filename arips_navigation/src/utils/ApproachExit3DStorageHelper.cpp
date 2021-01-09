#include <arips_navigation/utils/ApproachExit3DStorageHelper.h>
#include <toponav_ros/TopoMapYamlStorage.h>
#include <toponav_ros/utils/YamlTools.h>

namespace toponav_ros {

ApproachExit3DPtr
ApproachExit3DStorageHelper::parseApproachExit(YAML::Node const &config, std::string const &errorMessage) {
  std::string type = config["type"].as<std::string>();
  
  if(type == "LineArea") {
    tf2::Vector3 start, end;
    tf2::Stamped<tf2::Quaternion> rot;
    
    rot.frame_id_ = YamlTools::parseFrameID(config["frame_id"], errorMessage);
    rot.stamp_ = ros::Time::now();
    rot.setData(config["orientation"].as<tf2::Quaternion>());
    
    start = config["start"].as<tf2::Vector3>();
    end = config["end"].as<tf2::Vector3>();
    return ApproachLineArea::create(start, end, rot);
  } else if(type == "FixedPosition") {
    return std::make_shared<FixedPosition>(YamlTools::parsePose(config["pose"], errorMessage));
  } else
    return ApproachExit3DPtr();
}

}