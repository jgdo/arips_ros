#pragma once

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>

#include <memory>

#include <toponav_core/TopoMap.h>

namespace toponav_ros {

class MapPoseInterface {
public:
	typedef MapPoseInterface BaseClass;
	typedef std::shared_ptr<MapPoseInterface> Ptr;
  
  inline std::pair<toponav_core::GlobalPosition, tf2::Stamped<tf2::Transform>> findGlobalPose(geometry_msgs::PoseStamped const &poseMsg, const toponav_core::TopoMap &map) {
    tf2::Stamped<tf2::Transform> pose;
    tf2::convert(poseMsg, pose);
    return findGlobalPose(pose, map);
  };
  
	/**
	 * @return node and distance
	 */
	virtual std::pair<toponav_core::GlobalPosition, tf2::Stamped<tf2::Transform>> findGlobalPose(tf2::Stamped<tf2::Transform> const &pose,
                                                                                const toponav_core::TopoMap &map) = 0;
  
  virtual tf2::Stamped<tf2::Transform> getMapPoseFromPosition(toponav_core::GlobalPosition const& pos) = 0;
};

typedef MapPoseInterface::Ptr MapPosePluginPtr;
	
} // namespace topo_nav
