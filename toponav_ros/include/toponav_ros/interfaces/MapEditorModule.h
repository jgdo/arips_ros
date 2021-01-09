#pragma once

#include <boost/any.hpp>

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <memory>

#include <toponav_core/TopoMap.h>

namespace toponav_ros {

class MapEditorModule {
public:
	typedef MapEditorModule BaseClass;
	typedef std::shared_ptr<MapEditorModule> Ptr;
	
	virtual void activate() = 0;
	virtual void deactivate() = 0;
	
	virtual void poseCallback(geometry_msgs::PoseStamped const& pose) = 0;
};

typedef MapEditorModule::Ptr MapEditorModulePtr;
	
} // namespace topo_nav
