#pragma once

#include <toponav_core/ModuleContainer.h>

#include <ros/ros.h>
#include <toponav_ros/interfaces/NodeVisualizationInterface.h>
#include <toponav_ros/interfaces/EdgeVisualizationInterface.h>
#include <toponav_core/TopoPath.h>
#include <toponav_ros/RosContext.h>

namespace toponav_ros {

class TopoPathVisualizer {
public:
	TopoPathVisualizer(toponav_core::ModuleContainer& factory, RosContext& context);
	
	void visualizePath(toponav_core::TopoPath const& path);
	
	void fillNavPath(toponav_core::TopoPath const& path, nav_msgs::Path* navPath);

private:
	NodeVisualizationInterface::Ptr nodeViz;
	EdgeVisualizationInterface::Ptr edgeViz;
	
	RosContext& _context;
	
	ros::Publisher _planPub, _poseArrayPub;
	
	
	// visualization_msgs::MarkerArray pathMarkers;
	nav_msgs::Path pathMsg;
};
	
} // namespace topo_nav

