#pragma once

#include <toponav_ros/interfaces/NodeVisualizationInterface.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>


#include <toponav_ros/PlanningContext.h>
#include <arips_navigation/utils/ApproachLineArea.h>
#include "FlatGroundModule.h"

namespace toponav_ros {

class FlatNodeVisualizer: public NodeVisualizationInterface {
public:
  inline FlatNodeVisualizer(PlanningContext& context, std::string const& nodeType): _context(context), own_node_type_(nodeType) {}
	
	virtual void initializeVisualization(MapEditor *editor);
  
	virtual void vizualizeNode(const toponav_core::TopoMap::Node *node) override;

	

	virtual void appendRegionEdgeToPlan(nav_msgs::Path *pathPtr, std::string regionType,
										boost::any const &pathData) override;

protected:
  void segmentLayerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, std::string mapName);
  
private:
  PlanningContext& _context;
	std::string own_node_type_;
};

} // namespace topo_nav
