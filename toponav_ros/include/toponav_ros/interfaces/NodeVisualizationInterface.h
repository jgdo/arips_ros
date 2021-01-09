#pragma once

#include <boost/any.hpp>

#include <interactive_markers/interactive_marker_server.h>
#include <string>
#include <memory>

#include <toponav_core/TopoMap.h>

#include <boost/optional.hpp>
#include <nav_msgs/Path.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>

namespace toponav_ros {

class MapEditor;

class NodeVisualizationInterface {
public:
	typedef NodeVisualizationInterface BaseClass;
	typedef std::shared_ptr<NodeVisualizationInterface> Ptr;
	
	typedef boost::optional<tf2::Stamped<tf2::Transform>> OptionalPoseStamped;
	
	/**
	 * May only add menus to the meny handlers, no interactive markers!
	 * @param editor
	 */
	virtual inline void initializeVisualization(MapEditor *editor) {
		if(this->mapEditor) {
			throw std::runtime_error("EdgeVisualizationInterface::initialize(): already initialized");
		}
		
		this->mapEditor = editor;
	}

	virtual void vizualizeNode(const toponav_core::TopoMap::Node *node) = 0;
	
	virtual void appendRegionEdgeToPlan(nav_msgs::Path *pathPtr, std::string regionType, boost::any const &pathData) = 0;

protected:
  MapEditor* mapEditor = nullptr;
};

typedef NodeVisualizationInterface::Ptr NodeVisualizationInterfacePtr;

} // namespace topo_nav
