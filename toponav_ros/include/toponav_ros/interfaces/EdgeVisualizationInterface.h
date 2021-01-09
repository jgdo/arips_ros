#pragma once

#include <boost/any.hpp>

#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <string>
#include <memory>
#include <interactive_markers/interactive_marker_server.h>

#include <toponav_core/TopoMap.h>

namespace toponav_ros {

class MapEditor;

class EdgeVisualizationInterface {
public:
	typedef EdgeVisualizationInterface BaseClass;
	typedef std::shared_ptr<EdgeVisualizationInterface> Ptr;
	
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
	
	virtual void beginMapVisualization() {}
	virtual void endMapVisualization() {}

	virtual void visualizeEdge(const toponav_core::TopoMap::Edge *edge) = 0;
	
	virtual void appendTransitionToPlan(nav_msgs::Path *pathPtr, std::string transitionType, boost::any const &pathData) {};

protected:
	MapEditor* mapEditor = nullptr;
};

typedef EdgeVisualizationInterface::Ptr EdgeVisualizationPluginPtr;

} // namespace topo_nav
