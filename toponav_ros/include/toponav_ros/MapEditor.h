#pragma once

#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include "toponav_ros/interfaces/EdgeVisualizationInterface.h"
#include "toponav_ros/interfaces/NodeVisualizationInterface.h"
#include "toponav_ros/interfaces/MapEditorModule.h"
#include "toponav_ros/interfaces/EdgeStorageInterface.h"
#include "toponav_ros/interfaces/NodeStorageInterface.h"

#include <toponav_core/TopoMap.h>
#include <toponav_core/ModuleContainer.h>
#include <toponav_ros/interfaces/MapPoseInterface.h>

#include "PlanningContext.h"

namespace toponav_ros {

class MapEditor {
public:
	MapEditor(toponav_core::ModuleContainer &factory, PlanningContext& context);
	
	void startEditMap();
	
	void activateEditorModule(MapEditorModule* plugin);
	void deactivateEditorModule(MapEditorModule* plugin = nullptr);
	
	void displayMap();
	
	std::string requestUserStringInput(std::string requestText);
  
	inline interactive_markers::InteractiveMarkerServer& getMarkerServer() {
		return server;
	}
	
	inline toponav_core::TopoMap* getMap() {
		return _context.topoMap.get();
	}
	
	inline interactive_markers::MenuHandler& getMenuHandler() {
		return menu_handler;
	}
	
	inline interactive_markers::MenuHandler::EntryHandle getCreateEdgesMenuHandle() const {
		return handleEdges;
	}
	
	inline interactive_markers::MenuHandler::EntryHandle getOtherMenuHandle() const {
		return handleOther;
	}
	
protected:
	void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
	
	void saveMapCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
	
	void saveMapAsCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
	
	void exportDotCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
	
	void poseCB(const geometry_msgs::PoseStampedConstPtr& msg);
  
  PlanningContext& _context;
	
	NodeVisualizationInterface::Ptr nodeViz;
	EdgeVisualizationInterface::Ptr edgeViz;
	
	NodeStorageInterfacePtr nodeStore;
	EdgeStorageInterfacePtr edgeStore;
	
	MapEditorModule* activeEditorModule = nullptr;
	
	interactive_markers::InteractiveMarkerServer server;
	
	interactive_markers::MenuHandler menu_handler;
	
	interactive_markers::MenuHandler::EntryHandle handleEdges;
	interactive_markers::MenuHandler::EntryHandle handleOther;
	
	std::map<interactive_markers::MenuHandler::EntryHandle, std::string> createEdgeNames;
	
	ros::Subscriber _poseSub;
	
	ros::ServiceClient _stringClient;
	
	enum {
		IDLE, CREATING_EDGE_APPROACH, CREATING_EDGE_EXIT, CREATING_MAP_DOOR, WAITING_DOOR_SELECTION
	} state = IDLE;
};

} // namespace topo_nav

