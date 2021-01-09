#include "toponav_ros/MapEditor.h"

#include <toponav_msgs/GetString.h>
#include <toponav_ros/TopoMapYamlStorage.h>

namespace toponav_ros {

using namespace visualization_msgs;
using namespace interactive_markers;

using namespace toponav_core;

static const std::string EDITOR_MARKER = "editor_main_marker";

MapEditor::MapEditor(ModuleContainer &factory, PlanningContext& context) :
		_context(context),
		nodeViz(factory.getCreateModule<NodeVisualizationInterface>()),
		edgeViz(factory.getCreateModule<EdgeVisualizationInterface>()),
		nodeStore(factory.getCreateModule<NodeStorageInterface>()),
		edgeStore(factory.getCreateModule<EdgeStorageInterface>()),
		server("map_editor") {
	
	ros::NodeHandle nh;
	_poseSub = nh.subscribe("editor_pose", 1, &MapEditor::poseCB, this);
	_stringClient = nh.serviceClient<toponav_msgs::GetString>("/get_string");
	
	_context.mapChanged.connect(boost::bind(&MapEditor::displayMap, this));
}

void MapEditor::startEditMap() {
  handleEdges = menu_handler.insert("Create Topo Edge...");
  handleOther = menu_handler.insert("Other ..");
  menu_handler.insert("Override Current Topo Map", boost::bind(&MapEditor::saveMapCB, this, _1));
  menu_handler.insert("Save Topo Map As ..", boost::bind(&MapEditor::saveMapAsCB, this, _1));
	menu_handler.insert("Export Topo Map to DOT file ..", boost::bind(&MapEditor::exportDotCB, this, _1));
  
  // initiaize viz so it can place own sub menus
  edgeViz->initializeVisualization(this);
  nodeViz->initializeVisualization(this);
  
	displayMap();
}

void MapEditor::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
	
}

void MapEditor::poseCB(const geometry_msgs::PoseStampedConstPtr &msg) {
	if(activeEditorModule) {
		activeEditorModule->poseCallback(*msg);
	}
}

void MapEditor::displayMap() {
  server.clear();
  
  visualization_msgs::InteractiveMarker int_marker;
  
  int_marker.header.frame_id = _context.globalFrame;
  int_marker.header.stamp = ros::Time::now();
  int_marker.name = EDITOR_MARKER;
  int_marker.description = "Map Editor Handle";
  int_marker.scale = 3;
  
  InteractiveMarkerControl control;
  control.interaction_mode = InteractiveMarkerControl::MOVE_3D;
  control.always_visible = true;
  
  Marker marker;
  
  marker.type = Marker::CUBE;
  marker.pose.position.z = 5;
  marker.scale.x = 2;
  marker.scale.y = 2;
  marker.scale.z = 2;
  marker.color.r = 1;
  marker.color.g = 0.3;
  marker.color.b = 0.8;
  marker.color.a = 1.0;
  
  control.markers.push_back(marker);
  int_marker.controls.push_back(control);
  
  server.insert(int_marker, boost::bind(&MapEditor::processFeedback, this, _1));
  
  menu_handler.apply(server, EDITOR_MARKER);
  
	edgeViz->beginMapVisualization();
	
	_context.topoMap->foreachNode([this](const TopoMap::Node* node) {
		nodeViz->vizualizeNode(node);
	});
	
	_context.topoMap->foreachEdge([this](const TopoMap::Edge* edge) {
		edgeViz->visualizeEdge(edge);
	});
	
	edgeViz->endMapVisualization();
	
	server.applyChanges();
}

void MapEditor::saveMapCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
	if(!_context.topoMap->getMetaData().fileName.empty()) {
		ROS_INFO_STREAM("Overriding map " << _context.topoMap->getMetaData().fileName);
		TopoMapYamlStorage::saveMap(_context.topoMap.get(), _context.topoMap->getMetaData().fileName, nodeStore, edgeStore);
	} else {
		saveMapAsCB(feedback);
	}
}

void MapEditor::saveMapAsCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
	std::string filename;
	
	toponav_msgs::GetString srv;
	srv.request.title = "Save Topo Map As:";
	srv.request.type = toponav_msgs::GetString::Request::GET_WRITE_FILE;
	if(_stringClient.call(srv)) {
		if(srv.response.success) {
			filename = srv.response.str;
		} else {
			ROS_INFO("Save Map cancelled by user.");
			return;
		}
	} else {
		ROS_WARN_STREAM("String service not available, using auto-generated map filename.");
		
		std::stringstream filenamess;
		filenamess << "topomap_" << ros::Time::now();
		filename = filenamess.str();
	}
  
  TopoMapYamlStorage::saveMap(_context.topoMap.get(), filename, nodeStore, edgeStore);
}

void MapEditor::activateEditorModule(MapEditorModule* plugin) {
	if(activeEditorModule && activeEditorModule != plugin) {
		activeEditorModule->deactivate();
	}
	
	activeEditorModule = plugin;
	activeEditorModule->activate();
}

void MapEditor::deactivateEditorModule(MapEditorModule* plugin) {
	if(plugin) {
		plugin->deactivate();
		
		if(plugin == activeEditorModule)
			activeEditorModule = nullptr;
	} else if(activeEditorModule) {
		activeEditorModule->deactivate();
		activeEditorModule = nullptr;
	}
}

std::string MapEditor::requestUserStringInput(std::string requestText) {
	std::string str;
	
	toponav_msgs::GetString srv;
	srv.request.title = requestText;
	srv.request.type = toponav_msgs::GetString::Request::GET_STR;
	if(_stringClient.call(srv)) {
		if(srv.response.success) {
			str = srv.response.str;
		} else {
			ROS_INFO("String request cancelled by user.");
		}
	} else {
		ROS_INFO_STREAM("String service not available");
	}
	
	return str;
}

void MapEditor::exportDotCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
	std::string filename;
	
	toponav_msgs::GetString srv;
	srv.request.title = "Export to Topo Map as DOT file:";
	srv.request.type = toponav_msgs::GetString::Request::GET_WRITE_FILE;
	if(_stringClient.call(srv)) {
		if(srv.response.success) {
			filename = srv.response.str;
		} else {
			ROS_INFO("DOT export cancelled by user.");
			return;
		}
	} else {
		ROS_WARN_STREAM("String service not available, using auto-generated DOT filename.");
		
		std::stringstream filenamess;
		filenamess << "topo_map_dot_" << ros::Time::now();
		filename = filenamess.str();
	}
	
	ROS_INFO_STREAM("Exporting Topo Map to DOT file '" << filename << "'.");
	_context.topoMap->exportToDot(filename);
}
	
} // namespace topo_nav
