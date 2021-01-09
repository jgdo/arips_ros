#include <arips_navigation/FlatNodeVisualizer.h>
#include <arips_navigation/FlatGroundModule.h>

#include <geometry_msgs/PoseStamped.h>
#include <arips_navigation/utils/ApproachLineArea.h>

#include <toponav_ros/MapEditor.h>

namespace toponav_ros {

using namespace toponav_core;

void FlatNodeVisualizer::initializeVisualization(MapEditor *editor) {
	NodeVisualizationInterface::initializeVisualization(editor);
  
  auto segmentHandle = mapEditor->getMenuHandler().insert(mapEditor->getOtherMenuHandle(), "Auto-segment flat layer");
  for(auto& entry: FlatGroundModule::getMapData(mapEditor->getMap())) {
    mapEditor->getMenuHandler().insert(segmentHandle, entry.first, boost::bind(&FlatNodeVisualizer::segmentLayerFeedback, this, _1, entry.first));
  }
}

void FlatNodeVisualizer::appendRegionEdgeToPlan(nav_msgs::Path *pathPtr, std::string regionType,
												boost::any const &pathData) {
	const std::vector<geometry_msgs::PoseStamped>* plan = boost::any_cast<std::vector<geometry_msgs::PoseStamped>>(&pathData);
	pathPtr->poses.insert(pathPtr->poses.end(), plan->begin(), plan->end());

//	if(plan && plan->size() > 0) {
//		visualization_msgs::Marker marker;
//
//		marker.type = visualization_msgs::Marker::LINE_STRIP;
//		marker.header.frame_id = plan->at(0).header.frame_id;
//		marker.header.stamp = ros::Time::now();
//		marker.action = visualization_msgs::Marker::ADD;
//		marker.points.reserve(plan->size());
//		marker.scale.x = 0.1;
//		marker.ns = "FlatNodeVisualizer";
//		marker.id = pathPtr->markers.size();
//		marker.color.a = 1;
//		marker.color.r = 1;
//		marker.color.g = 0;
//		marker.color.b = 0;
//
//		for(auto& poseConst: *plan) {
//			geometry_msgs::PoseStamped pose = poseConst;
//			pose.header.stamp = ros::Time::now(); // TODO better solution?
//
//			geometry_msgs::PoseStamped poseOut;
//			tf.transformPose(marker.header.frame_id, pose, poseOut);
//			marker.points.push_back(pose.pose.position);
//		}
//
//		pathPtr->markers.push_back(marker);
//	}
}

void FlatNodeVisualizer::vizualizeNode(const TopoMap::Node *node) {
	// nothing to do here for now
}

void FlatNodeVisualizer::segmentLayerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback,
                                              std::string mapName) {
  FlatGroundModule::segmentAllNodes(mapEditor->getMap(), mapName, own_node_type_,50); // FIXME hardcoded number
}

  
} // namespace topo_nav
