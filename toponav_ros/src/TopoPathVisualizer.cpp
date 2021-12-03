#include "toponav_ros/TopoPathVisualizer.h"
#include <geometry_msgs/PoseArray.h>

namespace toponav_ros {

using namespace toponav_core;

TopoPathVisualizer::TopoPathVisualizer(ModuleContainer &factory, RosContext& context):
	nodeViz(factory.getCreateModule<NodeVisualizationInterface>()),
	edgeViz(factory.getCreateModule<EdgeVisualizationInterface>()),
	_context(context) {
	ros::NodeHandle nh;
	// _planPub = nh.advertise<visualization_msgs::MarkerArray>("topo_plan", 3, true);
	_planPub = nh.advertise<nav_msgs::Path>("/topo_planner/original_path", 3, true);
	_poseArrayPub = nh.advertise<geometry_msgs::PoseArray>("/topo_planner/poses_plan", 3, true);
}

void TopoPathVisualizer::visualizePath(TopoPath const &path) {
	fillNavPath(path, &pathMsg);
	_planPub.publish(pathMsg);
  
  geometry_msgs::PoseArray poses;
  poses.header = pathMsg.header;
  poses.poses.reserve(pathMsg.poses.size());
  for(auto& p: pathMsg.poses)
    poses.poses.push_back(p.pose);
  _poseArrayPub.publish(poses);
}

void TopoPathVisualizer::fillNavPath(TopoPath const &path, nav_msgs::Path *navPath) {
	//	if(pathMarkers.markers.size() > 0) { // used before -> send delete command
//		for(auto& marker: pathMarkers.markers)
//			marker.action = visualization_msgs::Marker::DELETE;
//		_planPub.publish(pathMarkers);
//		
//		pathMarkers.markers.clear();
//	}
  navPath->poses.clear();
	navPath->header.frame_id = _context.globalFrame;
	navPath->header.stamp = ros::Time::now();
	
	LambdaPlanVisitor visitor(
			[&, this](TopoPath::RegionMovement const* mov) {
				if(mov->pathData.empty()) { // no path data given, just draw a straight line from node a->b
					// FIXME
					//			visualization_msgs::Marker marker;
					//			
					//			geometry_msgs::PoseStamped poseA = nodePlanner->getMapPoseFromPosition(GlobalPosition(n->predEdge->a->region, n->predEdge->a->position));
					//			geometry_msgs::PoseStamped poseB = nodePlanner->getMapPoseFromPosition(GlobalPosition(n->predEdge->b->region, n->predEdge->b->position));
					//			// TODO make sure both poses are in global frame
					//			
					//			marker.type = visualization_msgs::Marker::LINE_STRIP;
					//			marker.header.frame_id = poseA.header.frame_id;
					//			marker.header.stamp = ros::Time::now() - ros::Duration(0.01); // FIXME
					//			marker.action = visualization_msgs::Marker::ADD;
					//			marker.points.reserve(2);
					//			marker.scale.x = 0.1;
					//			marker.ns = "DijkstraTopoPlanner";
					//			marker.id = pathMarkers.markers.size();
					//			marker.color.a = 1;
					//			marker.color.r = 1;
					//			marker.color.g = 0;
					//			marker.color.b = 1;
					//			
					//			
					//			marker.points.push_back(poseA.pose.position);
					//			marker.points.push_back(poseB.pose.position);
					//			
					//			pathMarkers.markers.push_back(marker);
				} else {
					nodeViz->appendRegionEdgeToPlan(navPath, mov->start.node->getRegionType(), mov->pathData);
				}
			},
			[&, this](TopoPath::Transition const* trans) {
				if(trans->pathData.empty()) { // no path data given, just draw a straight line from node a->b
					// FIXME
				} else {
					edgeViz->appendTransitionToPlan(navPath, trans->topoEdge->getTransitionType(), trans->pathData);
				}
			}
	);
	
	path.visitPlan(visitor);
}
	
	
} // namespace topo_nav

