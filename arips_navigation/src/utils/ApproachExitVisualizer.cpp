#include <arips_navigation/utils/ApproachExitVisualizer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


using namespace toponav_core;

static visualization_msgs::InteractiveMarker createApproachTipMarker(tf2::Vector3 pos, const std::string& frame, std::string name, tf2::Vector3 const& color) {
  geometry_msgs::PoseStamped poseStart;
  tf2::toMsg(pos, poseStart.pose.position);
  poseStart.header.stamp = ros::Time::now();
  poseStart.header.frame_id = frame;
  
  visualization_msgs::InteractiveMarker int_marker;
  
  int_marker.header = poseStart.header;
  int_marker.name = name;
  int_marker.scale = 1;
  int_marker.pose = poseStart.pose;
  
  visualization_msgs::InteractiveMarkerControl control;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
  // controlStart.orientation_mode = visualization_msgs::InteractiveMarkerControl::VIEW_FACING;
  control.always_visible = true;
  tf2::Quaternion rotY(tf2::Vector3(0,1,0), M_PI_2);
  tf2::convert(rotY, control.orientation);
  
  visualization_msgs::Marker marker;
  marker.header = poseStart.header;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = poseStart.pose;
  marker.color.r = color.x();
  marker.color.g = color.y();
  marker.color.b = color.z();
  marker.color.a = 1;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  
  control.markers.push_back(marker);
  int_marker.controls.push_back(control);
  
  return int_marker;
}

static visualization_msgs::InteractiveMarker createApproachCenterMarker(tf2::Stamped<tf2::Transform> poseTf, std::string name, tf2::Vector3 const& color) {
  poseTf.stamp_ = ros::Time::now();
  geometry_msgs::PoseStamped poseStart;
  tf2::toMsg(poseTf, poseStart);
  
  visualization_msgs::InteractiveMarker int_marker;
  
  int_marker.header = poseStart.header;
  int_marker.name = name;
  int_marker.scale = 1;
  int_marker.pose = poseStart.pose;
  
  visualization_msgs::InteractiveMarkerControl control;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  // controlStart.orientation_mode = visualization_msgs::InteractiveMarkerControl::VIEW_FACING;
  control.always_visible = true;
  tf2::Quaternion rotY(tf2::Vector3(0,1,0), M_PI_2);
  tf2::convert(rotY, control.orientation);
  
  visualization_msgs::Marker marker;
  marker.header = poseStart.header;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = poseStart.pose;
  marker.color.r = color.x();
  marker.color.g = color.y();
  marker.color.b = color.z();
  marker.color.a = 1;
  marker.scale.x = 0.3;
  marker.scale.y = 0.06;
  marker.scale.z = 0.06;
  
  control.markers.push_back(marker);
  int_marker.controls.push_back(control);
  
  return int_marker;
}

static tf2::Stamped<tf2::Transform> transformAt(tf2_ros::BufferInterface* buffer,
        const tf2::Stamped<tf2::Transform>& in,
        const std::string& target_frame, const ros::Time& target_time,
                                                ros::Duration timeout=ros::Duration(0.0)) {
    tf2::Stamped<tf2::Transform> out;
    tf2::doTransform(in, out, buffer->lookupTransform(target_frame, tf2::getFrameId(in), target_time, timeout));
    return out;
}

std::set<std::string>
toponav_ros::ApproachExitVisualizer::createEdgeMarkers(const toponav_core::TopoMap::Edge *edge,
                                                    ApproachExit3DPtr const &approach,
                                                    ApproachExit3DPtr const &exit,
                                                    uint8_t control_interaction_mode) {
  std::set<std::string> names;
  
  tf2::Stamped<tf2::Transform> centerApproach, centerExit;
  if(ApproachLineAreaPtr approachLine = std::dynamic_pointer_cast<ApproachLineArea>(approach)) {
    insert(names, createApproachTipMarker(approachLine->start, approachLine->orientation.frame_id_, edge->getName() + "_approach_start", markerColor),
                                        boost::bind(&ApproachExitVisualizer::processApproachLineCB, this, _1, edge, approachLine,
                                                    0));
    insert(names, createApproachTipMarker(approachLine->end, approachLine->orientation.frame_id_, edge->getName() + "_approach_end", markerColor),
                                        boost::bind(&ApproachExitVisualizer::processApproachLineCB, this, _1, edge, approachLine,
                                                    1));
    
    centerApproach.frame_id_ = approachLine->orientation.frame_id_;
    centerApproach.stamp_ = ros::Time::now();
    centerApproach.setOrigin((approachLine->start + approachLine->end) * 0.5);
    centerApproach.setRotation(approachLine->orientation);
    
    insert(names, createApproachCenterMarker(centerApproach, edge->getName()+"_approach_center", markerColor),
                                        boost::bind(&ApproachExitVisualizer::processApproachLineCB, this, _1, edge, approachLine,
                                                    2));
  } else if(FixedPositionPtr approachPos = std::dynamic_pointer_cast<FixedPosition>(approach)) {
    insert(names, createApproachTipMarker(approachPos->pose.getOrigin(), approachPos->pose.frame_id_, edge->getName() + "_approach_center", markerColor),
           boost::bind(&ApproachExitVisualizer::processFixedPositionCB, this, _1, edge, approachPos,
                       0));
  
    centerApproach = approachPos->pose;
  
    insert(names, createApproachCenterMarker(centerApproach, edge->getName()+"_approach_direction", markerColor),
           boost::bind(&ApproachExitVisualizer::processFixedPositionCB, this, _1, edge, approachPos,
                       1));
  } // FIXME else
  
  if(ApproachLineAreaPtr approachLine = std::dynamic_pointer_cast<ApproachLineArea>(exit)) {
    centerExit.frame_id_ = approachLine->orientation.frame_id_;
    centerExit.stamp_ = ros::Time::now();
    centerExit.setOrigin((approachLine->start + approachLine->end) * 0.5);
    centerExit.setRotation(approachLine->orientation);
  } else if(FixedPositionPtr approachPos = std::dynamic_pointer_cast<FixedPosition>(exit)) {
    centerExit = approachPos->pose;
  } // FIXME ELSE
  
  if(!centerApproach.frame_id_.empty() && !centerExit.frame_id_.empty()) {
    visualization_msgs::InteractiveMarker edgeIntMarker;
    edgeIntMarker.header.frame_id = _context.globalFrame;
    edgeIntMarker.header.stamp = ros::Time::now();
    edgeIntMarker.name = "step_edge_" + edge->getName();
    edgeIntMarker.description = edge->getName();
    edgeIntMarker.scale = 1;
  
    visualization_msgs::InteractiveMarkerControl control;
    control.name = edge->getName();
    control.interaction_mode = control_interaction_mode;
    control.always_visible = true;
    edgeIntMarker.controls.push_back(control);

    try {
        tf2::Stamped<tf2::Transform> transformedApproach = transformAt(_context.tfBuffer, centerApproach,
                                                                                        _context.globalFrame,
                                                                       centerApproach.stamp_ /* ros::Time::now()*/);
        tf2::Stamped<tf2::Transform> transformedExit = transformAt(_context.tfBuffer,
                centerExit, _context.globalFrame, centerExit.stamp_  /* ros::Time::now()*/);


        visualization_msgs::Marker edgeMarker1;
        edgeMarker1.header.frame_id = _context.globalFrame;
        edgeMarker1.header.stamp = ros::Time();
        edgeMarker1.ns = "StepEdge_" + edge->getName();
        edgeMarker1.id = 0;
        edgeMarker1.text = edge->getName();
        edgeMarker1.type = visualization_msgs::Marker::ARROW;
        edgeMarker1.action = visualization_msgs::Marker::ADD;
        edgeMarker1.pose.position.x = 0;
        edgeMarker1.pose.position.y = 0;
        edgeMarker1.pose.position.z = 0;
        edgeMarker1.pose.orientation.x = 0.0;
        edgeMarker1.pose.orientation.y = 0.0;
        edgeMarker1.pose.orientation.z = 0.0;
        edgeMarker1.pose.orientation.w = 1.0;
        edgeMarker1.scale.x = 0.05;
        edgeMarker1.scale.y = 0.1;
        edgeMarker1.scale.z = 0.3;

        edgeMarker1.color.a = 1.0; // Don't forget to set the alpha!
        edgeMarker1.color.r = 0;
        edgeMarker1.color.g = 1;
        edgeMarker1.color.b = 0;

        geometry_msgs::Point approachMsg, exitMsg;
        tf2::toMsg(transformedApproach.getOrigin(), approachMsg);
        tf2::toMsg(transformedExit.getOrigin(), exitMsg);
        edgeMarker1.points.push_back(approachMsg);
        edgeMarker1.points.push_back(exitMsg);

        edgeIntMarker.controls.back().markers.push_back(edgeMarker1);

        insert(names, edgeIntMarker);

    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
    }
  }
  
  return names;
}

void toponav_ros::ApproachExitVisualizer::processApproachLineCB(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, const TopoMap::Edge *edge,
    ApproachLineAreaPtr approach, int spot) {
  if(feedback->event_type != visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
    return;
  
  const auto transform = _context.tfBuffer->lookupTransform(approach->orientation.frame_id_, feedback->header.frame_id, ros::Time(0));
    geometry_msgs::Pose poseTransformedMsg;
  tf2::doTransform(feedback->pose, poseTransformedMsg, transform);

    tf2::Transform poseTransformed;
    tf2::fromMsg(poseTransformedMsg, poseTransformed);

  
  if(spot == 0) { // start
    approach->start = poseTransformed.getOrigin();
  } else if(spot == 1) { // end
    approach->end = poseTransformed.getOrigin();
  } else if(spot == 2) { // center
    approach->orientation.setData(poseTransformed.getRotation());
  } else return;

  onEdgeApproachChanged(edge);
  _context.mapChanged();
}

void toponav_ros::ApproachExitVisualizer::processFixedPositionCB(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, TopoMap::Edge const *edge,
    toponav_ros::FixedPositionPtr approach, int spot) {
  if(feedback->event_type != visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
    return;

  const auto transform = _context.tfBuffer->lookupTransform(approach->pose.frame_id_, feedback->header.frame_id, ros::Time(0));

    geometry_msgs::Pose poseMsg;
    tf2::doTransform(feedback->pose, poseMsg, transform);
      tf2::Transform pose;
      tf2::convert(poseMsg, pose);

  
  if(spot == 0) { // center
    approach->pose.setOrigin(pose.getOrigin());
  } else if(spot == 1) { // direction
    approach->pose.setRotation(pose.getRotation());
  } else return;
  
  onEdgeApproachChanged(edge);
  _context.mapChanged();
}

void toponav_ros::ApproachExitVisualizer::initEdgeDotColor(TopoMap::Edge* edge) {
  std::stringstream ss;
  ss << "#" << std::setfill('0') << std::setw(1) << std::hex << markerColor.x() << markerColor.y() << markerColor.z();
  edge->propertyMap()[TopoMap::ENTRY_DOT_COLOR] = ss.str();
}
