#pragma once

#include "ApproachLineArea.h"
#include "FixedPosition.h"

#include <set>
#include <visualization_msgs/InteractiveMarker.h>
#include <toponav_ros/MapEditor.h>

namespace toponav_ros {

class ApproachExitVisualizer: public PlanningContextHolder, public EdgeVisualizationInterface {
public:
  void initEdgeDotColor(toponav_core::TopoMap::Edge* edge);
  
  std::set<std::string> createEdgeMarkers(const toponav_core::TopoMap::Edge *edge,
                                            ApproachExit3DPtr const &approach,
                                            ApproachExit3DPtr const &exit,
                                            uint8_t control_interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE);
  
  using PlanningContextHolder::PlanningContextHolder;
  
  /**
   * Is called whenever an approach pose changes (i.e. due to editing in rviz)
   * @param edge
   */
  virtual void onEdgeApproachChanged(const toponav_core::TopoMap::Edge* edge) {}

protected:
  tf2::Vector3 markerColor = tf2::Vector3(1, 0, 0);
  
  inline void insert(std::set<std::string>& names, const visualization_msgs::InteractiveMarker &int_marker) {
    names.insert(int_marker.name);
    mapEditor->getMarkerServer().insert(int_marker);
  }
  
  void inline insert(std::set<std::string>& names, const visualization_msgs::InteractiveMarker &int_marker,
               interactive_markers::InteractiveMarkerServer::FeedbackCallback feedback_cb) {
    names.insert(int_marker.name);
    mapEditor->getMarkerServer().insert(int_marker, feedback_cb);
  }
  
  /**
   *
   * @param feedback
   * @param approach
   * @param spot 0: start, 1: end, 2: center
   */
  void processApproachLineCB(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, const toponav_core::TopoMap::Edge *edge,
    ApproachLineAreaPtr approach, int spot);
  
  /**
  *
  * @param feedback
  * @param approach
  * @param spot 0: center, 1: direction
  */
  void processFixedPositionCB(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, toponav_core::TopoMap::Edge const *edge,
    toponav_ros::FixedPositionPtr approach, int spot);
};

} // namespace topo_nav
