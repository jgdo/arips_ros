#pragma once

#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <toponav_ros/interfaces/NodeVisualizationInterface.h>

#include "FlatGroundModule.h"
#include <arips_navigation/utils/ApproachLineArea.h>
#include <toponav_ros/PlanningContext.h>
#include <toponav_ros/interfaces/MapEditorModule.h>

namespace toponav_ros {

class FlatNodeVisualizer : public NodeVisualizationInterface, public MapEditorModule {
  public:
    FlatNodeVisualizer(PlanningContext &context, std::string const &nodeType);
    void activate() override;
    void deactivate() override;
    void poseCallback(const geometry_msgs::PoseStamped &pose) override;

    virtual void initializeVisualization(MapEditor *editor);

    virtual void vizualizeNode(const toponav_core::TopoMap::Node *node) override;

    virtual void appendRegionEdgeToPlan(nav_msgs::Path *pathPtr, std::string regionType,
                                        boost::any const &pathData) override;

  protected:
    void segmentLayerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void addNodeFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  private:
    PlanningContext &_context;
    std::string own_node_type_;

    ros::Publisher m_nodeMarkerPub;
};

} // namespace toponav_ros
