#pragma once
#include <toponav_ros/utils/NamedModuleContainer.h>

#include <toponav_core/interfaces/NodePlanningInterface.h>
#include <toponav_ros/interfaces/MapPoseInterface.h>
#include <toponav_ros/interfaces/NodeVisualizationInterface.h>
#include <toponav_ros/interfaces/NodeStorageInterface.h>

namespace toponav_ros {

class NodeModuleContainer: public toponav_core::NodePlanningInterface, public MapPoseInterface, public NodeVisualizationInterface, public NodeStorageInterface {
public:
  typedef std::shared_ptr<NodeModuleContainer> Ptr;
  
  inline void addPlanningModule(std::string type, toponav_core::NodePlanningInterfacePtr const &plugin) {
    planningContainer.addModule(type, plugin);
  }
  
  inline void addMapPoseModule(std::string type, MapPosePluginPtr const &plugin) {
    poseContainer.addModule(type, plugin);
  }
  
  inline void addVisualizationModule(std::string type, NodeVisualizationInterfacePtr const &plugin) {
    vizContainer.addModule(type, plugin);
  }
  
  inline void addStorageModule(std::string type, NodeStorageInterfacePtr const &plugin) {
    storageContainer.addModule(type, plugin);
  }
  
  std::pair<double, toponav_core::LocalPositionConstPtr>
  computeCostsOnRegion(const toponav_core::TopoMap::Node *node, toponav_core::LocalPosition const &start, toponav_core::AbstractApproachExitData const &end,
                       toponav_core::AbstractPathData *pathData) override;
  
  void convertGlobalPoseToApproachExitData(const toponav_core::GlobalPosition &pos, toponav_core::AbstractApproachExitData *approachData) override;
  
  tf2::Stamped<tf2::Transform> getMapPoseFromPosition(toponav_core::GlobalPosition const &pos) override;
  
  std::pair<toponav_core::GlobalPosition, tf2::Stamped<tf2::Transform>>
  findGlobalPose(tf2::Stamped<tf2::Transform> const &pose, const toponav_core::TopoMap &map) override;
  
  void vizualizeNode(const toponav_core::TopoMap::Node *node) override;
  
  void appendRegionEdgeToPlan(nav_msgs::Path *pathPtr, std::string regionType, boost::any const &pathData) override;
  
  void initializeVisualization(MapEditor *editor) override;
  
  double getHeuristics(toponav_core::TopoMap::Node const *region, toponav_core::AbstractApproachExitData const &startData,
                       toponav_core::AbstractApproachExitData const &endData) override;
  
  std::string getModuleType(const toponav_core::TopoMap::Node *node) override;

private:
  void beginParsing(toponav_core::TopoMap *map, YAML::Node const &parserConfig) override;
  
  void endParsing() override;
  
  YAML::Node beginSaving(toponav_core::TopoMap *map) override;
  
  void endSaving() override;

private:
  void parseNodeData(YAML::Node const &config, toponav_core::TopoMap::Node *node) override;
  
  YAML::Node saveNodeData(toponav_core::TopoMap::Node const *node) override;

protected:
  NamedModuleContainer<toponav_core::NodePlanningInterface> planningContainer;
  NamedModuleContainer<MapPoseInterface> poseContainer;
  NamedModuleContainer<NodeVisualizationInterface> vizContainer;
  NamedModuleContainer<NodeStorageInterface> storageContainer;
};
  
typedef NodeModuleContainer::Ptr NodeModuleContainerPtr;

} // namespace
