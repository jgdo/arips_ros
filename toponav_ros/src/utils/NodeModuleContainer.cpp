#include <toponav_ros/utils/NodeModuleContainer.h>

using namespace toponav_core;

tf2::Stamped <tf2::Transform> toponav_ros::NodeModuleContainer::getMapPoseFromPosition(const GlobalPosition &pos) {
  return poseContainer.getModule(pos.node->getRegionType())->getMapPoseFromPosition(pos);
}

std::pair <toponav_core::GlobalPosition, tf2::Stamped<tf2::Transform>>
toponav_ros::NodeModuleContainer::findGlobalPose(const tf2::Stamped <tf2::Transform> &pose, const TopoMap &map) {
  for(auto& plugin: poseContainer) {
    auto ret = plugin.second->findGlobalPose(pose, map);
    if(ret.first.node)
      return ret;
  }
  
  return std::pair<GlobalPosition, tf2::Stamped<tf2::Transform>>();
}

void toponav_ros::NodeModuleContainer::vizualizeNode(const toponav_core::TopoMap::Node *node) {
  if(auto module = vizContainer.getModuleOrNull(node->getRegionType()))
    module->vizualizeNode(node);
}

void toponav_ros::NodeModuleContainer::appendRegionEdgeToPlan(nav_msgs::Path *pathPtr, std::string regionType,
                                                           boost::any const &pathData) {
  if(auto module = vizContainer.getModuleOrNull(regionType))
    module->appendRegionEdgeToPlan(pathPtr, regionType, pathData);
}

void toponav_ros::NodeModuleContainer::parseNodeData(YAML::Node const &config, toponav_core::TopoMap::Node *node) {
  if(auto module = storageContainer.getModuleOrNull(node->getRegionType()))
    module->parseNodeData(config, node);
  else {
      ROS_ERROR_STREAM("No node module found for node type " << node->getRegionType());
  }
}

YAML::Node toponav_ros::NodeModuleContainer::saveNodeData(TopoMap::Node const *node) {
  if(auto module = storageContainer.getModuleOrNull(node->getRegionType()))
    return module->saveNodeData(node);
  else
    return YAML::Node();
}

void toponav_ros::NodeModuleContainer::initializeVisualization(toponav_ros::MapEditor *editor) {
  for(auto& plugin: vizContainer)
    plugin.second->initializeVisualization(editor);
}

void toponav_ros::NodeModuleContainer::beginParsing(toponav_core::TopoMap *map, YAML::Node const &parserConfig) {
  for(auto& plugin: storageContainer) {
    plugin.second->beginParsing(map, parserConfig["module_" + plugin.first]);
  }
}

void toponav_ros::NodeModuleContainer::endParsing() {
  for(auto& plugin: storageContainer)
    plugin.second->endParsing();
}

YAML::Node toponav_ros::NodeModuleContainer::beginSaving(TopoMap *map) {
  YAML::Node node;
  
  for(auto& plugin: storageContainer) {
    node["module_" + plugin.first] = plugin.second->beginSaving(map);
  }
  
 return node;
}

void toponav_ros::NodeModuleContainer::endSaving() {
  for(auto& plugin: storageContainer)
    plugin.second->endSaving();
}

std::pair<double, toponav_core::LocalPositionConstPtr>
toponav_ros::NodeModuleContainer::computeCostsOnRegion(const toponav_core::TopoMap::Node *node,
                                                    const toponav_core::LocalPosition &start,
                                                    const toponav_core::AbstractApproachExitData &end,
                                                       toponav_core::AbstractPathData *pathData) {
  return planningContainer.getModule(node->getRegionType())->computeCostsOnRegion(node, start, end, pathData);
}

void toponav_ros::NodeModuleContainer::convertGlobalPoseToApproachExitData(const GlobalPosition &pos,
                                                                           toponav_core::AbstractApproachExitData *approachData) {
  return planningContainer.getModule(pos.node->getRegionType())->convertGlobalPoseToApproachExitData(pos, approachData);
}

double toponav_ros::NodeModuleContainer::getHeuristics(const toponav_core::TopoMap::Node *region,
                                                    const toponav_core::AbstractApproachExitData &startData,
                                                    const toponav_core::AbstractApproachExitData &endData) {
  return planningContainer.getModule(region->getRegionType())->getHeuristics(region, startData, endData);
}

std::string toponav_ros::NodeModuleContainer::getModuleType(const toponav_core::TopoMap::Node *node) {
  return planningContainer.getModule(node->getRegionType())->getModuleType(node);
}
