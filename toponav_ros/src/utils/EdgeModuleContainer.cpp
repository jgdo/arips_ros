#include <toponav_ros/utils/EdgeModuleContainer.h>

namespace toponav_ros {

using namespace toponav_core;

void EdgeModuleContainer::beginParsing(TopoMap *map, YAML::Node const &parserConfig) {
	for(auto& plugin: storageContainer) {
		plugin.second->beginParsing(map, parserConfig["module_" + plugin.first]);
	}
}

void EdgeModuleContainer::endParsing() {
	for(auto& plugin: storageContainer) {
		plugin.second->endParsing();
	}
}

void EdgeModuleContainer::parseEdgeData(YAML::Node const &config, TopoMap::Edge *edge) {
	if(auto module = storageContainer.getModuleOrNull(edge->getTransitionType()))
		module->parseEdgeData(config, edge);
	else {
	    ROS_ERROR_STREAM("No edge module found for edge type " << edge->getTransitionType());
	}
}

YAML::Node EdgeModuleContainer::beginSaving(TopoMap *map) {
	YAML::Node node;

	for(auto& plugin: storageContainer) {
		node["module_" + plugin.first] = plugin.second->beginSaving(map);
	}
	
	return node;
}

void EdgeModuleContainer::endSaving() {
	for(auto& plugin: storageContainer) {
		plugin.second->endSaving();
	}
}

YAML::Node EdgeModuleContainer::saveEdgeData(TopoMap::Edge const *edge) {
	if(auto module = storageContainer.getModuleOrNull(edge->getTransitionType()))
		return module->saveEdgeData(edge);
	else
		return YAML::Node();
}

void EdgeModuleContainer::initializeVisualization(MapEditor *editor) {
	for(auto& plugin: vizContainer) {
		plugin.second->initializeVisualization(editor);
	}
}

void EdgeModuleContainer::beginMapVisualization() {
	for(auto& plugin: vizContainer) {
		plugin.second->beginMapVisualization();
	}
}

void EdgeModuleContainer::endMapVisualization() {
	for(auto& plugin: vizContainer) {
		plugin.second->endMapVisualization();
	}
}

void EdgeModuleContainer::visualizeEdge(const TopoMap::Edge *edge) {
	if(auto module = vizContainer.getModuleOrNull(edge->getTransitionType()))
    module->visualizeEdge(edge);
}

void EdgeModuleContainer::appendTransitionToPlan(nav_msgs::Path *pathPtr,
												 std::string transitionType, boost::any const &pathData) {
  if(auto module = vizContainer.getModuleOrNull(transitionType))
    module->appendTransitionToPlan(pathPtr, transitionType, pathData);
}

bool EdgeModuleContainer::areEdgesCoupled(const TopoMap::Edge *edgeA, const TopoMap::Edge *edgeB) {
	return edgeA == edgeB || planningContainer.getModule(edgeA->getTransitionType())->areEdgesCoupled(edgeA, edgeB);
}

void EdgeModuleContainer::initApproachData(const TopoMap::Edge *edge, AbstractApproachExitData *approachData) {
	planningContainer.getModule(edge->getTransitionType())->initApproachData(edge, approachData);
}

void EdgeModuleContainer::initExitData(const TopoMap::Edge *edge, AbstractApproachExitData *exitData) {
	planningContainer.getModule(edge->getTransitionType())->initExitData(edge, exitData);
}

std::pair<double, LocalPositionConstPtr>
EdgeModuleContainer::computeTransitionCost(const TopoMap::Edge *edge, const LocalPosition &approachPose,
																					 boost::any *pathData) {
	return planningContainer.getModule(edge->getTransitionType())->computeTransitionCost(edge, approachPose, pathData);
}

void EdgeModuleContainer::parseEdgeOutData(YAML::Node const &config, TopoMap::Edge *edge) {
	if(auto module = storageContainer.getModuleOrNull(edge->getTransitionType()))
		module->parseEdgeOutData(config, edge);
}

void EdgeModuleContainer::parseEdgeInData(YAML::Node const &config, TopoMap::Edge *edge) {
	if(auto module = storageContainer.getModuleOrNull(edge->getTransitionType()))
		module->parseEdgeInData(config, edge);
}

YAML::Node EdgeModuleContainer::saveEdgeOutData(TopoMap::Edge const *edge) {
	if(auto module = storageContainer.getModuleOrNull(edge->getTransitionType()))
		return module->saveEdgeOutData(edge);
	else
		return YAML::Node();
}

YAML::Node EdgeModuleContainer::saveEdgeInData(TopoMap::Edge const *edge) {
  if(auto module = storageContainer.getModuleOrNull(edge->getTransitionType()))
    return module->saveEdgeInData(edge);
  else
    return YAML::Node();
}

double EdgeModuleContainer::getHeuristics(TopoMap::Edge const *edge) {
	return planningContainer.getModule(edge->getTransitionType())->getHeuristics(edge);
}

std::string EdgeModuleContainer::getModuleType(const TopoMap::Edge *edge) {
	return planningContainer.getModule(edge->getTransitionType())->getModuleType(edge);
}

bool EdgeModuleContainer::isEdgeSaveable(const TopoMap::Edge *edge) const {
	if(auto module = storageContainer.getModuleOrNull(edge->getTransitionType()))
		return module->isEdgeSaveable(edge);
	else
		return false;
}
	
} // namespace topo_nav























