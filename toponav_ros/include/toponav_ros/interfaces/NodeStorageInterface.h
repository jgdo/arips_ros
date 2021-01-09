#pragma once

#include <ros/ros.h>

#include <memory>

#include <toponav_core/TopoMap.h>
#include <yaml-cpp/yaml.h>

namespace toponav_ros {

class NodeStorageInterface {
public:
	typedef NodeStorageInterface BaseClass;
	typedef std::shared_ptr<NodeStorageInterface> Ptr;

	virtual void beginParsing(toponav_core::TopoMap *map, YAML::Node const &parserConfig) {}
	virtual void endParsing() {}

	virtual void parseNodeData(YAML::Node const& config, toponav_core::TopoMap::Node* node) = 0;
	
	virtual YAML::Node beginSaving(toponav_core::TopoMap *map) { return YAML::Node(); }
	virtual void endSaving() {};
	
	virtual YAML::Node saveNodeData(toponav_core::TopoMap::Node const *node) = 0;
};

typedef NodeStorageInterface::Ptr NodeStorageInterfacePtr;

} // namespace topo_nav
