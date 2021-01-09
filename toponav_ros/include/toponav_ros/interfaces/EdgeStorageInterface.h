#pragma once

#include <ros/ros.h>

#include <memory>

#include <toponav_core/TopoMap.h>

#include <yaml-cpp/yaml.h>

namespace toponav_ros {

class EdgeStorageInterface {
public:
	typedef EdgeStorageInterface BaseClass;
	typedef std::shared_ptr<EdgeStorageInterface> Ptr;

	virtual void beginParsing(toponav_core::TopoMap *map, YAML::Node const &parserConfig) {}
	virtual void endParsing() {}

	virtual void parseEdgeData(YAML::Node const& config, toponav_core::TopoMap::Edge* edge) = 0;
	virtual void parseEdgeOutData(YAML::Node const& config, toponav_core::TopoMap::Edge* edge) = 0;
	virtual void parseEdgeInData(YAML::Node const &config, toponav_core::TopoMap::Edge *edge) = 0;
	
	virtual bool isEdgeSaveable(const toponav_core::TopoMap::Edge *edge) const { return true; }
	
	virtual YAML::Node beginSaving(toponav_core::TopoMap *map) { return YAML::Node(); }
	virtual void endSaving() {};
	
	virtual YAML::Node saveEdgeData(toponav_core::TopoMap::Edge const *edge) = 0;
  virtual YAML::Node saveEdgeOutData(toponav_core::TopoMap::Edge const *edge) = 0;
  virtual YAML::Node saveEdgeInData(toponav_core::TopoMap::Edge const *edge) = 0;
};

typedef EdgeStorageInterface::Ptr EdgeStorageInterfacePtr;
	
} // namespace topo_nav

