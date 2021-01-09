#pragma once

#include <toponav_ros/interfaces/NodeStorageInterface.h>
#include "FlatGroundModule.h"

#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <navfn/navfn_ros.h>
#include <toponav_ros/PlanningContext.h>

namespace toponav_ros {

class FlatParser: public NodeStorageInterface, public PlanningContextHolder {
public:
	typedef std::shared_ptr<FlatParser> Ptr;

	using PlanningContextHolder::PlanningContextHolder;

	virtual void beginParsing(toponav_core::TopoMap *map, YAML::Node const &parserConfig) override;

	virtual void endParsing() override ;

	virtual void parseNodeData(YAML::Node const &config, toponav_core::TopoMap::Node *node) override;
  
  virtual YAML::Node beginSaving(toponav_core::TopoMap *map) override;
  
	virtual YAML::Node saveNodeData(toponav_core::TopoMap::Node const *node) override;

private:
  FlatGroundModule::MapData costmapPlannerMap;
	
	toponav_core::TopoMap* currentMap = nullptr;
};

typedef FlatParser::Ptr FlatParserPtr;

} // namespace topo_nav
