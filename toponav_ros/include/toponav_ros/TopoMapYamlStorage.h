#pragma once

#include <tf/transform_datatypes.h>

#include <toponav_core/TopoMap.h>
#include <toponav_ros/interfaces/NodeStorageInterface.h>
#include <toponav_ros/interfaces/EdgeStorageInterface.h>

#include <toponav_core/ModuleContainer.h>

namespace toponav_ros {

class TopoMapYamlStorage {
public:
	static void parseMap(toponav_core::TopoMap& map, std::string filename, toponav_core::ModuleContainer& factory);
	static void saveMap(toponav_core::TopoMap* map, std::string filename, NodeStorageInterfacePtr const& nodeStore, EdgeStorageInterfacePtr const& edgeStore);
};

} // namespace topo_nav
