#pragma once

#include <toponav_core/TopoPlannerBase.h>
#include <toponav_core/ModuleContainer.h>

namespace toponav_ros {

class TopoPlannerLoader {
public:
  toponav_core::TopoPlannerBasePtr loadTopoPlanner(std::string const& type, std::string const& name, toponav_core::ModuleContainer& factory);
};

} // topo_nav;

