#pragma once

#include <string>

namespace toponav_ros {

struct CommonCostProfiles  {
  static const std::string DEFAULT_PROFILE; //  = "default";
  
  static const std::string TIME_COSTS, DISTANCE_COSTS, ENERGY_COSTS;
};
  
} // namespace topo_nav
