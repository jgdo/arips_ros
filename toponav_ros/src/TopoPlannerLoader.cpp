#include <toponav_ros/TopoPlannerLoader.h>
#include <toponav_core/DijkstraTopoPlanner.h>

#include <ros/ros.h>
#include <toponav_core/PathRepairTopoPlanner.h>

namespace toponav_ros {

using namespace toponav_core;

TopoPlannerBasePtr TopoPlannerLoader::loadTopoPlanner(std::string const &type, std::string const &name, ModuleContainer& factory) {
  ros::NodeHandle pnh("~/" + name);
  
  if(type == "DijkstraTopoPlanner") {
    DijkstraTopoPlanner::Config dijkstraConfig;
    pnh.getParam("dijkstra_discovery", dijkstraConfig.prePathDiscovery);
    pnh.getParam("dijkstra_region_heuristics", dijkstraConfig.useRegionCostsHeuristic);
    pnh.getParam("dijkstra_back_heuristics", dijkstraConfig.useBackDijkstraHeuristic);
    
    return std::make_shared<DijkstraTopoPlanner>(&factory, dijkstraConfig);
  } else if(type == "PathRepairTopoPlanner") {
    return std::make_shared<PathRepairTopoPlanner>(&factory);
  } else {
    ROS_ERROR_STREAM("Could not load topo planner of type '" << type << "'.");
    return TopoPlannerBasePtr();
  }
}
  
} // namespace topo_nav