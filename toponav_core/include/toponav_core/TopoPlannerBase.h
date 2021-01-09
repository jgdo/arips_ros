#pragma once

#include "TopoMap.h"
#include "TopoPath.h"

namespace toponav_core {

class TopoPlannerBase {
public:
  typedef std::shared_ptr<TopoPlannerBase> Ptr;
  
  /**
   * @brief Contains statistics about a path search.
   */
  struct Statistics {
    std::map<std::string, std::pair<size_t, size_t>> computeRegionCostsCalls;
    std::map<std::string, std::pair<size_t, size_t>> computeTransitionCostsCalls;
    
    double finalPlanningTime;
  };
  
  /**
   * @brief Find the shortest path between start and goal on a topological map.
   *
   * @param topoMap
   * @param start
   * @param goal
   * @param path output, if nut null, path should be returned here
   * @param statisticsOutput output, if not null, statistics about the search might be returned here.
   *
   * @return true if path could be found, otherwise false.
   */
  virtual bool plan(TopoMap const *topoMap, const GlobalPosition &start, const GlobalPosition &goal, TopoPath *path, Statistics *statisticsOutput) = 0;
};

typedef TopoPlannerBase::Ptr TopoPlannerBasePtr;

} // namespace topo_nav