#pragma once

#include "TopoPlannerBase.h"


namespace toponav_core {

class ModuleContainer;

class PathRepairTopoPlannerImpl_;

class PathRepairTopoPlanner: public TopoPlannerBase {
public:
  PathRepairTopoPlanner(ModuleContainer* factory);
  
  virtual bool plan(TopoMap const *topoMap, const GlobalPosition &start, const GlobalPosition &end, TopoPath *plan, Statistics *statisticsOutput);

private:
  std::shared_ptr<PathRepairTopoPlannerImpl_> impl;
};
  
} // namespace topo_nav
