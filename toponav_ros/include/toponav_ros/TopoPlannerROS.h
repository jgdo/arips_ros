#pragma once

#include <ros/ros.h>

#include <toponav_core/TopoMap.h>
#include <toponav_core/DijkstraTopoPlanner.h>

#include <toponav_msgs/GetPlan.h>

#include <toponav_ros/TopoMapYamlStorage.h>
#include <toponav_ros/MapEditor.h>
#include <toponav_ros/PlanningContext.h>
#include <toponav_ros/TopoPathVisualizer.h>
#include <toponav_ros/interfaces/CostsProfileInterface.h>

namespace toponav_ros {

class TopoPlannerROS {
public:
  bool init(std::string const& name, std::shared_ptr<toponav_core::ModuleContainer> const& factory, tf2_ros::Buffer* tfBuffer);
  
  inline bool isInitialized() const {
    return (bool)_context.topoMap;
  }
  
  inline PlanningContext& getContext() {
    return _context;
  }
  
  inline TopoPathVisualizer& getPathViz() {
    return *_pathViz;
  }
  
  void setCostsProfile(std::string const& profile);
  
protected:
  PlanningContext _context;
  std::shared_ptr<TopoPathVisualizer> _pathViz;
  std::shared_ptr<toponav_core::ModuleContainer> _factory;
  std::shared_ptr<MapEditor> _mapEditor;
  CostsProfileInterfacePtr cost_profile_;
};
  
} // namespace topo_nav
