#pragma once

#include <toponav_core/interfaces/NodePlanningInterface.h>
#include <toponav_core/TopoMap.h>
#include "toponav_ros/interfaces/MapPoseInterface.h"

#include <boost/signals2.hpp>
#include <toponav_ros/utils/EdgeModuleContainer.h>
#include <toponav_ros/utils/NodeModuleContainer.h>
#include <toponav_core/TopoPlannerBase.h>

#include "RosContext.h"

#include "utils/PlanQueue.h"

namespace toponav_ros {

class PlanningContext: public RosContext {
public:
  toponav_core::TopoMapPtr topoMap;
  boost::signals2::signal<void()> mapChanged;
  
  toponav_core::TopoPlannerBasePtr pathPlanner;
	MapPosePluginPtr poseService;
  PlanQueue planQueue;
  
  toponav_core::NodePlanningInterfacePtr nodePlanner;
  toponav_core::EdgePlanningInterfacePtr edgePlanner;
  
  template<class T>
  inline bool isModuleType(const toponav_core::TopoMap::Node *node) const {
    return nodePlanner->getModuleType(node) == T::className;
  }
  
  template<class T>
  inline bool isModuleType(const toponav_core::TopoMap::Edge *edge) const {
    return edgePlanner->getModuleType(edge) == T::className;
  }
};

class PlanningContextHolder {
public:
	inline PlanningContextHolder(PlanningContext& context): _context(context) {}
	
protected:
  PlanningContext& _context;
};

} // namespace topo_nav 

