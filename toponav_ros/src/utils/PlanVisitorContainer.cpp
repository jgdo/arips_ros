#include <toponav_ros/utils/PlanVisitorContainer.h>

namespace toponav_ros {

using namespace toponav_core;

void PLanVisitorContainer::visitRegionMovement(const TopoPath::RegionMovement *movement) {
  movementContainer.getModule(movement->goal.node->getRegionType())->visitRegionMovement(movement);
}

void PLanVisitorContainer::visitTransition(TopoPath::Transition const *transition) {
  transitionContainer.getModule(transition->topoEdge->getTransitionType())->visitTransition(transition);
}
	
} // namespace topo_nav

