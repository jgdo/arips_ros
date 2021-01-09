#include "toponav_core/TopoPath.h"

namespace toponav_core {

void TopoPath::RegionMovement::visitPlanVisitor(TopoPath::PathVisitor *visitor) {
	visitor->visitRegionMovement(this);
}

void TopoPath::Transition::visitPlanVisitor(TopoPath::PathVisitor *visitor) {
	visitor->visitTransition(this);
}

void TopoPath::visitPlan(PathVisitor &visitor) const {
	for(auto& e: pathElements)
		e->visitPlanVisitor(&visitor);
}

void ComposedPlanVisitor::visitRegionMovement(TopoPath::RegionMovement const* movement) {
	regionMovementVis.visitRegionMovement(movement);
}

void ComposedPlanVisitor::visitTransition(TopoPath::Transition const* transition) {
	transitionVis.visitTransition(transition);
}

void LambdaPlanVisitor::visitRegionMovement(TopoPath::RegionMovement const* movement) {
	regionMovementVis(movement);
}

void LambdaPlanVisitor::visitTransition(TopoPath::Transition const* transition) {
	transitionVis(transition);
}

} // namespace topo_nav 

