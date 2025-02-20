#include <arips_navigation/topo/TopoPath.h>


void TopoPath::Movement::visitPlanVisitor(TopoPath::PathVisitor *visitor) {
	visitor->visitMovement(this);
}

void TopoPath::Transition::visitPlanVisitor(TopoPath::PathVisitor *visitor) {
	visitor->visitTransition(this);
}

void TopoPath::visitPlan(PathVisitor &visitor) const {
	for(auto& e: pathElements)
		e->visitPlanVisitor(&visitor);
}

void ComposedPlanVisitor::visitMovement(TopoPath::Movement const* movement) {
	MovementVis.visitMovement(movement);
}

void ComposedPlanVisitor::visitTransition(TopoPath::Transition const* transition) {
	transitionVis.visitTransition(transition);
}

void LambdaPlanVisitor::visitMovement(TopoPath::Movement const* movement) {
	MovementVis(movement);
}

void LambdaPlanVisitor::visitTransition(TopoPath::Transition const* transition) {
	transitionVis(transition);
}

