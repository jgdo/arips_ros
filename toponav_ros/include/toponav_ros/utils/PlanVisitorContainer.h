#pragma once

#include <toponav_core/TopoPath.h>
#include "NamedModuleContainer.h"

namespace toponav_ros {

class PLanVisitorContainer: public toponav_core::TopoPath::PathVisitor {
public:
	inline void addMovementVisitor(std::string type, toponav_core::TopoPath::MovementVisitor::Ptr const& plugin) {
		movementContainer.addModule(type, plugin);
	}
	
	inline void addTransitionVisitor(std::string type, toponav_core::TopoPath::TransitionVisitor::Ptr const& plugin) {
		transitionContainer.addModule(type, plugin);
	}
	
	virtual void visitRegionMovement(toponav_core::TopoPath::RegionMovement const* movement) override;
	
	virtual void visitTransition(toponav_core::TopoPath::Transition const* transition) override;
	
private:
	NamedModuleContainer<toponav_core::TopoPath::MovementVisitor> movementContainer;
	NamedModuleContainer<toponav_core::TopoPath::TransitionVisitor> transitionContainer;
};

} // namespace topo_nav
