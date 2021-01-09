#pragma once

#include <string>
#include <memory>
#include <vector>
#include <functional>

#include <boost/any.hpp>

#include "TopoMap.h"

namespace toponav_core {

class TopoPath {
public:
	class RegionMovement;
	class Transition;
	
	class PathVisitor {
	public:
		typedef std::shared_ptr<PathVisitor> Ptr;
		
		inline virtual ~PathVisitor() {}
		
		inline virtual void visitRegionMovement(RegionMovement const* movement) {}
		inline virtual void visitTransition(Transition const* transition) {}
	};
	
	class MovementVisitor {
	public:
		typedef std::shared_ptr<MovementVisitor> Ptr;
		
		inline virtual ~MovementVisitor() {}
		
		inline virtual void visitRegionMovement(RegionMovement const* movement) {}
	};
	
	class TransitionVisitor {
	public:
		typedef std::shared_ptr<TransitionVisitor> Ptr;
		
		inline virtual ~TransitionVisitor() {}
		
		inline virtual void visitTransition(Transition const* transition) {}
	};
	
	class PathSegment {
	public:
		const double costs;
		
		typedef std::shared_ptr<PathSegment> Ptr;
		
    inline PathSegment(double costs): costs(costs) {}
		inline virtual ~PathSegment() {}
		
		virtual void visitPlanVisitor(PathVisitor* visitor) = 0;
	};
	
	class RegionMovement: public PathSegment{
	public:
		GlobalPosition start, goal;
		const boost::any pathData;
		
		inline RegionMovement(const GlobalPosition &start, const GlobalPosition &goal, double costs, const boost::any &pathData) :
        PathSegment(costs), start(start), goal(goal), pathData(pathData) { }
		
		virtual void visitPlanVisitor(PathVisitor* visitor) override;
	};
	
	class Transition: public PathSegment {
	public:
		GlobalPosition approachPoint, exitPoint;
		const TopoMap::Edge* topoEdge;
		
		const boost::any pathData;
		
		inline Transition(const GlobalPosition &approachPoint, const GlobalPosition &exitPoint, double costs, const TopoMap::Edge *topoEdge,
				   const boost::any &pathData) : PathSegment(costs), approachPoint(approachPoint), exitPoint(exitPoint), topoEdge(topoEdge),
												 pathData(pathData) { }
		
		virtual void visitPlanVisitor(PathVisitor* visitor) override;
	};
	
	std::vector<PathSegment::Ptr> pathElements;
	
	DataMap userdata;
	double totalCosts;
	
	void visitPlan(PathVisitor &visitor) const;
  
  inline TopoPath() {}
  inline TopoPath(const TopoPath& other) = default;
  inline TopoPath(TopoPath&& other): pathElements(std::move(other.pathElements)) {}
};

class ComposedPlanVisitor: public TopoPath::PathVisitor {
public:
	inline ComposedPlanVisitor(TopoPath::MovementVisitor& rmv, TopoPath::TransitionVisitor& tv):
			regionMovementVis(rmv), transitionVis(tv) {
	}
	
	virtual void visitRegionMovement(TopoPath::RegionMovement const* movement) override ;
	
	virtual void visitTransition(TopoPath::Transition const* transition) override;

private:
	TopoPath::MovementVisitor regionMovementVis;
	TopoPath::TransitionVisitor transitionVis;
};

class LambdaPlanVisitor: public TopoPath::PathVisitor {
public:
	typedef std::function<void (TopoPath::RegionMovement const*)> TRMVis;
	typedef std::function<void (TopoPath::Transition const*)> TTVis;
	
	inline LambdaPlanVisitor(const TRMVis& rmv, TTVis const& tv):
			regionMovementVis(rmv), transitionVis(tv) {
	}
	
	virtual void visitRegionMovement(TopoPath::RegionMovement const* movement) override;
	
	virtual void visitTransition(TopoPath::Transition const* transition) override;

private:
	TRMVis regionMovementVis;
	TTVis transitionVis;
};
	
} // namespace topo_nav 

