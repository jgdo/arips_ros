#pragma once

#include <functional>
#include <memory>
#include <vector>

#include <arips_navigation/topo/TopoMap.h>

class TopoPath {
public:
    class Movement;
    class Transition;

    class PathVisitor {
    public:
        typedef std::unique_ptr<PathVisitor> Ptr;

        virtual ~PathVisitor() {}

        virtual void visitMovement(Movement const* movement) {}
        virtual void visitTransition(Transition const* transition) {}
    };

    class MovementVisitor {
    public:
        typedef std::unique_ptr<MovementVisitor> Ptr;

        virtual ~MovementVisitor() {}

        virtual void visitMovement(Movement const* movement) {}
    };

    class TransitionVisitor {
    public:
        typedef std::unique_ptr<TransitionVisitor> Ptr;

        virtual ~TransitionVisitor() {}

        virtual void visitTransition(Transition const* transition) {}
    };

    class PathSegment {
    public:
        const double costs;

        typedef std::unique_ptr<PathSegment> Ptr;

        PathSegment(double costs) : costs(costs) {}
        virtual ~PathSegment() {}

        virtual void visitPlanVisitor(PathVisitor* visitor) = 0;
    };

    class Movement : public PathSegment {
    public:
        TopoPose2D start, goal;
        std::vector<Pose2D> pathPoints;

        Movement(TopoPose2D start, TopoPose2D goal, double costs, std::vector<Pose2D> pathPoints)
            : PathSegment(costs), start(start), goal(goal), pathPoints{std::move(pathPoints)} {}

        virtual void visitPlanVisitor(PathVisitor* visitor) override;
    };

    class Transition : public PathSegment {
    public:
        TopoPose2D approachPoint, exitPoint;
        const TopoDoor* topoEdge;

        Transition(TopoPose2D approachPoint, TopoPose2D exitPoint, double costs,
                   const TopoDoor* topoEdge)
            : PathSegment(costs), approachPoint(approachPoint), exitPoint(exitPoint),
              topoEdge(topoEdge) {}

        virtual void visitPlanVisitor(PathVisitor* visitor) override;
    };

    std::vector<PathSegment::Ptr> pathElements;

    double totalCosts;

    void visitPlan(PathVisitor& visitor) const;

    TopoPath() {}
    TopoPath(const TopoPath& other) = default;
    TopoPath(TopoPath&& other) = default;
};

class ComposedPlanVisitor : public TopoPath::PathVisitor {
public:
    ComposedPlanVisitor(TopoPath::MovementVisitor& rmv, TopoPath::TransitionVisitor& tv)
        : MovementVis(rmv), transitionVis(tv) {}

    virtual void visitMovement(TopoPath::Movement const* movement) override;

    virtual void visitTransition(TopoPath::Transition const* transition) override;

private:
    TopoPath::MovementVisitor MovementVis;
    TopoPath::TransitionVisitor transitionVis;
};

class LambdaPlanVisitor : public TopoPath::PathVisitor {
public:
    typedef std::function<void(TopoPath::Movement const*)> TRMVis;
    typedef std::function<void(TopoPath::Transition const*)> TTVis;

    LambdaPlanVisitor(const TRMVis& rmv, TTVis const& tv) : MovementVis(rmv), transitionVis(tv) {}

    virtual void visitMovement(TopoPath::Movement const* movement) override;

    virtual void visitTransition(TopoPath::Transition const* transition) override;

private:
    TRMVis MovementVis;
    TTVis transitionVis;
};
