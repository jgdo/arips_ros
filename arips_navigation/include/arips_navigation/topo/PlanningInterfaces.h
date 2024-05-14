#pragma once

#include <optional>

#include <arips_navigation/path_planning/PlanningMath.h>
#include <arips_navigation/topo/TopoMap.h>

class RoomPlanningInterface {
public:
    typedef std::shared_ptr<RoomPlanningInterface> Ptr;

    /**
     * @brief Given a start position and a goal area, compute the movement costs to the goal and the
     * actual goal position.
     *
     * The method might save some information about the computation, which will be preserved in the
     * final topological path.
     *
     * @param node topo node representing the region
     * @param start start position
     * @param end goal area
     * @return costs. Null if no path was found
     */
    virtual std::optional<double> computeCostsOnRegion(const TopoRoom* node, Pose2D start,
                                                       Pose2D end) = 0;

    /**
     * @brief Compute minimum costs between two areas.
     *
     * The costs _may never_ overestimate the actual distance as computed by computeCostsOnRegion().
     * Implementation should be fast. Leave default implementation returning 0.0 if no simple
     * heuristic available for this region type (not recommended!).
     *
     * @param region region where start and end lie on
     * @param startData start area
     * @param endData end area
     * @return minimum costs between two start and end
     */
    virtual double getHeuristics(Pose2D start, Pose2D end) {
        return 0.0;
    } // no heuristics per default
};

typedef RoomPlanningInterface::Ptr RoomPlanningInterfacePtr;

class DoorPlanningInterface {
public:
    typedef DoorPlanningInterface BaseClass;
    typedef std::shared_ptr<DoorPlanningInterface> Ptr;

    /**
     * @brief Compute the transition costs and the exit pose of an edge given an approach pose.
     *
     * May store some data in pathData object which will be contained in the final topological path.
     *
     * @param edge
     * @param approachPose approach pose inside the edge's source node region
     * @param pathData output, optional, for storing data in the final path.
     * @return costs and exit position inside the destination region. Null indicates that transition
     * cannot be made.
     */
    virtual std::optional<double> computeTransitionCost(const TopoDoor* edge,
                                                        Pose2D approachPose) = 0;

    /**
     * @brief Minimum costs for transitioning this edge. May never overestimate the actual costs as
     * computed by computeTransitionCost().
     *
     * Implementation should be fast. Keep default implementation returning 0.0 if no admissible
     * heuristic exists for this edge (not recommended!).
     *
     * @param edge
     * @return Minimum edge transition costs.
     */
    virtual double getHeuristics(TopoDoor const* edge) { return 0.0; } // no heuristics per default

    /**
     * @brief Indicates whether two topo edges are coupled, i.e. represent different crossing
     * directions of the same physical object.
     * @param edgeA
     * @param edgeB
     * @return true if A and B are coupled, false otherwise.
     */
    virtual bool areEdgesCoupled(const TopoDoor* edgeA, const TopoDoor* edgeB) = 0;
};

typedef DoorPlanningInterface::Ptr DoorPlanningInterfacePtr;

struct PlanningStatistics {
    std::pair<size_t, size_t> computeRegionCostsCalls;
    std::pair<size_t, size_t> computeTransitionCostsCalls;

    double finalPlanningTime;
};
