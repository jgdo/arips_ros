#pragma once

#include <toponav_core/TopoMap.h>

#include <boost/shared_ptr.hpp>

namespace toponav_core {

/**
 * @brief Planning interface for transitions (topo edges) between regions (topo nodes) as used by the topo planner.
 */
class EdgePlanningInterface {
public:
	typedef EdgePlanningInterface BaseClass;
	typedef std::shared_ptr<EdgePlanningInterface> Ptr;
	
	/**
   * @brief Initialize the approach area data of given edge into the given object.
   *
   * The approach area data object must be handleable by the module for the edge's source node's region as goal parameter
   * of the NodePlanningInterface::computeCostsOnRegion() method.
   *
   * @param edge
   * @param approachData output, must not be empty (boost::none) after this operation!!!
   */
	virtual void initApproachData(const TopoMap::Edge *edge, AbstractApproachExitData* approachData) = 0;
	
	/**
   * @brief Initialize the exit area data of given edge into the given object.
   *
   * The exit area data object must be handleable as by the module for the edge's destination node's region as start
   * parameter of the NodePlanningInterface::getHeuristics() method.
   *
   * @param edge
   * @param exitData MUST NOT be empty (i.e. boost::none) after this operation!!!
   */
	virtual void initExitData(const TopoMap::Edge *edge, AbstractApproachExitData* exitData) = 0;
  
  /**
   * @brief Compute the transition costs and the exit pose of an edge given an approach pose.
   *
   * May store some data in pathData object which will be contained in the final topological path.
   *
   * @param edge
   * @param approachPose approach pose inside the edge's source node region
   * @param pathData output, optional, for storing data in the final path.
   * @return costs and exit position inside the destination region. Null indicates that transition cannot be made.
   */
  virtual std::pair<double, LocalPositionConstPtr> computeTransitionCost(const TopoMap::Edge* edge, const LocalPosition &approachPose, boost::any *pathData) = 0;
	
  /**
   * @brief Minimum costs for transitioning this edge. May never overestimate the actual costs as computed by computeTransitionCost().
   *
   * Implementation should be fast. Keep default implementation returning 0.0 if no admissible heuristic exists for
   * this edge (not recommended!).
   *
   * @param edge
   * @return Minimum edge transition costs.
   */
	virtual double getHeuristics(TopoMap::Edge const* edge) { return 0.0; } // no heuristics per default
	
  /**
   * @brief Indicates whether two topo edges are coupled, i.e. represent different crossing directions of the same physical object.
   * @param edgeA
   * @param edgeB
   * @return true if A and B are coupled, false otherwise.
   */
	virtual bool areEdgesCoupled(const TopoMap::Edge* edgeA, const TopoMap::Edge* edgeB) = 0;
  
  /**
   * @brief String identifying this module. Usually the module's full class name.
   *
   * @param edge
   * @return module identifying string, e.g. class name
   */
  virtual std::string getModuleType(const TopoMap::Edge* edge) = 0;
};

typedef EdgePlanningInterface::Ptr EdgePlanningInterfacePtr;
	
}; // namespace topo_nav
