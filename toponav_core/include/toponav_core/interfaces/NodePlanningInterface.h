#pragma once

#include <toponav_core/TopoMap.h>
#include <memory>
#include <tf/transform_datatypes.h>

namespace toponav_core {

/**
 * @brief Planning interface for region modules as needed by the topological planner.
 *
 * Each topological node (region) represents a homogeneous motion state space. A region module is responsible for a
 * particular region type, e.g. flat area, rough terrain, ...
 */
class NodePlanningInterface {
public:
	typedef NodePlanningInterface BaseClass;
	typedef std::shared_ptr<NodePlanningInterface> Ptr;
	
  /**
   * @brief Given a start position and a goal area, compute the movement costs to the goal and the actual goal position.
   *
   * The method might save some information about the computation, which will be preserved in the final topological path.
   *
   * @param node topo node representing the region
   * @param start start position
   * @param end goal area
   * @param pathData output object for path data.
   * @return costs and actual goal pose. Null indicates that no local path could be found.
   */
  virtual std::pair<double, LocalPositionConstPtr> computeCostsOnRegion(const TopoMap::Node *node, LocalPosition const& start, AbstractApproachExitData const& end, AbstractPathData* pathData) = 0;
  
	/**
	 * @brief Convert a position on this region type to the approach/exit area format which can be later used as goal in computeCostsOnRegion().
	 *
	 * This prevents having multiple computeCostsOnRegion() methods for different goal types.
	 *
	 * @param pos position inside a region handled by this module.
	 * @param approachData output, optional, pos in approach/exit area format.
	 */
  virtual void convertGlobalPoseToApproachExitData(const GlobalPosition &pos, AbstractApproachExitData *approachData) = 0;
	
	/**
	 * @brief Compute minimum costs between two areas.
	 *
	 * The costs _may never_ overestimate the actual distance as computed by computeCostsOnRegion(). Implementation should be fast.
	 * Leave default implementation returning 0.0 if no simple heuristic available for this region type (not recommended!).
	 *
	 * @param region region where start and end lie on
	 * @param startData start area
	 * @param endData end area
	 * @return minimum costs between two start and end
	 */
	virtual double getHeuristics(TopoMap::Node const* region, AbstractApproachExitData const& startData, AbstractApproachExitData const& endData) { return 0.0; } // no heuristics per default
  
	/**
	 * @brief String idendifying this module. Usually the module's full class name.
	 *
	 * @param node
	 * @return module identifying string, e.g. class name
	 */
  virtual std::string getModuleType(const TopoMap::Node* node) = 0;
};

typedef NodePlanningInterface::Ptr NodePlanningInterfacePtr;

}; // namespace topo_nav
