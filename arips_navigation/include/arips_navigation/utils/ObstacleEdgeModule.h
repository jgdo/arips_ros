#include <arips_navigation/FlatGroundModule.h>
#include "ApproachExitVisualizer.h"
#include "ApproachExit3DEdgeModuleBase.h"

namespace toponav_ros {

class ObstacleEdgeModule;

template <>
struct BaseTraits<ObstacleEdgeModule>: public ApproachExit3DEdgeModuleBaseTraits {
	struct CostSettings {
		double time_costs = 1;
		double energy_costs = 1;
		double distance_costs = 1;
	};
	
	static inline CostSettings positiveObstacleCostSettings() {
		return CostSettings(); // Change fields if required
	}
	
	static inline CostSettings negativeObstacleCostSettings() {
		return CostSettings(); // Change fields if required
	}
  
  /**
   * For usage as map user data
   */
  struct MapData {};
  
  /**
   * For usage as edge user data
   */
  struct EdgeData {
    std::string obstacleName;
    CostSettings costs;
		
		enum ObstacleType {
			POSITIVE_OBSTACLE, NEGATIVE_OBSTACLE
		} type;
  };
};

class ObstacleEdgeModule: public EdgeModuleHelperBase<ObstacleEdgeModule>, public toponav_core::EdgePlanningInterface, public ApproachExitVisualizer, public MapEditorModule, public BaseTraits<ObstacleEdgeModule>, public CostsProfileHolderBase {
public:
  typedef std::shared_ptr<ObstacleEdgeModule> Ptr;
  
  static const std::string className;
  
  typedef typename FlatGroundModule::PositionData PositionData;
  typedef typename FlatGroundModule::PositionDataPtr PositionDataPtr;
	
	ObstacleEdgeModule(PlanningContext& context);
  
  virtual void initApproachData(const toponav_core::TopoMap::Edge *edge, toponav_core::AbstractApproachExitData* approachData) override;
  
  virtual void initExitData(const toponav_core::TopoMap::Edge *edge, toponav_core::AbstractApproachExitData* exitData) override;
  
  std::pair<double, toponav_core::LocalPositionConstPtr>
  computeTransitionCost(const toponav_core::TopoMap::Edge *edge, const toponav_core::LocalPosition &approachPose, boost::any *pathData) override;
  
  double getHeuristics(toponav_core::TopoMap::Edge const *edge) override;
  
  bool areEdgesCoupled(const toponav_core::TopoMap::Edge *edgeA, const toponav_core::TopoMap::Edge *edgeB) override;
  
  std::string getModuleType(const toponav_core::TopoMap::Edge *edge) override;
	
	void initializeVisualization(MapEditor *editor) override;
	
	void visualizeEdge(const toponav_core::TopoMap::Edge *edge) override;
  
  void activate() override;
  
  void deactivate() override;
  
  void poseCallback(geometry_msgs::PoseStamped const &pose) override;
	
	toponav_core::TopoMap::Edge *createObstacleEdge(const tf2::Stamped<tf2::Transform> &approach, const tf2::Stamped<tf2::Transform> &exit,
                                                    std::string const &obstacleName, CostSettings const &costs, EdgeData::ObstacleType obstacleType);
  
protected:
  double getCosts(CostSettings const&) const;
	
	MoveGoalProperties goal_properties;
};

typedef ObstacleEdgeModule::Ptr ObstacleEdgeModulePtr;
  
} // namespace topo_nav