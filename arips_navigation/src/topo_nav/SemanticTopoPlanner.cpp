#include <arips_navigation/topo_nav/SemanticTopoPlanner.h>

#include <arips_navigation/topo/DijkstraTopoPlanner.h>
#include <arips_navigation/topo/TopoMap.h>

static constexpr auto ApproachPointDist_m = 1.05;

class DoorPlanner : public DoorPlanningInterface {
public:
    std::optional<double> computeTransitionCost(const TopoDoor* edge,
                                                Pose2D approachPose) override {
        return 1.0;
    }

    bool areEdgesCoupled(const TopoDoor* edgeA, const TopoDoor* edgeB) override {
        return edgeA->_otherDoorEdge == edgeB;
    }
};

class RoomPlanner : public RoomPlanningInterface {
public:
    RoomPlanner(Locomotion& locomotion, const Costmap& costmap)
        : mLocomotion{locomotion}, mCostmap{costmap} {}

    std::pair<std::optional<double>, std::optional<std::vector<Pose2D>>>
    computeCostsOnRegion(const TopoRoom* node, Pose2D start, Pose2D goal) override {
        const auto potmap = mLocomotion.makePlan(mCostmap, start, goal);

        if (!potmap) {
            return {};
        }

        return {potmap->atPos(potmap->goal().point), potmap->traceCurrentPath(start)};
    }

    Locomotion& mLocomotion;
    const Costmap& mCostmap;
};

struct SemanticTopoPlanner::Pimpl {
    explicit Pimpl(Locomotion& locomotion) : mLocomotion{locomotion} {}

    std::optional<TopoPath> plan(const Costmap& costmap,
                                 const arips_semantic_map_msgs::SemanticMap& semanticMap,
                                 Pose2D start, Pose2D goal) {
        TopoMap map;
        auto* node = map.addNode("world");

        int doorCount = 0;
        for (const auto& doorMsg : semanticMap.doors) {
            auto* doorA = map.addEdge(node, node, "door_a_" + std::to_string(doorCount));
            auto* doorB = map.addEdge(node, node, "door_b_" + std::to_string(doorCount));
            doorCount++;

            const Vector2d extent{doorMsg.extent.x, doorMsg.extent.y};
            const Vector2d pivot{doorMsg.pivot.x, doorMsg.pivot.y};
            const auto d = extent - pivot;
            const auto center = pivot + d * 0.5;
            const auto approachDiff = Vector2d{-d.y(), d.x()}.normalized() * ApproachPointDist_m;
            const auto approachA = center + approachDiff;
            const auto approachB = center - approachDiff;
            const auto angleA = atan2(-approachDiff.y(), -approachDiff.x());
            const auto angleB = atan2(approachDiff.y(), approachDiff.x());

            doorA->_approachPose = Pose2D{approachA, angleA};
            doorA->_exitPose = Pose2D{approachB, angleA};
            doorB->_approachPose = Pose2D{approachB, angleB};
            doorB->_exitPose = Pose2D{approachA, angleB};

            doorA->_otherDoorEdge = doorB;
            doorB->_otherDoorEdge = doorA;
        }

        auto roomPlanner = std::make_shared<RoomPlanner>(mLocomotion, costmap);
        DijkstraTopoPlanner planner{roomPlanner, mDoorPlanner};
        auto optPlan = planner.plan(&map, TopoPose2D{node, start}, TopoPose2D{node, goal});
        return optPlan;
    }

    Locomotion& mLocomotion;
    std::shared_ptr<DoorPlanner> mDoorPlanner = std::make_shared<DoorPlanner>();
};

SemanticTopoPlanner::SemanticTopoPlanner(Locomotion& locomotion)
    : mPimpl{std::make_unique<Pimpl>(locomotion)} {}

SemanticTopoPlanner::~SemanticTopoPlanner() = default;

std::optional<TopoPath>
SemanticTopoPlanner::plan(const Costmap& costmap,
                          const arips_semantic_map_msgs::SemanticMap& semanticMap, Pose2D start,
                          Pose2D goal) {
    return mPimpl->plan(costmap, semanticMap, start, goal);
}
