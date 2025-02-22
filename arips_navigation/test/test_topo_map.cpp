//
// Created by jgdo on 22.03.21.
//

#include <gtest/gtest.h>

#include <arips_navigation/topo/DijkstraTopoPlanner.h>
#include <arips_navigation/topo/TopoMap.h>

class DummyRoomPlanner : public RoomPlanningInterface {
public:
    std::optional<double> computeCostsOnRegion(const TopoRoom* node, Pose2D start,
                                               Pose2D end) override {
        return (end.point - start.point).norm();
    }
};

class DummyDoorPlanner : public DoorPlanningInterface {
    public:
    std::optional<double> computeTransitionCost(const TopoDoor* edge,
                                                        Pose2D approachPose) override {
                                                            return 1.0;
                                                        }

    bool areEdgesCoupled(const TopoDoor* edgeA, const TopoDoor* edgeB) override {
        return false;
    }
};

TEST(TopoMap, DummyPlanning) {
    TopoMap map;
    auto* r1 = map.addNode("r1");
    auto* r2 = map.addNode("r2");
    auto* r3 = map.addNode("r3");

    auto* door_12 = map.addEdge(r1, r2, "door_12");
    door_12->_approachPose = Pose2D{{2.0, 0.0}, 2.0};
    door_12->_exitPose = Pose2D{{3.0, 0.0}, 3.0};
    auto* door_23 = map.addEdge(r2, r3, "door_23");
    door_23->_approachPose = Pose2D{{4.0, 0.0}, 4.0};
    door_23->_exitPose = Pose2D{{5.0, 0.0}, 5.0};

    auto roomPlanner = std::make_shared<DummyRoomPlanner>();
    auto doorPlanner = std::make_shared<DummyDoorPlanner>();

    DijkstraTopoPlanner planner{roomPlanner, doorPlanner};

    auto optPlan = planner.plan(&map, GlobalPose2D{r1, {{1.0, 0.0}, 1.0}}, GlobalPose2D{r3, {{6.0, 0.0}, 6.0}});

    EXPECT_TRUE(optPlan.has_value());
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
