#pragma once

#include <arips_navigation/topo/PlanningInterfaces.h>
#include <arips_navigation/topo/PlanningMap.h>
#include <arips_navigation/topo/TopoPath.h>

#include "MapTransformer.h"

class DijkstraTopoPlanner {
public:
    struct Config {
        bool prePathDiscovery = true; // remove deadlock nodes before planning
        bool useRegionCostsHeuristic =
            true; // use heuristics to compute min transition costs before computing actual costs
        bool useBackDijkstraHeuristic =
            true; // first perform dijkstra from goal to start using edge heuristics as costs
    };

    // https://gcc.gnu.org/bugzilla/show_bug.cgi?id=88165
    static Config defaultConfig() {
        return Config{};
    }

    DijkstraTopoPlanner(RoomPlanningInterfacePtr roomPlanner, DoorPlanningInterfacePtr doorPlanner,
                        DijkstraTopoPlanner::Config const& config = defaultConfig());

    std::optional<TopoPath> plan(TopoMap const* topoMap, TopoPose2D start, TopoPose2D end,
                                 PlanningStatistics* statisticsOutput=nullptr);

protected:
    struct DataContainerProvider {
        template <class ParentNode, class ParentEdge> struct Node {
            Pose2D pose;

            double hVal;
            bool backDijkstraVisited = false;

            struct PredEntry {
                // If predEdge is null, this is a start node

                Pose2D localPosition; // if null, it means the real costs are not computed yet
                double costFromStart;
                ParentEdge* predEdge;

                inline PredEntry(Pose2D pos, double costs, ParentEdge* pred)
                    : localPosition(pos), costFromStart(costs), predEdge(pred) {}

                bool areCostsReal() const { 
                    return !predEdge || predEdge->costsState == LocalEdge::REAL; 
                }
            };

            std::map<double, PredEntry> predMap;

            inline Node(double h) : hVal(h) {}

            inline void reset(double h) {
                predMap.clear();
                hVal = 0;
                backDijkstraVisited = false;
                hVal = h;
            }

            inline double minCostsFromStart() const {
                return predMap.empty() ? ParentNode::infCosts : predMap.begin()->first;
            }

            inline double getRanking() const { return minCostsFromStart() + hVal; }
        };

        template <class, class> struct Edge {
            void reset() {}
        };
    };

    DijkstraTopoPlanner::Config _config;

    RoomPlanningInterface::Ptr nodePlanner;
    DoorPlanningInterface::Ptr edgePlanner;

    PlanningMap<DataContainerProvider> localMap;
    typedef PlanningMap<DataContainerProvider>::Node LocalNode;
    typedef PlanningMap<DataContainerProvider>::Edge LocalEdge;

    std::set<LocalNode*> mainQueue;

    std::pair<size_t, size_t> regionCostsCount; // number of computeCostsOnRegion() calls counter
    std::pair<size_t, size_t> transitionCostsCount; // number of computeTransitionCost() calls counter (heuristic, real)

    void initLocalMap(TopoMap const* topoMap, TopoPose2D start, TopoPose2D end);

    LocalNode* getAndRemoveLowestCostNode();

    // void expandLocalMap(LocalNode *n);

    bool updateCostAndPred(LocalEdge* v, bool evaluateReal);

    inline bool mayVisitEdge(const LocalEdge* e) { return e->b->state == LocalNode::DISCOVERED; }

    void calcEdgeHeuristics(LocalEdge* e);

    void doBackDijkstraHeuristic();

    void bfsFromGoal();
};
