#pragma once

#include <toponav_core/interfaces/EdgePlanningInterface.h>
#include "TopoPlannerBase.h"
#include "ModuleContainer.h"
#include "PlanningMap.h"
#include "MapTransformer.h"

namespace toponav_core {

class DijkstraTopoPlanner: public TopoPlannerBase {
public:
  struct Config {
    bool prePathDiscovery = true; // remove deadlock nodes before planning
    bool useRegionCostsHeuristic = true; // use heuristics to compute min transition costs before computing actual costs
    bool useBackDijkstraHeuristic = true; // first perform dijkstra from goal to start using edge heuristics as costs
  };
  
  DijkstraTopoPlanner(ModuleContainer* factory, DijkstraTopoPlanner::Config const& config);
  
  virtual bool plan(TopoMap const *topoMap, const GlobalPosition &start, const GlobalPosition &end, TopoPath *plan,
                    TopoPlannerBase::Statistics *statisticsOutput) override;

protected:
  struct DataContainerProvider {
    template<class ParentNode, class ParentEdge>
    struct Node {
      AbstractApproachExitData approachExitData;
      
      double hVal;
      bool backDijkstraVisited = false;
      
      struct PredEntry {
        // at least one of localPosition or predEdge must not be null !!!
        
        LocalPositionConstPtr localPosition; // if null, it means the real costs are not computed yet
        double costFromStart;
        ParentEdge* predEdge;
        
        inline PredEntry(const LocalPositionConstPtr& pos, double costs, ParentEdge* pred):
            localPosition(pos), costFromStart(costs), predEdge(pred) {
        }
        
        inline bool areCostsReal() const {
          return !!localPosition;
        }
      };
      
      std::map<double, PredEntry> predMap;
      
      inline Node(double h):
          hVal(h) {
      }
      
      inline void reset(double h) {
        predMap.clear();
        hVal = 0;
        backDijkstraVisited = false;
        hVal = h;
      }
      
      inline double minCostsFromStart() const {
        return predMap.empty()? ParentNode::infCosts : predMap.begin()->first;
      }
      
      inline double getRanking() const {
        return minCostsFromStart() + hVal;
      }
    };
    
    template<class, class>
    struct Edge {
      AbstractPathData pathData;
      
      inline void reset() {
        pathData = boost::none;
      }
    };
  };
  
  DijkstraTopoPlanner::Config _config;
  
  NodePlanningInterface::Ptr nodePlanner;
  EdgePlanningInterface::Ptr edgePlanner;
  
  PlanningMap<DataContainerProvider> localMap;
  typedef PlanningMap<DataContainerProvider>::Node LocalNode;
  typedef PlanningMap<DataContainerProvider>::Edge LocalEdge;
  
  GlobalPosition start, end;
  
  std::set<LocalNode*> mainQueue;
  
  std::map<std::string, std::pair<size_t, size_t>> regionCostsCount; // region type -> number of computeCostsOnRegion() calls counter
  std::map<std::string, std::pair<size_t, size_t>> transitionCostsCount; // transition type -> number of computeTransitionCost() calls counter (heuristic, real)
  
  void initLocalMap(TopoMap const* topoMap);
  
  LocalNode* getAndRemoveLowestCostNode();
  
  // void expandLocalMap(LocalNode *n);
  
  bool updateCostAndPred(LocalEdge* v, bool evaluateReal);
  
  inline bool mayVisitEdge(const LocalEdge* e) {
    return e->b->state == LocalNode::DISCOVERED;
  }
  
  void calcEdgeHeuristics(LocalEdge* e);
  
  void doBackDijkstraHeuristic();
  
  void bfsFromGoal();
};


}; // namespace topo_nav
