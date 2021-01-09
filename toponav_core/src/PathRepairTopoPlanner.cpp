#include <toponav_core/PathRepairTopoPlanner.h>
#include <toponav_core/MapTransformer.h>
#include <toponav_core/PlanningMap.h>

#include <toponav_core/ModuleContainer.h>

#include <ctime>
static double get_cpu_time(){
  return (double)clock() / CLOCKS_PER_SEC;
}

namespace toponav_core {

class PathRepairTopoPlannerImpl_ {
public:
  PathRepairTopoPlannerImpl_(ModuleContainer *factory):
      nodePlanner(factory->getCreateModule<NodePlanningInterface>()),
      edgePlanner(factory->getCreateModule<EdgePlanningInterface>()) {
  }
  
  bool plan(TopoMap const *topoMap, const GlobalPosition &start, const GlobalPosition &end, TopoPath *plan, TopoPlannerBase::Statistics *statisticsOutput) {
    if(!start) {
      ROS_ERROR_STREAM("DijkstraTopoPlanner::plan() error: start is null");
      return false;
    }
    
    if(!end) {
      ROS_ERROR_STREAM("DijkstraTopoPlanner::plan() error: end is null");
      return false;
    }
    
    this->start = start;
    this->end = end;
    
    double now = get_cpu_time();
    initLocalMap(topoMap);
    double initTime = get_cpu_time() - now;
    ROS_INFO_STREAM("local map init took " << initTime);
    
    now = get_cpu_time();
    computeShortestPath();
  
    ROS_INFO_STREAM("Heuristic shortest path ready, performing path repair");
    std::list<LocalEdge*> edgePath;
    
    if(localMap.getEndNode()->gVal != LocalNode::infCosts) {
      LocalNode*n = localMap.getEndNode();
      while(true) {
        
        double rhs = LocalNode::infCosts;
        LocalEdge* predEdge = nullptr;
        for(LocalEdge* pe: n->predEdges) { // FIXME faster
          if(predEdge) {
            double tmp = pe->a->gVal + updateEdgeCosts(pe);
            if(tmp < rhs) {
              predEdge = pe;
              rhs = tmp;
            }
          } else {
            predEdge = pe;
            rhs = pe->a->gVal + updateEdgeCosts(pe);
          }
        }
        n->rhsVal = rhs;
        
        if(doubleNotEq(n->rhsVal, n->gVal)) {
          n->gVal = n->rhsVal;
          if(edgePath.size()) {
            n = edgePath.front()->b;
            edgePath.pop_front();
          }
        } else if(predEdge->costsState == LocalEdge::REAL) {
          if(edgePath.size()) {
            auto e = edgePath.front();
            computeRealCosts(e);
            n = e->b;
            edgePath.pop_front();
          } else {
            break;
          }
        } else if(predEdge->a == localMap.getStartNode()) {
          computeRealCosts(predEdge);
        } else {
          edgePath.push_front(predEdge);
          n = predEdge->a;
        }
      }
    }
  
    ROS_INFO_STREAM("topo plan ready, collecting final path");
  
    getFinalPath(&edgePath);
    
    double searchTime = get_cpu_time() - now;
    ROS_INFO_STREAM("search took " << searchTime);
    
    // localMap.saveToDot("after.dot");
    
    if(edgePath.size()) {
      // fill plan
      if(plan) {
        plan->totalCosts = localMap.getEndNode()->gVal;
        plan->pathElements.clear();
        
        for(LocalEdge* e: edgePath) {
          ROS_INFO_STREAM("cost = " << e->cost << " gVal = " << e->b->gVal);
          
          if(e->isSameRegion()) {
            plan->pathElements.push_back(std::make_shared<TopoPath::RegionMovement>(
                GlobalPosition(e->topoNode, e->a->lastLocalPos),
                GlobalPosition(e->topoNode, e->b->lastLocalPos),
                e->cost,
                e->pathData
            ));
          } else {
            plan->pathElements.push_back(std::make_shared<TopoPath::Transition>(
                GlobalPosition(e->a->region, e->a->lastLocalPos),
                GlobalPosition(e->b->region, e->b->lastLocalPos),
                e->cost,
                e->topoEdge,
                e->pathData
            ));
          }
        }
      }
      
      if(statisticsOutput) {
        statisticsOutput->computeRegionCostsCalls = regionCostsCount;
        statisticsOutput->computeTransitionCostsCalls = transitionCostsCount;
  
        statisticsOutput->finalPlanningTime = initTime + searchTime;
      }
      
      ROS_INFO_STREAM("Found topo plan of size " << edgePath.size() << " and costs " << localMap.getEndNode()->gVal << ", rhs = " << localMap.getEndNode()->rhsVal);
      ROS_INFO_STREAM("Planning time was " << (initTime + searchTime));
      for(auto& entry: regionCostsCount) {
        ROS_INFO_STREAM("region cost calls [" << entry.first << "] = " << entry.second.first << " | " << entry.second.second);
      }
  
      for(auto& entry: transitionCostsCount) {
        ROS_INFO_STREAM("transition cost calls [" << entry.first << "] = " << entry.second.first << " | " << entry.second.second);
      }
      
      return true;
    } else {
      ROS_INFO_STREAM("No topo plan found");
      
      return false;
    }
  }

private:
  struct DataContainerProvider {
    template<class ParentNode, class ParentEdge>
    struct Node {
      AbstractApproachExitData approachExitData;
      
      
      double gVal;
      double rhsVal;
      double hGoal = 0;
      
      LocalPositionConstPtr lastLocalPos;
      
      inline void reset(NodePlanningInterface* planner, const AbstractApproachExitData& goalData) {
        // predMap.clear();
        gVal = ParentNode::infCosts;
        rhsVal = ParentNode::infCosts;
        
        if(!approachExitData.empty())
          hGoal = 0; // planner->getHeuristics(getParentThis()->region, approachExitData, goalData);
        else {
          hGoal = 0;
          ROS_WARN("Computing heuristics to goal failed");
        }
        
        lastLocalPos.reset();
      }
      
      inline Node(NodePlanningInterface* planner, const AbstractApproachExitData& goalData):
          gVal(ParentNode::infCosts), rhsVal(ParentNode::infCosts) {
      }
      
      inline ParentNode* getParentThis() {
        return static_cast<ParentNode*>(this);
      }
      
      inline const ParentNode* getParentThis() const{
        return static_cast<const ParentNode*>(this);
      }
    };
    
    template<class, class ParentEdge>
    struct Edge {
      AbstractPathData pathData;
      
      inline void reset() {
        pathData = boost::none;
      }
      
      inline ParentEdge* getParentThis() {
        return static_cast<ParentEdge*>(this);
      }
    };
  };
  
  NodePlanningInterface::Ptr nodePlanner;
  EdgePlanningInterface::Ptr edgePlanner;
  
  PlanningMap<DataContainerProvider> localMap;
  typedef PlanningMap<DataContainerProvider>::Node LocalNode;
  typedef PlanningMap<DataContainerProvider>::Edge LocalEdge;
  
  GlobalPosition start, end;
  
  std::set<LocalNode*> queue;
  
  std::map<std::string, std::pair<size_t, size_t>> regionCostsCount; // region type -> number of computeCostsOnRegion() calls counter (heuristic, real)
  std::map<std::string, std::pair<size_t, size_t>> transitionCostsCount; // transition type -> number of computeTransitionCost() calls counter (heuristic, real)
  
  /*
  struct {
    bool prePathDiscovery = false;
  } _config; // FIXME
  
  */
  
  void initLocalMap(TopoMap const* topoMap) {
    queue.clear();
    regionCostsCount.clear();
    transitionCostsCount.clear();
    
    AbstractApproachExitData goalData;
    nodePlanner->convertGlobalPoseToApproachExitData(end, &goalData);
    
    if(topoMap->getVersionCount() != localMap.getLastTopoMapVersion()) {
      // MapTransformer::createLocalMap(*topoMap, localMap, nodePlanner, edgePlanner, _config.prePathDiscovery ? LocalNode::UNKNOWN : LocalNode::DISCOVERED, nodePlanner.get(), goalData);
      MapTransformer::createLocalMap(*topoMap, localMap, nodePlanner, edgePlanner, LocalNode::DISCOVERED, nodePlanner.get(), goalData);
      // localMap.resetStartEndPaths(_config.prePathDiscovery? LocalNode::UNKNOWN : LocalNode::DISCOVERED, nodePlanner.get(), goalData);
      localMap.resetStartEndPaths(LocalNode::DISCOVERED, nodePlanner.get(), goalData);
      ROS_INFO("Topo map changed, re-creating planning map");
    } else {
      // mark everything as discovered if no pre path discovery
      //localMap.resetStartEndPaths(_config.prePathDiscovery? LocalNode::UNKNOWN : LocalNode::DISCOVERED, nodePlanner.get(), goalData);
      localMap.resetStartEndPaths(LocalNode::DISCOVERED, nodePlanner.get(), goalData);
      ROS_INFO("Re-using planning map for Dijkstra");
    }
    
    localMap.setStartNode(start.node, nodePlanner.get(), goalData);
    localMap.getStartNode()->lastLocalPos = start.localPos;
    localMap.setEndNode(end.node, nodePlanner.get(), goalData);
    nodePlanner->convertGlobalPoseToApproachExitData(start, &localMap.getStartNode()->approachExitData);
    nodePlanner->convertGlobalPoseToApproachExitData(end, &localMap.getEndNode()->approachExitData);
    
    /*
    if(_config.prePathDiscovery) {
      localMap.discoverStartEndNodes();
    } else */ {
      localMap.getStartNode()->state = LocalNode::DISCOVERED;
      localMap.getEndNode()->state = LocalNode::DISCOVERED;
    }
    
    // localMap.saveToDot("before.dot");
    
    localMap.getStartNode()->rhsVal = 0;
    
    if(localMap.getStartNode()->state == LocalNode::DISCOVERED) {
      queue.insert(localMap.getStartNode());
    }
  }
  
  struct Key {
    double k1, k2;
    
    bool operator<(const Key& other) const {
      if(!doubleNotEq(k1, other.k1))
        return k1 < other.k1;
      else
        return k2 < other.k2;
    }
  };
  
  Key calcKey(LocalNode* node) {
    double m = std::min(node->gVal, node->rhsVal);
    return Key {m + node->hGoal, m} ;
  };
  
  std::set<LocalNode*>::iterator findLowestQ() {
    return std::min_element(queue.begin(), queue.end(), [this](LocalNode* a, LocalNode *b) {return calcKey(a) < calcKey(b); });;
  }
  
  LocalNode* getAndRemoveLowestCostNode()  {
    auto iter = findLowestQ();
    LocalNode* n = *iter;
    queue.erase(iter);
    n->state = LocalNode::VISITED;
    return n;
  }
  
  double updateEdgeCosts(LocalEdge* e) {
    if(e->costsState == LocalEdge::UNKNOWN) {
      if(e->isSameRegion()) {
        e->cost = nodePlanner->getHeuristics(e->topoNode, e->a->approachExitData, e->b->approachExitData);
        regionCostsCount[e->topoNode->getRegionType()].first++;
      } else {
        e->cost = edgePlanner->getHeuristics(e->topoEdge);
        transitionCostsCount[e->topoEdge->getTransitionType()].first++;
      }
      e->costsState = LocalEdge::HEURISTIC;
    }
    
    return e->cost;
  }
  
  inline static bool doubleNotEq(double a, double b) {
    return std::abs(a - b) > std::numeric_limits<double>::epsilon();
  }
  
  void updateVertex(LocalNode* u) {
    if(u != localMap.getStartNode()) {
      double m = LocalNode::infCosts;
      for(auto edge: u->predEdges) {
        m = std::min(m, edge->a->gVal + updateEdgeCosts(edge));
      }
      u->rhsVal = m;
    }
    
    queue.erase(u); // erase if any
    if(doubleNotEq(u->gVal, u->rhsVal)) { // FIXME
      queue.insert(u);
    }
  }
  
  void computeRealCosts(LocalEdge* e) {
    LocalPositionConstPtr pose;
    if (e->isTransitionEdge()) { // transition edge -> recompute exit pose and costs based on current approach pose
      std::tie(e->cost, pose) = edgePlanner->computeTransitionCost(e->topoEdge, *e->a->lastLocalPos, &e->pathData);
      if (!pose) {
        ROS_WARN_STREAM("DijkstraTopoPlanner: exit pose for edge '" << e->topoEdge->getName() << "' was computed to nullptr, ignoring");
        e->costsState = LocalEdge::REAL;
        e->cost = LocalNode::infCosts;
        return;
      }
      
      transitionCostsCount[e->topoEdge->getTransitionType()].second++; // TODO: really ignore failed calls?
    } else {
      std::tie(e->cost, pose) = nodePlanner->computeCostsOnRegion(e->b->region, *e->a->lastLocalPos, e->b->approachExitData, &e->pathData);
      if (!pose) {
        ROS_WARN_STREAM("DijkstraTopoPlanner: computeCostsOnRegion '" << e->b->region->getName()  << "' returned nullptr pose, ignoring");
        e->costsState = LocalEdge::REAL;
        e->cost = LocalNode::infCosts;
        return;
      }
      
      // ROS_INFO_STREAM("copute cost " << e->cost);
      
      regionCostsCount[e->topoNode->getRegionType()].second++; // TODO: really ignore failed calls?
    }
    
    e->costsState = LocalEdge::REAL;
    e->b->lastLocalPos = pose;
  }
  
  void computeShortestPath() {
    for(auto iter = findLowestQ(); !queue.empty() /* && ((calcKey(*iter) < calcKey(localMap.getEndNode())) or doubleNotEq(localMap.getEndNode()->rhsVal, localMap.getEndNode()->gVal))*/;  iter = findLowestQ()) {
      LocalNode* u = *iter;
      queue.erase(iter);
      
      if(u->gVal > u->rhsVal) {
        u->gVal = u->rhsVal;
      } else {
        u->gVal = LocalNode::infCosts;
        updateVertex(u);
      }
      
      for(auto e: u->edges)
        updateVertex(e->b);
    }
  }
  
  void getFinalPath(std::list<LocalEdge*>* path) {
    path->clear();
    
    if(localMap.getEndNode()->gVal == LocalNode::infCosts)
      return;
    
    for(LocalNode* n = localMap.getEndNode();;) {
      double rhs = LocalNode::infCosts;
      LocalEdge* predEdge = nullptr;
      for(LocalEdge* pe: n->predEdges) { // FIXME faster
        if(predEdge) {
          double tmp = pe->a->gVal + updateEdgeCosts(pe);
          if(tmp < rhs) {
            predEdge = pe;
            rhs = tmp;
          }
        } else {
          predEdge = pe;
          rhs = pe->a->gVal + updateEdgeCosts(pe);
        }
      }
      
      path->push_front(predEdge);
      
      if(predEdge->a == localMap.getStartNode()) {
        break;
      }
      
      n = predEdge->a;
    }
    
    return;
  }
};

PathRepairTopoPlanner::PathRepairTopoPlanner(ModuleContainer *factory): impl(new PathRepairTopoPlannerImpl_(factory)) {
  
}

bool PathRepairTopoPlanner::plan(TopoMap const *topoMap, const GlobalPosition &start, const GlobalPosition &end,
                                 TopoPath *plan, TopoPlannerBase::Statistics *statisticsOutput) {
  return impl->plan(topoMap, start, end, plan, statisticsOutput);
}

} // namespace topo_nav
