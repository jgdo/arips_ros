#include "toponav_core/DijkstraTopoPlanner.h"

#include <ctime>

static double get_cpu_time(){
  return (double)clock() / CLOCKS_PER_SEC;
}

namespace toponav_core {

DijkstraTopoPlanner::DijkstraTopoPlanner(ModuleContainer *factory, DijkstraTopoPlanner::Config const &config) :
    _config(config),
    nodePlanner(factory->getCreateModule<NodePlanningInterface>()),
    edgePlanner(factory->getCreateModule<EdgePlanningInterface>()) {
}

bool DijkstraTopoPlanner::plan(TopoMap const *topoMap, const GlobalPosition &start, const GlobalPosition &end,
                               TopoPath *plan, TopoPlannerBase::Statistics *statisticsOutput) {
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
  while(localMap.getEndNode()->state == LocalNode::DISCOVERED && !mainQueue.empty()) {
    LocalNode* u = getAndRemoveLowestCostNode();
    
    // costs are only estimated based on heuristics, so compute real costs and insert it back
    if(!u->predMap.begin()->second.areCostsReal()) {
      LocalEdge* edge = u->predMap.begin()->second.predEdge;
      u->predMap.erase(u->predMap.begin()); // erase this entry since updateCostAndPred() will generate a new one with real costs
      if(updateCostAndPred(edge, true))
        mainQueue.insert(u);
      continue;
    }
    
    u->state = LocalNode::VISITED;
    
    // std::cout << "visiting node " << u << " " << u->minCostsFromStart() << std::endl;
    
    // stop if it is guaranteed that no better path exists
    if(u->minCostsFromStart() > localMap.getEndNode()->minCostsFromStart())
      break;
    
    for(auto e: u->edges) {
      if (mayVisitEdge(e)) {
        bool ok = updateCostAndPred(e, !_config.useRegionCostsHeuristic);
        if(ok && mainQueue.find(e->b) == mainQueue.end()) {
          mainQueue.insert(e->b);
        }
      }
    }
  }
  double searchTime = get_cpu_time() - now;
  ROS_INFO_STREAM("search took " << searchTime);
  
  localMap.saveToDot("after.dot");
  
  if(localMap.getEndNode()->state == LocalNode::VISITED) {
    std::list<LocalNode*> nodePath;
    
    
    std::cout << "pred costs: ";
    for(auto& e: localMap.getEndNode()->predMap) {
      std::cout << e.first << " ";
    }
    std::cout << std::endl;
    
    // inverse direction such that nodePath contains nodes  ]start...end]
    // notice that the start node IS NOT in the nodePath!
    for (LocalNode* n = localMap.getEndNode(); n != localMap.getStartNode(); n = n->predMap.begin()->second.predEdge->a)
      nodePath.push_front(n);
    
    // fill plan
    if(plan) {
      plan->totalCosts = localMap.getEndNode()->predMap.begin()->second.costFromStart;
      plan->pathElements.clear();
      
      for(LocalNode* n: nodePath) {
        auto& predEntry = n->predMap.begin()->second;
        
        if(predEntry.predEdge->isSameRegion()) {
          plan->pathElements.push_back(std::make_shared<TopoPath::RegionMovement>(
              GlobalPosition(predEntry.predEdge->topoNode, predEntry.predEdge->a->predMap.begin()->second.localPosition),
              GlobalPosition(predEntry.predEdge->topoNode, predEntry.localPosition),
              predEntry.costFromStart - predEntry.predEdge->a->minCostsFromStart(),
              predEntry.predEdge->pathData
          ));
        } else {
          plan->pathElements.push_back(std::make_shared<TopoPath::Transition>(
              GlobalPosition(predEntry.predEdge->a->region, predEntry.predEdge->a->predMap.begin()->second.localPosition),
              GlobalPosition(predEntry.predEdge->b->region, predEntry.localPosition),
              predEntry.costFromStart - predEntry.predEdge->a->minCostsFromStart(),
              predEntry.predEdge->topoEdge,
              predEntry.predEdge->pathData
          ));
        }
      }
    }
    
    if(statisticsOutput) {
      statisticsOutput->computeRegionCostsCalls = regionCostsCount;
      statisticsOutput->computeTransitionCostsCalls = transitionCostsCount;
      
      statisticsOutput->finalPlanningTime = initTime + searchTime;
    }
    
    ROS_INFO_STREAM("Found topo plan of size " << plan->pathElements.size() << " and costs " << localMap.getEndNode()->predMap.begin()->second.costFromStart);
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

void DijkstraTopoPlanner::initLocalMap(TopoMap const *topoMap) {
  mainQueue.clear();
  regionCostsCount.clear();
  transitionCostsCount.clear();
  
  if(topoMap->getVersionCount() != localMap.getLastTopoMapVersion()) {
    MapTransformer::createLocalMap(*topoMap, localMap, nodePlanner, edgePlanner, _config.prePathDiscovery ? LocalNode::UNKNOWN : LocalNode::DISCOVERED, _config.useBackDijkstraHeuristic? LocalNode::infCosts : 0.0);
    ROS_INFO("Topo map changed, re-creating planning map");
  } else {
    // mark everything as discovered if no pre path discovery
    localMap.resetStartEndPaths(_config.prePathDiscovery? LocalNode::UNKNOWN : LocalNode::DISCOVERED, _config.useBackDijkstraHeuristic? LocalNode::infCosts : 0.0);
    ROS_INFO("Re-using planning map for Dijkstra");
  }
  
  localMap.setStartNode(start.node, _config.useBackDijkstraHeuristic? LocalNode::infCosts : 0);
  localMap.getStartNode()->predMap.emplace(std::piecewise_construct, std::make_tuple(0), std::make_tuple(start.localPos, 0, nullptr)); // first node has costs 0 and no pred edge
  localMap.setEndNode(end.node, _config.useBackDijkstraHeuristic? LocalNode::infCosts : 0);
  nodePlanner->convertGlobalPoseToApproachExitData(start, &localMap.getStartNode()->approachExitData);
  nodePlanner->convertGlobalPoseToApproachExitData(end, &localMap.getEndNode()->approachExitData);
  
  if(_config.prePathDiscovery) {
    bfsFromGoal();
  } else {
    localMap.getStartNode()->state = LocalNode::DISCOVERED;
    localMap.getEndNode()->state = LocalNode::DISCOVERED;
  }
  
  localMap.saveToDot("before.dot");
  
  if(localMap.getStartNode()->state == LocalNode::DISCOVERED) {
    if(_config.useBackDijkstraHeuristic) {
      doBackDijkstraHeuristic();
    }
    
    mainQueue.insert(localMap.getStartNode());
  }
}

bool DijkstraTopoPlanner::updateCostAndPred(DijkstraTopoPlanner::LocalEdge *v, bool evaluateReal) {
  LocalPositionConstPtr pose;
  
  if(!evaluateReal && v->costsState == LocalEdge::UNKNOWN) {
    calcEdgeHeuristics(v);
  } else if(evaluateReal) {
    if (v->isTransitionEdge()) { // transition edge -> recompute exit pose and costs based on current approach pose
      std::tie(v->cost, pose) = edgePlanner->computeTransitionCost(v->topoEdge, *v->a->predMap.begin()->second.localPosition,
                                                                   &v->pathData);
      if (!pose) {
        ROS_WARN_STREAM("DijkstraTopoPlanner: exit pose for edge '" << v->topoEdge->getName()
                                                                    << "' was computed to nullptr, ignoring");
        return false;
      }
      
      transitionCostsCount[v->topoEdge->getTransitionType()].second++;
    } else {
      std::tie(v->cost, pose) = nodePlanner->computeCostsOnRegion(v->b->region,
                                                                  *v->a->predMap.begin()->second.localPosition,
                                                                  v->b->approachExitData, &v->pathData);
      if (!pose) {
        ROS_WARN_STREAM("DijkstraTopoPlanner: computeCostsOnRegion '" << v->b->region->getName()
                                                                      << "' returned nullptr pose, ignoring");
        return false;
      }
      
      regionCostsCount[v->topoNode->getRegionType()].second++; // TODO: really ignore failed calls?
    }
    
    v->costsState = LocalEdge::REAL;
  }
  
  double alternateCost = v->a->minCostsFromStart() + v->cost;
  v->b->predMap.emplace(std::piecewise_construct, std::make_tuple(alternateCost), std::make_tuple(pose, alternateCost, v));
  std::cout << "updating edge " << v << " " <<  v->cost << std::endl;
  return true;
}

void DijkstraTopoPlanner::calcEdgeHeuristics(DijkstraTopoPlanner::LocalEdge *e) {
  if(e->isSameRegion()) {
    e->cost = nodePlanner->getHeuristics(e->topoNode, e->a->approachExitData, e->b->approachExitData);
    regionCostsCount[e->topoNode->getRegionType()].first++;
  } else {
    e->cost = edgePlanner->getHeuristics(e->topoEdge);
    transitionCostsCount[e->topoEdge->getTransitionType()].first++;
  }
  e->costsState = LocalEdge::HEURISTIC;
}

void DijkstraTopoPlanner::doBackDijkstraHeuristic() {
  std::set<LocalNode*> q;
  
  localMap.getEndNode()->hVal = 0;
  q.insert(localMap.getEndNode());
  while(!q.empty()) {
    // remove lowest node
    auto iter = std::min_element(q.begin(), q.end(), [](LocalNode* a, LocalNode *b) {return a->hVal < b->hVal; });
    LocalNode* n = *iter;
    n->backDijkstraVisited = true;
    q.erase(iter);
    
    // update succs
    for(LocalEdge *e: n->predEdges) {
      if(e->a->state == LocalNode::DISCOVERED && !e->a->backDijkstraVisited) {
        if(q.find(e->a) == q.end()) {
          q.insert(e->a);
        }
        
        if(e->costsState == LocalEdge::UNKNOWN)
          calcEdgeHeuristics(e);
        
        double altDist = n->hVal + e->cost;
        if(altDist < e->a->hVal)
          e->a->hVal = altDist;
      }
    }
  }
  
  ROS_INFO_STREAM("hStart = " << localMap.getStartNode()->hVal);
}

void DijkstraTopoPlanner::bfsFromGoal() {
  std::list<LocalNode*> queue = {localMap.getEndNode()};
  while(!queue.empty()) {
    LocalNode* n = queue.front();
    queue.pop_front();
    
    n->state = LocalNode::DISCOVERED; // DISCOVERED corresponds to visited in bfs
    for(LocalEdge* predE: n->predEdges) {
      LocalNode* pred = predE->a;
      if(pred->state == LocalNode::UNKNOWN) { // UKNOWN means whether visited nor in queue
        pred->state = LocalNode::DISCOVERING; // DISCOVERING means in queue but not visited
        queue.push_back(pred);
      }
    }
  }
}

DijkstraTopoPlanner::LocalNode *DijkstraTopoPlanner::getAndRemoveLowestCostNode() {
  auto iter = std::min_element(mainQueue.begin(), mainQueue.end(), [](LocalNode* a, LocalNode *b) {return a->getRanking() < b->getRanking(); });
  LocalNode* n = *iter;
  mainQueue.erase(iter);
  return n;
}
  
}; // namespace topo_nav
