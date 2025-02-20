#pragma once

#include <arips_navigation/topo/PlanningInterfaces.h>

#include <iostream>


class MapTransformer {
public:
  template<class PM, typename... Args>
  static void
  createLocalMap(TopoMap const &topoMap, PM &planningMap,
                  const DoorPlanningInterfacePtr &edgePlanner, typename PM::Node::State initialState, Args&&... args) {
    planningMap.clear();
  
    std::map<const TopoDoor*, std::pair<typename PM::Node*, typename PM::Node*>> approachExitPoses;
    
    // first collect edges. Can't write code inside lambda because of bug in g++ 4.8 concerning the capture of args... inside lambda
    // see https://gcc.gnu.org/bugzilla/show_bug.cgi?id=55914
    std::vector<const TopoDoor*> edges;
    edges.reserve(topoMap.getNumEdges());
    
    topoMap.foreachEdge([&](const TopoDoor* edge) {
      edges.push_back(edge);
    });
    
    for(auto edge: edges) {
      auto approach = planningMap.addLonelyNode(edge->getSource(), initialState, std::forward<Args>(args)...);
      approach->pose = edge->approachPose();
      auto exit = planningMap.addLonelyNode(edge->getDestination(), initialState, std::forward<Args>(args)...);
      exit->pose = edge->exitPose();
      approachExitPoses.emplace(edge, std::make_pair(approach, exit));
  
      // create approach->exit transition edge
      planningMap.addEdge(approach, exit, edge);
    }
  
    for(auto edge: edges) {
      auto exit = approachExitPoses.at(edge).second;
      auto region = edge->getDestination();
    
      for(TopoDoor const* outEdge: region->getOutEdges()) {
        if(!edgePlanner->areEdgesCoupled(edge, outEdge)) {
          auto approach = approachExitPoses.at(outEdge).first;
          // create exit->approach edge inside region
          planningMap.addEdge(exit, approach, region);
        }
      }
    };
  
    planningMap.resetApproachExitPosesMap(std::move(approachExitPoses), topoMap.getVersionCount());
    
    std::cout << "created planning map with " << planningMap.getNumNodes() << " nodes and " << planningMap.getNumEdges() << " edges" << std::endl;
  }

  
private:
  MapTransformer();
};
  
