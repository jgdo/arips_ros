#pragma once

#include <toponav_core/interfaces/EdgePlanningInterface.h>
#include <toponav_core/interfaces/NodePlanningInterface.h>

#include <iostream>

namespace toponav_core {

class MapTransformer {
public:
  template<class PM, typename... Args>
  static void
  createLocalMap(TopoMap const &topoMap, PM &planningMap, NodePlanningInterfacePtr const &nodePlanner,
                  const EdgePlanningInterfacePtr &edgePlanner, typename PM::Node::State initialState, Args&&... args) {
    planningMap.clear();
  
    std::map<const TopoMap::Edge*, std::pair<typename PM::Node*, typename PM::Node*>> approachExitPoses;
    
    // first collect edges. Can't write code inside lambda because of bug in g++ 4.8 concerning the capture of args... inside lambda
    // see https://gcc.gnu.org/bugzilla/show_bug.cgi?id=55914
    std::vector<const TopoMap::Edge*> edges;
    edges.reserve(topoMap.getNumEdges());
    
    topoMap.foreachEdge([&](const TopoMap::Edge* edge) {
      edges.push_back(edge);
    });
    
    for(auto edge: edges) {
      auto approach = planningMap.addLonelyNode(edge->getSource(), initialState, std::forward<Args>(args)...);
      edgePlanner->initApproachData(edge, &approach->approachExitData);
      auto exit = planningMap.addLonelyNode(edge->getDestination(), initialState, std::forward<Args>(args)...);
      edgePlanner->initExitData(edge, &exit->approachExitData);
      approachExitPoses.emplace(edge, std::make_pair(approach, exit));
  
      // create approach->exit transition edge
      planningMap.addEdge(approach, exit, edge);
    }
  
    for(auto edge: edges) {
      auto exit = approachExitPoses.at(edge).second;
      auto region = edge->getDestination();
    
      for(TopoMap::Edge const* outEdge: region->getOutEdges()) {
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
  
} // namespace topo_nav
