#pragma once

#include <arips_navigation/topo/TopoMap.h>

#include <fstream>

template<class TDataContainerProvider>
class PlanningMap {
public:
  class Edge;
  
  struct Node: public TDataContainerProvider::template Node<Node, Edge>, public std::enable_shared_from_this<Node> {
    static constexpr double infCosts = std::numeric_limits<double>::max();
    
    const TopoRoom* region;
    
    std::set<Edge*> edges;
    std::set<Edge*> predEdges;
    
    enum State {
      UNKNOWN, DISCOVERING, DISCOVERED, DEADEND, VISITED
    } state = UNKNOWN;
  
    template<typename... Args>
    inline Node(const TopoRoom* region, Args&&... args):
        TDataContainerProvider::template Node<Node, Edge>::Node(std::forward<Args>(args)...),
        region(region) {
    }
  
    template<typename... Args>
    inline void reset(State newState, Args&&... args) {
      TDataContainerProvider::template Node<Node, Edge>::reset(std::forward<Args>(args)...);
      state = newState;
    }
    
    /**
     * Disconnects this nodes from other nodes, i.e. removes all edges to self from successors and predecessors
     */
    inline void disconnectNode(PlanningMap* map) {
      // delete in succ nodes
      for(auto& e: edges) {
        if(e->b != this) // check for edges to self
          e->b->predEdges.erase(e);
        
        map->edges.erase(e->shared_from_this());
      }
      edges.clear();
      
      // delete in pred nodes
      for(auto& e: predEdges) {
        if(e->a != this) // check for edges to self
          e->a->edges.erase(e);
  
        map->edges.erase(e->shared_from_this());
      }
      predEdges.clear();
    }
  };

// a->b
  struct Edge: public TDataContainerProvider::template Edge<Node, Edge>, public std::enable_shared_from_this<Edge> {
    Node* a, *b;
    
    double cost = Node::infCosts;
    
    enum CostsState {
      UNKNOWN, HEURISTIC, REAL
    } costsState = UNKNOWN;
    
    const TopoDoor* topoEdge = nullptr;
    const TopoRoom* topoNode = nullptr;
    
    inline Edge(Node* a, Node* b, const TopoDoor* edge): a(a), b(b), topoEdge(edge) {
      if(!a || !b)
        throw std::runtime_error("Created planning edge with a or b nullptr");
      
      a->edges.insert(this);
      b->predEdges.insert(this);
    }
    
    inline Edge(Node* a, Node* b, const TopoRoom* node): a(a), b(b), topoNode(node) {
      if(!a || !b)
        throw std::runtime_error("Created planning edge with a or b nullptr");
      
      a->edges.insert(this);
      b->predEdges.insert(this);
    }
    
    bool isSameRegion() const {
      return !!topoNode;
    }
    
    bool isTransitionEdge() const {
      return !isSameRegion();
    }
    
    inline void reset() {
      TDataContainerProvider::template Edge<Node, Edge>::reset();
      cost = Node::infCosts;
      costsState = UNKNOWN;
    }

      std::optional<std::vector<Pose2D>> pathPoints; // optionally store path
  };
  
  typedef std::shared_ptr<Node> NodePtr;
  typedef std::shared_ptr<Edge> EdgePtr;
  
  template<typename... Args>
  inline Node *addLonelyNode(const TopoRoom *region, typename Node::State initialState, Args&&... args) {
    NodePtr n = std::make_shared<Node>(region, std::forward<Args>(args)...);
    n->state = initialState;
    nodes.insert(n);
    return n.get();
  }
  
  // add edge and let v->a know about its neighbor
  template <typename... Args>
  inline Edge* addEdge(Args&&... args) {
    EdgePtr edge = std::make_shared<Edge>(std::forward<Args>(args)...);
    edges.insert(edge);
    return edge.get();
  }
  
  inline void clear() {
    approachExitPosesMap.clear();
    
    edges.clear();
    nodes.clear();
  
    startNode = nullptr;
    endNode = nullptr;
  };
  
  inline void resetApproachExitPosesMap(std::map<const TopoDoor*, std::pair<Node*, Node*>> && map, unsigned long long topoVersion) {
    approachExitPosesMap = std::move(map);
    lastTopoMapVersion = topoVersion;
  }
  
  template<typename... Args>
  inline void setStartNode(const TopoRoom* region, Args&&... args) {
    if(startNode) {
      throw std::runtime_error("Planning Map: cannot create start node since one already exists");
    }
  
    startNode = addLonelyNode(region, Node::UNKNOWN, std::forward<Args>(args)...);
    
    // create edges from start to all approach poses in this region
    for(const TopoDoor* edge: region->getOutEdges()) {
      addEdge(startNode, approachExitPosesMap.at(edge).first, region);
    }
  }
  
  template<typename... Args>
  inline void setEndNode(const TopoRoom* region, Args&&... args) {
    if(endNode) {
      throw std::runtime_error("Planning Map: cannot create end node since one already exists");
    }
  
    endNode = addLonelyNode(region, Node::UNKNOWN, std::forward<Args>(args)...);
    // create edges from all exit edges of this region to goal
    for(const TopoDoor* edge: region->getInEdges()) {
      auto& entry = approachExitPosesMap.at(edge);
      addEdge(entry.second, endNode, region);
    }
    
    if(startNode && startNode->region == region) {
      addEdge(startNode, endNode, region);
    }
  }
  
  template<typename... Args>
  inline void resetStartEndPaths(typename Node::State newNodeState, Args&&... args) {
    if(startNode) {
      startNode->disconnectNode(this);
      nodes.erase(startNode->shared_from_this());
      startNode = nullptr;
    }
    
    if(endNode) {
      endNode->disconnectNode(this);
      nodes.erase(endNode->shared_from_this());
      endNode = nullptr;
    }
  
    for(auto node: nodes) {
      node->reset(newNodeState, std::forward<Args>(args)...);
    }
    
    for(auto edge: edges) {
      edge->reset();
    }
  }
  
  inline Node* getStartNode() {
    return startNode;
  }
  
  inline Node* getEndNode() {
    return endNode;
  }
  
  inline unsigned long long getLastTopoMapVersion() const {
    return lastTopoMapVersion;
  }
  
  void saveToDot(std::string const& filename) const {
    std::ofstream fout(filename);
  
    fout << "digraph PlanningGraph {" << std::endl;
  
    std::map<typename Node::State , std::string> colors = { { Node::UNKNOWN ,"white" }, { Node::DISCOVERING, "yellow" }, { Node::DISCOVERED , "powderblue" }, { Node::DEADEND , "gray" }, { Node::VISITED , "palegreen"}};
  
    for(auto n: nodes) {
      if(n.get() == startNode)
        fout << "\"" << startNode << "\" [fillcolor=" << colors[startNode->state] << ", style=filled, shape=square]" << std::endl;
      else if(n.get() == endNode)
        fout << "\"" << endNode << "\" [fillcolor=" << colors[endNode->state] << ", style=filled, shape=star]" << std::endl;
      else
        fout << "\"" << n.get() << "\" [fillcolor=" << colors[n->state] << ", style=filled]" << std::endl;
    
    
    }
    
    for(auto e: edges) {
      fout << "\"" << e->a << "\" -> \"" << e->b << "\"[label=\"" << e.get() << "\"]" << std::endl;
    }

    fout << "}" << std::endl;
  }
  
  const std::pair<Node*, Node*>& getApproachExitPose(const TopoDoor* e) const {
    return approachExitPosesMap.at(e);
  };
  
  inline size_t getNumNodes() const {
    return nodes.size();
  }
  
  inline size_t getNumEdges() const {
    return edges.size();
  }
  
protected:
  std::set<NodePtr> nodes;
  std::set<EdgePtr> edges;
  
  Node* startNode = nullptr, *endNode = nullptr;
  
  std::map<const TopoDoor*, std::pair<Node*, Node*>> approachExitPosesMap;
  unsigned long long lastTopoMapVersion = 0;
};
