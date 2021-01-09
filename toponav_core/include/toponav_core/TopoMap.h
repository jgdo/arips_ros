#pragma once

#include <string>
#include <memory>
#include <map>
#include <set>

#include "utils/DataMap.h"

namespace toponav_core {

/**
 * @brief Topological Map. Contains Node and Edge definitions.
 *
 * Hold the topological graph. User data might be attached to the whole map and individual nodes and edges.
 */
class TopoMap {
public:    
	class Edge;
	
	typedef std::shared_ptr<TopoMap> Ptr;
	
	/**
	 * For counting strings
	 */
	typedef std::map<std::string, size_t> CountMap;

	/**
	 * @brief Topological Node
	 *
	 * Represents a region inside the topological map with an homogeneous motion state space.
	 */
	class Node {
    friend class TopoMap;
	public:
		Node(TopoMap *map, std::string name, std::string type);
		
		/**
		 * @return topological map this node belongs to
		 */
		inline TopoMap* getParentMap() {
			return _parentMap;
		}
		
		/**
		 * @return topological map this node belongs to
		 */
		inline const TopoMap* getParentMap() const {
			return _parentMap;
		}
		
		/**
		 * Add an outgoing edge to this node. For internal usage.
		 *
		 * @throws exception if edge->src is not this
		 */
		void addOutEdge(const Edge *edge);
    
    /**
     * Add an outgoing edge to this node. For internal usage.
     *
		 * @throws exception if edge->dst is not this
		 */
    void addInEdge(const Edge *edge);

		inline const boost::any& nodeData() const {
			return _nodeData;
		}
		
		/**
		 * @return User data of this node.
		 */
		inline boost::any& nodeData() {
			return _nodeData;
		}
		
		inline const DataMap& propertyMap() const {
			return _properties;
		}
		
		/**
		 * @return User data of this node.
		 */
		inline DataMap& propertyMap() {
			return _properties;
		}

		/**
		 * @return Region type of this node.
		 */
		inline const std::string& getRegionType() const {
			return _regionType;
		}
	
		const std::set<const Edge *>& getOutEdges() const {
			return _outEdges;
		}
    
    const std::set<const Edge *>& getInEdges() const {
      return _inEdges;
    }
		
		/**
		 * @return Unique name which identifies this node in the containing map.
		 */
    inline std::string const& getName() const {
			return _name;
		}

	protected:
		TopoMap* _parentMap;
		
		std::string _name; // unique name
		std::string _regionType; // e.g. 'flat', 'stairs', 'ramp'

		std::set<const Edge *> _outEdges;
		std::set<const Edge *> _inEdges;

		boost::any _nodeData; // used by topo plug-ins
		DataMap _properties; // generic
	};

	/**
	 * @brief Topological Edge. Connects two nodes.
	 *
	 * Represents a transition connecting two motion state spaces (topological nodes).
	 */
	class Edge {
    friend class TopoMap;
	public:
		Edge(TopoMap* map, Node *src, Node *dst, std::string name, std::string transitionType);
		
		inline TopoMap* getParentMap() {
			return _parentMap;
		}
		
		inline const TopoMap* getParentMap() const {
			return _parentMap;
		}

		inline const std::string& getName() const {
			return _name;
		}

		inline Node* getSource() {
			return _src;
		}
		
		inline const Node* getSource() const {
			return _src;
		}

		inline const Node* getDestination() const {
			return _dst;
		}

		inline const boost::any& approachData() const {
			return _specificSrcEdgeData;
		}

		inline boost::any& approachData() {
			return _specificSrcEdgeData;
		}

		inline const boost::any& exitData() const {
			return _specificDstEdgeData;
		}

		inline boost::any& exitData() {
			return _specificDstEdgeData;
		}
		
		inline const boost::any& edgeData() const {
			return _specificEdgeData;
		}
		
		inline boost::any& edgeData() {
			return _specificEdgeData;
		}
		
		inline const DataMap& propertyMap() const {
			return _properties;
		}
		
		inline DataMap& propertyMap() {
			return _properties;
		}
		
		inline const std::string& getTransitionType() const {
			return _transitionType;
		}

	protected:
		TopoMap* _parentMap;
		Node *_src, *_dst;
		std::string _name; // unique name
		std::string _transitionType; // e.g. 'door', 'step'

		// TODO TopoEdge* reverse = nullptr;

		boost::any _specificSrcEdgeData; // used by topo plug-ins
		boost::any _specificDstEdgeData; // used by topo plug-ins
		
		boost::any _specificEdgeData; // for plugins
		
		DataMap _properties;
	};
	
	struct MapMetaData {
		std::string fileName;
	};

	static const std::string ENTRY_DOT_COLOR;
	
	TopoMap();

	Node * addNode(std::string name, std::string type);

	Node * getNode(std::string name);
	
	const Node * getNode(std::string name) const;
	
	inline size_t getNumNodes() const {
		return _nodes.size();
	}

	Edge * addEdge(Node *srcNode, Node *dstNode, std::string name, std::string type);

	Edge * getEdge(std::string name);
	
	inline size_t getNumEdges() const {
		return _edges.size();
	}
	
	inline DataMap& propertyMap() {
		return _mapData;
	}
	
	inline const DataMap& propertyMap() const {
		return _mapData;
	}
    
    template<class T>
    inline void foreachNode(const T& func) {
        for(auto& entry: _nodes) {
            func(entry.second.get());
        }
    }
	
	template<class T>
	inline void foreachNode(const T& func) const {
		for(auto& entry: _nodes) {
			func(entry.second.get());
		}
	}

    /**
     * Iterate through all nodes until a node is found for which func(node) returns something which is evaluated as true
     * 
     * Example 1 - searching for a node named "mynode":
     * 
     * Node* node = map.foreachNodeFind([](Node* node) -> Node* { return node->getName() == "mynode"? node : nullptr });
     * 
     * 
     * Example 2 - checking if a node named "mynode" exists:
     * 
     * Node* node = map.foreachNodeFind([](Node* node) -> bool { return node->getName() == "mynode"; });
     * 
     * T: function returning a value which can be interpreted as bool
     */
    template<class T>
    inline auto foreachNodeFind(const T& func) -> decltype(func(nullptr)) {
        for(auto& entry: _nodes) {
            auto res = func(entry.second.get());
            if(res)
                return res;
        }
        
        return decltype(func(nullptr))();
    }
	
	template<class T>
	inline auto foreachNodeFind(const T& func) const -> decltype(func(nullptr)) {
		for(const auto & entry: _nodes) {
			auto res = func(entry.second.get());
			if(res)
				return res;
		}
		
		return decltype(func(nullptr))();
	}
	
	template<class T>
	inline void foreachEdge(const T& func) {
		for(auto& entry: _edges) {
			func(entry.second.get());
		}
	}
	
	template<class T>
	inline void foreachEdge(const T& func) const {
		for(auto& entry: _edges) {
			func(entry.second.get());
		}
	}
	
	inline MapMetaData const& getMetaData() const {
		return _metaData;
	}
	
	inline MapMetaData & setMetaData() {
		return _metaData;
	}
  
  inline unsigned long long getVersionCount() const {
    return _versionCount;
  }
  
  void removeNode(Node* node);
  
  void removeEdge(Edge* edge);
	
	void exportToDot(std::string const &filename, std::map<std::string, std::string> const &node_colors = {},
                     std::map<std::string, std::string> const &edge_colors = {}) const;
	
	inline const CountMap& getCurrentNodeTypesCount() const {
		return node_types_;
	}
	
	inline const CountMap& getCurrentEdgeTypesCount() const {
		return edge_types_;
	}
    
protected:
	std::map<std::string, std::shared_ptr<Node>> _nodes; // name -> node
	std::map<std::string, std::shared_ptr<Edge>> _edges; // name -> edge
	
	// counts each node/edge type
	CountMap node_types_, edge_types_;
	
	DataMap _mapData;
	
	MapMetaData _metaData;
  
  unsigned long long _versionCount = 0;
	
	template<class ExT, class... Args>
	void static checkTrue(bool value, Args &&... args) {
		if (!value)
			throw ExT(std::forward<Args>(args)...);
	}

	inline static void checkTrue(bool value, std::string msg) {
		checkTrue<std::runtime_error>(value, msg);
	}
  
  inline void changed() {
    _versionCount++;
  }
}; // class TopoMap

typedef TopoMap::Ptr TopoMapPtr;

/**
 * Represents a position inside a region.
 */
class LocalPosition {
public:
	typedef std::shared_ptr<LocalPosition> Ptr;
	typedef std::shared_ptr<const LocalPosition> ConstPtr;

	inline virtual ~LocalPosition() {}
};

typedef LocalPosition::Ptr LocalPositionPtr;
typedef LocalPosition::ConstPtr LocalPositionConstPtr;

/**
 * Represents a full position containing the region and the local position inside this region
 */
class GlobalPosition {
public:
	typedef std::shared_ptr<GlobalPosition> Ptr;
	typedef std::shared_ptr<const GlobalPosition> ConstPtr;

	const TopoMap::Node* node = nullptr;
	LocalPositionConstPtr localPos;

	inline GlobalPosition() = default;

	inline GlobalPosition(const TopoMap::Node* region, const LocalPositionConstPtr& pos): node(region), localPos(pos) {
	}

	inline GlobalPosition(const GlobalPosition& other): node(other.node), localPos(other.localPos) {
	}
	
	inline operator bool() const {
        return node && localPos;
      }
//
//	inline GlobalPosition(GlobalPosition&& other): node(std::move(other.node)), localPos(std::move(other.localPos)) {
//	}
};

typedef GlobalPosition::Ptr GlobalPositionPtr;
typedef GlobalPosition::ConstPtr GlobalPositionConstPtr;

struct AbstractApproachExitData: public boost::any {
	using boost::any::any;
	using boost::any::operator=;
};


struct AbstractPathData: public boost::any {
	using boost::any::any;
	using boost::any::operator=;
};

}; // namespace topo_nav
