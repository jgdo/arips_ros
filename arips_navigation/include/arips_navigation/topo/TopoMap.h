#pragma once

#include <map>
#include <memory>
#include <set>
#include <string>

#include <arips_navigation/path_planning/PlanningMath.h>

class TopoMap;
class TopoDoor;

/**
 * @brief Topological Node
 *
 * Represents a region inside the topological map with an homogeneous motion state space.
 */
class TopoRoom {
    friend class TopoMap;
    friend class TopoDoor;

public:
    TopoRoom(TopoMap* map, std::string name);

    /**
     * @return topological map this node belongs to
     */
    inline TopoMap* getParentMap() { return _parentMap; }

    /**
     * @return topological map this node belongs to
     */
    inline const TopoMap* getParentMap() const { return _parentMap; }

    /**
     * Add an outgoing edge to this node. For internal usage.
     *
     * @throws exception if edge->src is not this
     */
    void addOutEdge(const TopoDoor* edge);

    /**
     * Add an outgoing edge to this node. For internal usage.
     *
     * @throws exception if edge->dst is not this
     */
    void addInEdge(const TopoDoor* edge);

    const std::set<const TopoDoor*>& getOutEdges() const { return _outEdges; }

    const std::set<const TopoDoor*>& getInEdges() const { return _inEdges; }

    /**
     * @return Unique name which identifies this node in the containing map.
     */
    inline std::string const& getName() const { return _name; }

protected:
    TopoMap* _parentMap;

    std::string _name; // unique name

    std::set<const TopoDoor*> _outEdges;
    std::set<const TopoDoor*> _inEdges;
};

/**
 * @brief Topological Edge. Connects two nodes.
 *
 * Represents a transition connecting two motion state spaces (topological nodes).
 */
class TopoDoor {
    friend class TopoMap;
    friend class TopoRoom;

public:
    TopoDoor(TopoMap* map, TopoRoom* src, TopoRoom* dst, std::string name);

    inline TopoMap* getParentMap() { return _parentMap; }

    inline const TopoMap* getParentMap() const { return _parentMap; }

    inline const std::string& getName() const { return _name; }

    inline TopoRoom* getSource() { return _src; }

    inline const TopoRoom* getSource() const { return _src; }

    inline const TopoRoom* getDestination() const { return _dst; }

    /**
     * Assign a new source to this edge
     * @param newSrc must not be null
     */
    void setSrc(TopoRoom* newSrc);

    /**
     * Assign a new destination to this edge
     * @param newDst must not be null
     */
    void setDst(TopoRoom* newDst);

     Pose2D approachPose() const {
        return _approachPose;
    }

    Pose2D exitPose() const {
        return _exitPose;
    }

// TODO protected:
    TopoMap* _parentMap;
    TopoRoom *_src, *_dst;
    std::string _name; // unique name

    Pose2D _approachPose;
    Pose2D _exitPose;
    TopoDoor* _otherDoorEdge = nullptr;
    // TODO additional edge data
};

/**
 * @brief Topological Map. Contains Node and Edge definitions.
 *
 * Hold the topological graph. User data might be attached to the whole map and individual nodes and
 * edges.
 */
class TopoMap {
    friend class TopoRoom;
    friend class TopoDoor;

public:
    using Ptr = std::shared_ptr<TopoMap>;

    TopoRoom* addNode(std::string name);

    TopoRoom* getNode(std::string name);

    const TopoRoom* getNode(std::string name) const;

    inline size_t getNumNodes() const { return _nodes.size(); }

    TopoDoor* addEdge(TopoRoom* srcNode, TopoRoom* dstNode, std::string name);

    TopoDoor* getEdge(std::string name);

    inline size_t getNumEdges() const { return _edges.size(); }

    template <class T> inline void foreachNode(const T& func) {
        for (auto& entry : _nodes) {
            func(entry.second.get());
        }
    }

    template <class T> inline void foreachNode(const T& func) const {
        for (auto& entry : _nodes) {
            func(entry.second.get());
        }
    }

    /**
     * Iterate through all nodes until a node is found for which func(node) returns something which
     * is evaluated as true
     *
     * Example 1 - searching for a node named "mynode":
     *
     * Node* node = map.foreachNodeFind([](Node* node) -> Node* { return node->getName() ==
     * "mynode"? node : nullptr });
     *
     *
     * Example 2 - checking if a node named "mynode" exists:
     *
     * Node* node = map.foreachNodeFind([](Node* node) -> bool { return node->getName() == "mynode";
     * });
     *
     * T: function returning a value which can be interpreted as bool
     */
    template <class T> inline auto foreachNodeFind(const T& func) -> decltype(func(nullptr)) {
        for (auto& entry : _nodes) {
            auto res = func(entry.second.get());
            if (res)
                return res;
        }

        return decltype(func(nullptr))();
    }

    template <class T> inline auto foreachNodeFind(const T& func) const -> decltype(func(nullptr)) {
        for (const auto& entry : _nodes) {
            auto res = func(entry.second.get());
            if (res)
                return res;
        }

        return decltype(func(nullptr))();
    }

    template <class T> inline void foreachEdge(const T& func) {
        for (auto& entry : _edges) {
            func(entry.second.get());
        }
    }

    template <class T> inline void foreachEdge(const T& func) const {
        for (auto& entry : _edges) {
            func(entry.second.get());
        }
    }

    void removeNode(TopoRoom* node);

    void removeEdge(TopoDoor* edge);

    void exportToDot(std::string const& filename) const;

    inline unsigned long long getVersionCount() const { return versionCount; }

protected:
    std::map<std::string, std::shared_ptr<TopoRoom>> _nodes; // name -> node
    std::map<std::string, std::shared_ptr<TopoDoor>> _edges; // name -> edge

    unsigned long long versionCount = 0;

    inline void changed() { versionCount++; }

}; // class TopoMap

struct TopoPose2D {
    const TopoRoom* room;
    Pose2D pose;

    TopoPose2D(const TopoRoom* room, Pose2D pose):
        room{room}, pose{pose} {}
};

using TopoMapPtr = TopoMap::Ptr;
