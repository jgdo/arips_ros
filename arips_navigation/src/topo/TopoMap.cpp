#include <arips_navigation/topo/TopoMap.h>
#include <fstream>
#include <iostream>

#include <ros/console.h>

template <class ExT, class... Args> static void checkTrue(bool value, Args&&... args) {
    if (!value)
        throw ExT(std::forward<Args>(args)...);
}

static void checkTrue(bool value, std::string msg) { checkTrue<std::runtime_error>(value, msg); }

TopoRoom* TopoMap::addNode(std::string name) {
    checkTrue(_nodes.find(name) == _nodes.end(),
              "TopoMap::addNode(): Node \"" + name + "\" already exists");

    auto node = std::make_shared<TopoRoom>(this, name);
    _nodes.emplace(name, node);
    changed();
    return node.get();
}

TopoDoor* TopoMap::addEdge(TopoRoom* srcNode, TopoRoom* dstNode, std::string name) {
    checkTrue(_edges.find(name) == _edges.end(),
              "TopoMap::addEdge(): Edge \"" + name + "\" already exists");
    checkTrue(srcNode, "TopoMap::addEdge(): srcNode is null");
    checkTrue(dstNode, "TopoMap::addEdge(): dstNode is null");

    auto edge = std::make_shared<TopoDoor>(this, srcNode, dstNode, name);
    _edges.emplace(name, edge);
    changed();
    return edge.get();
}

TopoRoom* TopoMap::getNode(std::string name) {
    auto iter = _nodes.find(name);
    if (iter != _nodes.end())
        return iter->second.get();
    else
        return nullptr;
}

const TopoRoom* TopoMap::getNode(std::string name) const {
    auto iter = _nodes.find(name);
    if (iter != _nodes.end())
        return iter->second.get();
    else
        return nullptr;
}

TopoDoor* TopoMap::getEdge(std::string name) {
    auto iter = _edges.find(name);
    return (iter != _edges.end()) ? iter->second.get() : nullptr;
}

void TopoMap::exportToDot(std::string const& filename) const {

    static const auto node_color = "white";
    static const auto edge_color = "black";

    std::ofstream fout(filename);

    fout << "digraph TopoGraph {" << std::endl;

    for (auto const& n : _nodes) {
        fout << "\"" << n.second->getName() << "\" [fillcolor=\"" << node_color
             << "\", style=filled, shape=circle]" << std::endl;
    }

    for (auto const& e : _edges) {
        fout << "\"" << e.second->getSource()->getName() << "\" -> \""
             << e.second->getDestination()->getName() << "\" [color=\"" << edge_color << "\"]"
             << std::endl;
    }

    fout << "}" << std::endl;
}

void TopoMap::removeNode(TopoRoom* node) {
    // delete in succ nodes
    for (auto& e : node->_outEdges) {
        if (e->_dst != node)
            e->_dst->_inEdges.erase(e);

        this->_edges.erase(e->getName());
    }
    node->_outEdges.clear();

    // delete in pred nodes
    for (auto& e : node->_inEdges) {
        if (e->_src != node)
            e->_src->_outEdges.erase(e);

        this->_edges.erase(e->getName());
    }
    node->_inEdges.clear();
    _nodes.erase(node->getName()); // now node is a dangling pointer!
    node = nullptr;

    changed();
}

void TopoMap::removeEdge(TopoDoor* edge) {
    edge->_src->_outEdges.erase(edge);
    edge->_dst->_inEdges.erase(edge);
    this->_edges.erase(edge->getName()); // now edge is a dangling pointer!
    edge = nullptr;
    changed();
}

TopoRoom::TopoRoom(TopoMap* map, std::string name) : _parentMap(map), _name(name) {}

void TopoRoom::addOutEdge(const TopoDoor* edge) {
    checkTrue(edge->getSource() == this, "TopoRoom::addOutEdge(): edge->src is not this");

    this->_outEdges.insert(edge);
}

void TopoRoom::addInEdge(const TopoDoor* edge) {
    checkTrue(edge->getDestination() == this, "TopoRoom::addInEdge(): edge->dst is not this");

    this->_inEdges.insert(edge);
}

TopoDoor::TopoDoor(TopoMap* map, TopoRoom* src, TopoRoom* dst, std::string name)
    : _parentMap(map), _src(src), _dst(dst), _name(name) {
    src->addOutEdge(this);
    dst->addInEdge(this);
}

void TopoDoor::setSrc(TopoRoom* newSrc) {
    checkTrue(newSrc, "TopoDoor::setSrc(): newSrc is null");
    if (_src != newSrc) {
        _src->_outEdges.erase(this);
        _src = newSrc;
        _src->addOutEdge(this);

        ROS_INFO("Updated edge %s src to %s", getName().c_str(), _src->getName().c_str());
    }

    _parentMap->changed();
}

void TopoDoor::setDst(TopoRoom* newDst) {
    checkTrue(newDst, "TopoDoor::setDst(): newDst is null");
    if (_dst != newDst) {
        _dst->_inEdges.erase(this);
        _dst = newDst;
        _dst->addInEdge(this);

        ROS_INFO("Updated edge %s src to %s", getName().c_str(), _dst->getName().c_str());
    }

    _parentMap->changed();
}
