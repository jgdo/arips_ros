#include "../include/toponav_core/TopoMap.h"
#include <iostream>
#include <fstream>

namespace toponav_core {

const std::string TopoMap::ENTRY_DOT_COLOR = "_dot_color";

TopoMap::TopoMap() {
}

TopoMap::Node *TopoMap::addNode(std::string name, std::string type) {
	checkTrue(_nodes.find(name) == _nodes.end(), "TopoMap::addNode(): Node \"" + name + "\" already exists");

	auto node = std::make_shared<Node>(this, name, type);
	_nodes.emplace(name, node);
	node_types_[node->getRegionType()]++;
	changed();
	return node.get();
}

TopoMap::Edge *TopoMap::addEdge(TopoMap::Node *srcNode, TopoMap::Node *dstNode, std::string name, std::string type) {
	checkTrue(_edges.find(name) == _edges.end(), "TopoMap::addEdge(): Edge \"" + name + "\" already exists");
	checkTrue(srcNode, "TopoMap::addEdge(): srcNode is null");
	checkTrue(dstNode, "TopoMap::addEdge(): dstNode is null");

	auto edge = std::make_shared<Edge>(this, srcNode, dstNode, name, type);
	_edges.emplace(name, edge);
	edge_types_[edge->getTransitionType()]++;
  changed();
	return edge.get();
}

TopoMap::Node *TopoMap::getNode(std::string name) {
	auto iter =  _nodes.find(name);
  if(iter != _nodes.end())
    return iter->second.get();
  else
    return nullptr;
}

const TopoMap::Node *TopoMap::getNode(std::string name) const {
	auto iter =  _nodes.find(name);
	if(iter != _nodes.end())
		return iter->second.get();
	else
		return nullptr;
}

TopoMap::Edge *TopoMap::getEdge(std::string name) {
	auto iter = _edges.find(name);
	return (iter != _edges.end()) ? iter->second.get() : nullptr;
}

void TopoMap::exportToDot(std::string const &filename, std::map<std::string, std::string> const &node_colors,
													std::map<std::string, std::string> const &edge_colors) const {
	std::ofstream fout(filename);
	
	fout << "digraph TopoGraph {" << std::endl;
	
	auto findColor = [](DataMap const& data, std::string const& key, std::map<std::string, std::string> const & color_map, const std::string& default_color) -> std::string {
		// try to find color in node data, can be stored under ENTRY_DOT_COLOR as const char* or std::string
		auto& entry = data.getOrEmpty(ENTRY_DOT_COLOR);
		if(auto char_pp = boost::any_cast<const char*>(&entry))
			return *char_pp;
		else if(auto str_p = boost::any_cast<std::string>(&entry))
			return *str_p;
		else { // not found in node data, look in node_colors map
			auto iter = color_map.find(key);
			if(iter != color_map.end()) {
				return iter->second;
			}
		};
		
		return default_color;
	};
	
	std::string default_node_color = "white";
	for(auto const& n: _nodes) {
		std::string color = findColor(n.second->propertyMap(), n.second->getRegionType(), node_colors, default_node_color);
		fout << "\"" << n.second->getName() << "\" [fillcolor=\"" << color << "\", style=filled, shape=circle]" << std::endl;
	}
	
	std::string default_edge_color = "black";
	for(auto const& e: _edges) {
		std::string color = findColor(e.second->propertyMap(), e.second->getTransitionType(), edge_colors, default_edge_color);
		fout << "\"" << e.second->getSource()->getName() << "\" -> \"" << e.second->getDestination()->getName() << "\" [color=\"" << color  <<"\"]" << std::endl;
	}
  
	fout << "}" << std::endl;
}

void TopoMap::removeNode(TopoMap::Node *node) {
	CountMap::iterator iter = node_types_.find(node->getRegionType());
	if(iter->second > 1)
		iter->second--;
	else
		node_types_.erase(iter);
	
	// delete in succ nodes
	for(auto& e: node->_outEdges) {
		if(e->_dst != node)
			e->_dst->_inEdges.erase(e);
		
		this->_edges.erase(e->getName());
	}
	node->_outEdges.clear();
	
	// delete in pred nodes
	for(auto& e: node->_inEdges) {
		if(e->_src != node)
			e->_src->_outEdges.erase(e);
		
		this->_edges.erase(e->getName());
	}
	node->_inEdges.clear();
	_nodes.erase(node->getName());  // now node is a dangling pointer!
	node = nullptr;
	
	changed();
}

void TopoMap::removeEdge(TopoMap::Edge *edge) {
	CountMap::iterator iter = edge_types_.find(edge->getTransitionType());
	if(iter->second > 1)
		iter->second--;
	else
		edge_types_.erase(iter);
	
  edge->_src->_outEdges.erase(edge);
  edge->_dst->_inEdges.erase(edge);
  this->_edges.erase(edge->getName()); // now edge is a dangling pointer!
	edge = nullptr;
	changed();
}

TopoMap::Node::Node(TopoMap *map, std::string name, std::string type) : _parentMap(map), _name(name), _regionType(type) {
}

void TopoMap::Node::addOutEdge(const TopoMap::Edge *edge) {
	checkTrue(edge->getSource() == this, "TopoMap::Node::addOutEdge(): edge->src is not this");

	this->_outEdges.insert(edge);
}

void TopoMap::Node::addInEdge(const TopoMap::Edge *edge) {
  checkTrue(edge->getDestination() == this, "TopoMap::Node::addInEdge(): edge->dst is not this");
  
  this->_inEdges.insert(edge);
}

TopoMap::Edge::Edge(TopoMap* map, TopoMap::Node *src, TopoMap::Node *dst, std::string name, std::string transitionType): _parentMap(map), _src(src), _dst(dst), _name(name), _transitionType(transitionType) {
	src->addOutEdge(this);
  dst->addInEdge(this);
}

/*
 *
 */
void TopoMap::Edge::setSrc(Node* newSrc) {
        checkTrue(newSrc, "TopoMap::Edge::setSrc(): newSrc is null");
    if(_src != newSrc) {
        _src->_outEdges.erase(this);
        _src = newSrc;
        _src->addOutEdge(this);

        ROS_INFO("Updated edge %s src to %s", getName().c_str(), _src->getName().c_str());
    }

    _parentMap->changed();
}


void TopoMap::Edge::setDst(Node* newDst) {
    checkTrue(newDst, "TopoMap::Edge::setDst(): newDst is null");
    if(_dst != newDst) {
        _dst->_inEdges.erase(this);
        _dst = newDst;
        _dst->addInEdge(this);

        ROS_INFO("Updated edge %s src to %s", getName().c_str(), _dst->getName().c_str());
    }

    _parentMap->changed();
}
	
}; // namespace topo_nav
