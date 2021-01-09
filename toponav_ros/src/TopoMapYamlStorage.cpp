#include <ros/ros.h>

#include "toponav_ros/TopoMapYamlStorage.h"

#include <fstream>

#include <yaml-cpp/yaml.h>

namespace toponav_ros {

using namespace toponav_core;
using namespace toponav_core;

static inline YAML::Node nullToUndefined(const YAML::Node& node ){
	if(node.IsNull())
		return YAML::Node();
	else
		return node;
}

void TopoMapYamlStorage::parseMap(TopoMap &map, std::string filename, ModuleContainer& factory) {
	NodeStorageInterfacePtr nodeParser = factory.getCreateModule<NodeStorageInterface>();
	EdgeStorageInterfacePtr edgeParser = factory.getCreateModule<EdgeStorageInterface>();
  
  YAML::Node root = YAML::LoadFile(filename);
  
	YAML::Node const nodes = root["nodes"];
	if(!nodes)
		ROS_WARN_STREAM("Topomap yaml contains no node definitions!");
	
	YAML::Node const edges = root["edges"];
	if(!edges)
		ROS_WARN_STREAM("Topomap yaml contains no edges definitions!");
	
	YAML::Node const pluginData = root["parser_data"];
	nodeParser->beginParsing(&map, nullToUndefined(pluginData["node_parser"]));
	edgeParser->beginParsing(&map, nullToUndefined(pluginData["edge_parser"]));

	// first round: collect nodes
	for(size_t iN = 0; iN < nodes.size(); iN++) {
		YAML::Node const& nodeDef = nodes[iN];

		std::string nodeName = nodeDef["name"].as<std::string>();
		std::string regionType = nodeDef["type"].as<std::string>();

		TopoMap::Node* node = map.addNode(nodeName, regionType);

		YAML::Node const nodeData = nullToUndefined(nodeDef["node_data"]);
		nodeParser->parseNodeData(nodeData, node);
	}
	
	for(size_t iE = 0; iE < edges.size(); iE++) {
		YAML::Node const edgeDef = edges[iE];
		
		std::string edgeName = edgeDef["name"].as<std::string>();
		std::string sourceName = edgeDef["source"].as<std::string>();
		std::string targetName = edgeDef["target"].as<std::string>();
		std::string transitionType = edgeDef["type"].as<std::string>();
		
		TopoMap::Node* sourceNode = map.getNode(sourceName);
		TopoMap::Node* targetNode = map.getNode(targetName);
		
		TopoMap::Edge *edgeAB = map.addEdge(sourceNode, targetNode, edgeName, transitionType);
		
		// approach data was out data before, exit data was in data
		YAML::Node const approachDef = nullToUndefined(edgeDef["approach_data"]? edgeDef["approach_data"] : edgeDef["out_data"]);
		YAML::Node const exitDef = nullToUndefined(edgeDef["exit_data"]? edgeDef["exit_data"] : edgeDef["in_data"]);
		
		YAML::Node const &dataDef = nullToUndefined(edgeDef["edge_data"]);
		edgeParser->parseEdgeData(dataDef, edgeAB);
		
		edgeParser->parseEdgeOutData(approachDef, edgeAB);
		edgeParser->parseEdgeInData(exitDef, edgeAB);
	}
			
	nodeParser->endParsing();
	edgeParser->endParsing();
  
	ROS_INFO_STREAM("Loaded topo map with " << map.getNumNodes() << " nodes and " << map.getNumEdges() << " edges.");
	map.setMetaData().fileName = filename;
	
	// map.exportToDot("topo_graph.dot", , ); // FIXME remove
}

void TopoMapYamlStorage::saveMap(TopoMap* map, std::string filename, NodeStorageInterfacePtr const &nodeStore,
                                 EdgeStorageInterfacePtr const &edgeStore) {
  ROS_INFO_STREAM("Saving topo map as " << filename);
  
  try {
    YAML::Node root;
    
    // save global map data
    YAML::Node parser_data;
    parser_data["node_parser"] = nodeStore->beginSaving(map);
    parser_data["edge_parser"] = edgeStore->beginSaving(map);
    root["parser_data"] = parser_data;
    
    // save nodes
    YAML::Node nodes(YAML::NodeType::Sequence);
    map->foreachNode([&](const TopoMap::Node *topoNode) {
      YAML::Node yamlNode;
      yamlNode["name"] = topoNode->getName();
      yamlNode["type"] = topoNode->getRegionType();
      yamlNode["node_data"] = nodeStore->saveNodeData(topoNode);
      nodes.push_back(yamlNode);
    });
    root["nodes"] = nodes;
    
    // save edges
    YAML::Node edges(YAML::NodeType::Sequence);
    map->foreachEdge([&](const TopoMap::Edge *topoEdge) {
			if(edgeStore->isEdgeSaveable(topoEdge)) {
				YAML::Node yamlEdge;
				yamlEdge["name"] = topoEdge->getName();
				yamlEdge["type"] = topoEdge->getTransitionType();
				yamlEdge["source"] = topoEdge->getSource()->getName();
				yamlEdge["target"] = topoEdge->getDestination()->getName();
				yamlEdge["approach_data"] = edgeStore->saveEdgeOutData(topoEdge);
				yamlEdge["exit_data"] = edgeStore->saveEdgeInData(topoEdge);
				yamlEdge["edge_data"] = edgeStore->saveEdgeData(topoEdge);
				edges.push_back(yamlEdge);
			}
    });
    root["edges"] = edges;
    
    edgeStore->endSaving();
    nodeStore->endSaving();
    
    std::ofstream fout(filename);
    fout << root;
    fout.close();
    
    map->setMetaData().fileName = filename;
    
    ROS_INFO_STREAM("Topo map saved");
  } catch (std::exception const& ex) {
    ROS_ERROR_STREAM("Failed to saved topo map: " << ex.what());
  }
}
  
  
} // namespace topo_nav




