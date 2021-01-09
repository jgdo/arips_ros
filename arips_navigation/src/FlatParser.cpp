#include <arips_navigation/FlatParser.h>

#include <arips_navigation/FlatGroundModule.h>
#include <toponav_ros/TopoMapYamlStorage.h>
#include <toponav_ros/PlanningContext.h>

#include <arips_navigation/utils/ApproachLineArea.h>

#include <arips_navigation/CostsPlanners.h>
#include <arips_navigation/utils/FixedPosition.h>
#include <toponav_ros/utils/YamlTools.h>

namespace toponav_ros {

using namespace toponav_core;

GEN_COSTS_READER_WRITER(FlatModuleCostsParser, BaseTraits<FlatGroundModule>::MapData,
      {"time_factor", &BaseTraits<FlatGroundModule>::MapData::time_factor},
      {"energy_factor", &BaseTraits<FlatGroundModule>::MapData::energy_factor},
      {"distance_factor", &BaseTraits<FlatGroundModule>::MapData::distance_factor});

void FlatParser::beginParsing(TopoMap *map, YAML::Node const &parserConfig) {
  costmapPlannerMap.clear();
  currentMap = map;
  
  FlatModuleCostsParser parser;
  parser.parse(&costmapPlannerMap, parserConfig["default_costs"], FlatGroundModule::className, "default costs");
  
  const YAML::Node mapsNode = parserConfig["layers"]? parserConfig["layers"] : parserConfig["maps"];
  for(auto& entry: mapsNode) {
    std::string costmapName = entry.first.as<std::string>();
    std::string plannerType = entry.second["planner"].as<std::string>();
    if(plannerType.empty())
      plannerType = "GridMapCostsPlanner";
  
    FlatGroundModule::CostsPlannerPtr planner;

#if 0
    if(plannerType == "GridMapCostsPlanner") {
      auto gridMapPlanner = std::make_shared<GridMapCostsPlanner>(costmapName + "_planner", costmapName);
      planner = gridMapPlanner;
    
      // FIXME: big hack for costmap init waiting
      auto last = ros::Time::now();
      while(gridMapPlanner->getMap().getLayers().empty()) {
        if((ros::Time::now() - last).toSec() > 5.0) {
          ROS_WARN_STREAM("Topo Planner: still waiting for map named '" << costmapName << "'");
          last = ros::Time::now();
        }
      
        ros::spinOnce();
        ros::Duration(0.1).sleep();
      }
    
    } else
#endif
    if(plannerType == "NavfnCostsPlanner") {
      std::shared_ptr<costmap_2d::Costmap2DROS> costmap = std::make_shared<costmap_2d::Costmap2DROS>("costmaps/" + costmapName, *_context.tfBuffer);
      costmap->start();
    
      std::shared_ptr<navfn::NavfnROS> navfn = std::make_shared<navfn::NavfnROS>(costmapName + "_planner", costmap.get());
    
      planner = std::make_shared<NavfnCostsPlanner>(costmap, navfn, costmapName);
    
      // FIXME: big hack for costmap init waiting
      auto last = ros::Time::now();
      while(!costmap->getCostmap() || costmap->getCostmap()->getSizeInCellsX() == 0) {
        if((ros::Time::now() - last).toSec() > 5.0) {
          ROS_WARN_STREAM("Topo Planner: still waiting for map named '" << costmapName << "'");
          last = ros::Time::now();
        }
      
        ros::spinOnce();
        ros::Duration(0.1).sleep();
      }
    } else {
      ROS_ERROR_STREAM("Flat map level '" << costmapName << "' specifies a UNKNOWN planner type '" << plannerType << "'. Ingoring map level.");
      continue;
    }
    
    FlatGroundModule::NodeMatrixPtr nodeMatrix = std::make_shared<FlatGroundModule::NodeMatrix>(0,0);
    const auto costmap = planner->getMap().getCostmap();
    *nodeMatrix = FlatGroundModule::NodeMatrix::Constant(costmap->getSizeInCellsX(), costmap->getSizeInCellsY(), nullptr);
    costmapPlannerMap.emplace(costmapName, FlatGroundModule::MapPlanningDataEntry {planner, nodeMatrix, _context.tfBuffer});
  }
}

void FlatParser::endParsing() {
  FlatGroundModule::setMapData(currentMap, costmapPlannerMap);
  
	costmapPlannerMap.clear();
}

void FlatParser::parseNodeData(YAML::Node const &config, TopoMap::Node *node) {
  
	std::string costmapName = (config["layer"]? config["layer"] : config["map"]).as<std::string>(); // TODO make generic
	int nodeX = config["x"].as<int>();
	int nodeY = config["y"].as<int>();
	double height = config["height"].as<double>();
  
  std::string plannerType = config["planner"].as<std::string>("GridMapCostsPlanner");
  
	auto costmapIter = costmapPlannerMap.find(costmapName);
	if(costmapIter == costmapPlannerMap.end()) {
		ROS_ERROR_STREAM("FlatParser: Could not find map layer entry '" << costmapName << "' for node '" << node->getName() << "'. Available:");
        for(const auto& e: costmapPlannerMap) {
            ROS_ERROR_STREAM("\t" << e.first);
        }
	}
  
  FlatGroundModule::initNodeData(node, costmapIter->second.planner, costmapIter->second.nodeMatrix,
                                   _context.tfBuffer, nodeX, nodeY, costmapName, height);
  
  // FIXME: what if layer not present?
  FlatGroundModule::regionGrow(node, costmapIter->second.planner->getMap().getCostmap(), nodeX, nodeY, costmapIter->second.nodeMatrix.get());
}

YAML::Node FlatParser::beginSaving(TopoMap *map) {
  const FlatGroundModule::MapData& data = FlatGroundModule::getMapData(map);
  YAML::Node node;
  
  FlatModuleCostsParser parser;
  node["default_costs"] = parser.storeAll(data);
  
  YAML::Node maps;
  for(auto& entry: data) {
    YAML::Node flat;
    
    if(auto planner = std::dynamic_pointer_cast<NavfnCostsPlanner>(entry.second.planner)) {
      flat["planner"] = "NavfnCostsPlanner";
    }
#if 0
    else if(auto planner = std::dynamic_pointer_cast<GridMapCostsPlanner>(entry.second.planner)) {
      flat["planner"] = "GridMapCostsPlanner";
    }
#endif
    else {
      ROS_ERROR_STREAM("Could not save flat map entry for map '" << entry.first << "', since the planner type is unknown");
    }
    
    maps[entry.first] = flat;
  }
  node["maps"] = maps;
  
  return node;
}

YAML::Node FlatParser::saveNodeData(TopoMap::Node const *node) {
	FlatGroundModule::NodeData const& data = FlatGroundModule::getNodeData(node);
  YAML::Node yaml;
  
  yaml["x"] = data.x;
  yaml["y"] = data.y;
  yaml["map"] = data.mapName;
  yaml["height"] = data.areaHeight;
  
	return yaml;
}

  
} // namespace topo_nav
