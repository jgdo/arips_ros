#include <arips_navigation/FlatParser.h>

#include <arips_navigation/FlatGroundModule.h>
#include <toponav_ros/PlanningContext.h>
#include <toponav_ros/TopoMapYamlStorage.h>

#include <arips_navigation/utils/ApproachLineArea.h>

#include <arips_navigation/CostsPlanners.h>
#include <arips_navigation/utils/FixedPosition.h>
#include <toponav_ros/utils/YamlTools.h>

namespace toponav_ros {

using namespace toponav_core;

GEN_COSTS_READER_WRITER(FlatModuleCostsParser, BaseTraits<FlatGroundModule>::MapData,
                        {"time_factor", &BaseTraits<FlatGroundModule>::MapData::time_factor},
                        {"energy_factor", &BaseTraits<FlatGroundModule>::MapData::energy_factor},
                        {"distance_factor",
                         &BaseTraits<FlatGroundModule>::MapData::distance_factor});

void FlatParser::beginParsing(TopoMap* map, YAML::Node const& parserConfig) {
    currentMap = map;

    FlatGroundModule::MapData mapData;

    FlatModuleCostsParser parser;
    parser.parse(&mapData, parserConfig["default_costs"], FlatGroundModule::className,
                 "default costs");

    mapData.planner = mPlanner;
    mapData.mNavContext = mNavContext;
    auto* costmap = &mapData.mNavContext->globalCostmap;
    // FIXME: big hack for costmap init waiting
    auto last = ros::Time::now();
    while (!costmap->getCostmap() || costmap->getCostmap()->getSizeInCellsX() == 0) {
        if ((ros::Time::now() - last).toSec() > 5.0) {
            ROS_WARN_STREAM("Topo Planner: still waiting for global costmap");
            last = ros::Time::now();
        }

        ros::spinOnce();
        ros::Duration(0.2).sleep();
    }

    mapData.nodeMatrix =
        FlatGroundModule::NodeMatrix::Constant(costmap->getCostmap()->getSizeInCellsX(),
                                               costmap->getCostmap()->getSizeInCellsY(), nullptr);

    FlatGroundModule::setMapData(map, mapData);
}

void FlatParser::endParsing() {}

void FlatParser::parseNodeData(YAML::Node const& config, TopoMap::Node* node) {
    int nodeX = config["x"].as<int>();
    int nodeY = config["y"].as<int>();

    auto& mapData = FlatGroundModule::getMapData(currentMap);

    FlatGroundModule::initNodeData(node, nodeX, nodeY);
            FlatGroundModule::regionGrow(node, mapData.planner->getMap().getCostmap(),
}

YAML::Node FlatParser::beginSaving(TopoMap* map) {
    const FlatGroundModule::MapData& data = FlatGroundModule::getMapData(map);
    YAML::Node node;

    FlatModuleCostsParser parser;
    node["default_costs"] = parser.storeAll(data);
    return node;
}

YAML::Node FlatParser::saveNodeData(TopoMap::Node const* node) {
    FlatGroundModule::NodeData const& data = FlatGroundModule::getNodeData(node);
    YAML::Node yaml;

    yaml["x"] = data.x;
    yaml["y"] = data.y;
    return yaml;
}

void FlatParser::segmentKnownNodes(TopoMap* map) {
    auto& mapData = FlatGroundModule::getMapData(map);
    map->foreachNode([&](TopoMap::Node* node) {
        const auto& nodeData = FlatGroundModule::getNodeData(node);

        // FIXME: what if layer not present?
        const size_t cells =
            FlatGroundModule::regionGrow(node, mapData.planner->getMap().getCostmap(), nodeData.x,
                                         nodeData.y, &mapData.nodeMatrix);
        if (cells > 0) {
            ROS_DEBUG_STREAM("FlatGroundModule: segmented " << cells << " cells for node "
                                                            << node->getName());
        } else {
            ROS_ERROR_STREAM("FlatGroundModule: Failed to segments cells for node '"
                             << node->getName() << "'. Planning with this node will likely fail!");
        }
    });
}

} // namespace toponav_ros
