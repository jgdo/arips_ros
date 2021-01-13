#include <arips_navigation/FlatGroundModule.h>
#include <arips_navigation/utils/ApproachLineArea.h>
#include <arips_navigation/utils/FixedPosition.h>

#include <set>
#include <toponav_ros/utils/CommonCostProfiles.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace toponav_ros {

static bool costValid(int cost) {
    return cost != costmap_2d::LETHAL_OBSTACLE
        && cost != costmap_2d::INSCRIBED_INFLATED_OBSTACLE
        && cost != costmap_2d::NO_INFORMATION;
}

using namespace toponav_core;

const std::string FlatGroundModule::className = "topo_nav::FlatGroundModule";

void FlatGroundModule::initNodeData(TopoMap::Node *node, CostsPlannerPtr planner, NodeMatrixPtr nodeMatrix,
                                    tf2_ros::Buffer* tfBuffer, int x, int y, std::string mapName, double height) {
  node->nodeData() = NodeData {planner, nodeMatrix, tfBuffer, x, y, mapName, height};
  node->propertyMap()[TopoMap::ENTRY_DOT_COLOR] = std::string("powderblue");
}

size_t FlatGroundModule::regionGrow(TopoMap::Node *node, const costmap_2d::Costmap2D* costmap, int x, int y, NodeMatrix *mat)
{
  if(mat->rows() != costmap->getSizeInCellsX() || mat->cols() != costmap->getSizeInCellsY()) {
      throw std::runtime_error("FlatGroundModule::regionGrow(): different node matrix and grid map size");
  }

  x = mat->rows() - 1 - x; // Invert x

  using Index = std::pair<int, int>;

  std::list<Index> points;
  points.emplace_back(x,y);
  
  size_t counter = 0;
  while(!points.empty()) {
    Index p = points.front();
    points.pop_front();
  
    // check out of range
    if(p.first < 0 || p.first >= mat->rows() || p.second< 0 || p.second >= mat->cols()) {
        continue;
    }
  
    // check for region grow
      const auto cost = costmap->getCost(p.first, p.second);

    if(!(*mat)(p.first, p.second)
            && cost != costmap_2d::LETHAL_OBSTACLE
            && cost != costmap_2d::INSCRIBED_INFLATED_OBSTACLE
            && cost != costmap_2d::NO_INFORMATION) {
      (*mat) (p.first, p.second) = node;
      counter++;

      points.emplace_back(p.first+1, p.second);
      points.emplace_back(p.first-1, p.second);
      points.emplace_back(p.first, p.second+1);
      points.emplace_back(p.first, p.second-1);
    }
  }
  
  return counter;
}

tf2::Stamped<tf2::Transform> FlatGroundModule::getMapPoseFromPosition(GlobalPosition const& pos) {
  tf2::Stamped<tf2::Transform> msg = getPositionDataFromGlobal(pos).pose;
  tf2::Stamped<tf2::Transform> transformed = _context.tfBuffer->transform(msg, _context.globalFrame);
  return transformed;
}

std::pair<GlobalPosition, tf2::Stamped<tf2::Transform>> FlatGroundModule::findGlobalPose(
    const tf2::Stamped<tf2::Transform> &pose, const TopoMap &map)
{
  GlobalPosition gp;
  
  if(pose.frame_id_.empty()) {
      ROS_ERROR("FlatGroundModule::findGlobalPose(): pose cannot have an empty frame_id");
    std::make_pair(gp, tf2::Stamped<tf2::Transform>());
  }
  
  tf2::Stamped<tf2::Transform> localPose;
  TopoMap::Node const* realNode = nullptr;
  
  for(auto& mapEntry: getMapData(&map)) {
     const auto& costmap_ros = mapEntry.second.planner->getMap();

    try {
        localPose = _context.tfBuffer->transform(pose, costmap_ros.getGlobalFrameID());
    } catch (const tf2::TransformException &ex) {
      ROS_WARN("FlatGroundModule::findGlobalPose(): %s", ex.what());
      continue;
    }
  
    if (std::abs(localPose.getOrigin().z()) < 0.5) { // FIXME: better check if this is the costmap
        unsigned int mx, my;

      // returns false if outside costmap bounds
      if (costmap_ros.getCostmap()->worldToMap(localPose.getOrigin().x(), localPose.getOrigin().y(), mx, my)) {
        realNode = (*mapEntry.second.nodeMatrix)(mx, my);
        if (realNode) {
          localPose.getOrigin().setZ(getNodeData(realNode).areaHeight); // lookup the configured height of this node
          break;
        }
      }
    }
  }
  
  gp.localPos = std::make_shared<PositionData>(localPose);
  gp.node = realNode;
  return std::make_pair(gp, localPose);
}

std::string FlatGroundModule::getNodeGlobalFrame(const TopoMap::Node *node) {
    const NodeData& nodeData = getNodeData(node);
    return nodeData.planner->getMap().getGlobalFrameID();
}

static void combineNodeMatrix(FlatGroundModule::NodeMatrix &mat, const FlatGroundModule::NodeMatrix& other) {
    for(int x = 0; x < mat.rows(); x++) {
        for (int y = 0; y < mat.cols(); y++) {
            if(other(x,y)) {
                mat(x,y) = other(x,y);
            }
        }
    }
};

void FlatGroundModule::segmentAllNodes(TopoMap *map, std::string const &mapName,
                                       std::string const &nodeType, size_t minNumCells) {
  MapPlanningDataEntry& entry = getMapData(map).at(mapName);
  const auto& costmap_ros = entry.planner->getMap();
  
  // FIXME: what if layer not present?
  const auto occupancy = costmap_ros.getCostmap();
  
  NodeMatrix& mat = *entry.nodeMatrix;
  size_t nodeCount = 0;

  const auto startNumNodes = map->getNumNodes();
  
  for(int x = 0; x < mat.rows(); x++) { // rows = x coordinate, see regionGrow() impl
    for(int y = 0; y < mat.cols(); y++) {
      if(!mat(x,y) && costValid(occupancy->getCost(x,y))) {
        // find free node name
        std::string name;
        do{
          name = mapName + "_node_" + std::to_string(nodeCount++);
        } while(map->getNode(name));
  
        TopoMap::Node* node = map->addNode(name, nodeType);
  
        NodeMatrix local_mat = NodeMatrix::Constant(mat.rows(), mat.cols(), nullptr);
        size_t numCells = regionGrow(node, occupancy, mat.rows() - 1 - x, y, &local_mat);
        if(minNumCells == 0 || numCells >= minNumCells) {
            combineNodeMatrix(mat, local_mat); // merge both matrices
          FlatGroundModule::initNodeData(node, entry.planner, entry.nodeMatrix,
                  entry.tfBuffer, mat.rows() - 1 - x, y, mapName, 0); // default height is 0
        } else { // area has to few cells, ignore
          map->removeNode(node);
        }
      }
    }
  }

    const auto addedNumNodes = map->getNumNodes() - startNumNodes;
  ROS_INFO_STREAM("Segmented costmap into " << addedNumNodes << " additional nodes.");

}

std::pair<double, LocalPositionConstPtr>
FlatGroundModule::computeCostsOnRegion(const TopoMap::Node *node, LocalPosition const &currentPos, AbstractApproachExitData const &end, AbstractPathData *pathData) {
  auto& approach = *boost::any_cast<ApproachExit3DPtr>(&end);
  const NodeData& nodeData = getNodeData(node);
  const PositionData& srcData = *dynamic_cast<const PositionData*>(&currentPos);
  auto& mapData = getMapData(node->getParentMap());

  tf2::Stamped<tf2::Transform> startTransformed = _context.tfBuffer->transform(srcData.pose, nodeData.planner->getMap().getGlobalFrameID());
  
  std::vector<geometry_msgs::PoseStamped> plan;
  geometry_msgs::PoseStamped startMsg;
  tf2::toMsg(startTransformed, startMsg);
  
  double distance;
  tf2::Stamped<tf2::Transform> actualApproachPose;
  // needed since local planner expects a height of 0
  startMsg.pose.position.z = 0;
  bool success = nodeData.planner->makePlan(startMsg, approach, plan, &distance, &actualApproachPose);
  
  if(!success) {
    return std::make_pair(std::numeric_limits<double>::max(), LocalPositionConstPtr());
  }
  
  // throw std::runtime_error("FlatGroundModule::computeCostOnRegion() for region '" + node->getName() + "' planning failed");
  
  double costs = getCosts(mapData, distance);
  
  if(pathData) {
    std::vector<geometry_msgs::PoseStamped> planTransformed;
    planTransformed.reserve(plan.size());
    
    for (auto &pose: plan) {
      // set the path height back to the height of the area
      pose.pose.position.z = nodeData.areaHeight;
      geometry_msgs::PoseStamped tp = _context.tfBuffer->transform(pose, _context.globalFrame);
      planTransformed.push_back(tp);
    }
    
    *pathData = plan; // planTransformed;
  }
  
  auto pos_data = std::make_shared<PositionData>(actualApproachPose);
  pos_data->goal_properties = approach->goal_properties;
  return std::make_pair(costs, pos_data);
}

void
FlatGroundModule::convertGlobalPoseToApproachExitData(const GlobalPosition &pos,
                                                        AbstractApproachExitData *approachData) {
  // every used planner always assumes that approach/exit data is an ApproachExit3DPtr. Node that any_cast do not supprt automatic upcasting,
  // so storing explicitely as ApproachExit3DPtr and not as the specialized type ptr is required!
  auto& pos_data = getPositionDataFromGlobal(pos);
  ApproachExit3DPtr ptr = std::make_shared<FixedPosition>(pos_data.pose);
  ptr->goal_properties = pos_data.goal_properties;
  *approachData = ptr;
}

double FlatGroundModule::getHeuristics(TopoMap::Node const *region, AbstractApproachExitData const &startData, AbstractApproachExitData const &endData) {
  auto start = (*boost::any_cast<ApproachExit3DPtr>(&startData))->getCenter();
  auto end = (*boost::any_cast<ApproachExit3DPtr>(&endData))->getCenter();

  end = _context.tfBuffer->transform(end, start.frame_id_); // transform end to start frame
  
  return getCosts(getMapData(region->getParentMap()), (start.getOrigin() - end.getOrigin()).length());
}

std::string FlatGroundModule::getModuleType(const TopoMap::Node *node) {
  return className;
}

double FlatGroundModule::getCosts(const BaseTraits<FlatGroundModule>::MapData &data, double distance) {
  std::map<std::string, double> costs {{CommonCostProfiles::DEFAULT_PROFILE, data.time_factor * distance},
                                       {CommonCostProfiles::TIME_COSTS, data.time_factor * distance},
                                       {CommonCostProfiles::DISTANCE_COSTS, data.distance_factor * distance},
                                       {CommonCostProfiles::ENERGY_COSTS, data.energy_factor * distance}
  };
  
  return costs_comp_.computeCosts(costs, className);
}

const FlatGroundModule::PositionData &FlatGroundModule::getPositionDataFromGlobal(GlobalPosition const& pos) {
  return *std::dynamic_pointer_cast<PositionData const>(pos.localPos);
}
  
}; // namespace topo_nav
