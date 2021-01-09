#include <toponav_ros/TopoPlannerROS.h>
#include <toponav_ros/utils/CostProfileCalculator.h>
#include <toponav_ros/utils/CommonCostProfiles.h>
#include <toponav_ros/TopoPlannerLoader.h>

using namespace toponav_core;

bool toponav_ros::TopoPlannerROS::init(std::string const &name,
        const std::shared_ptr<toponav_core::ModuleContainer> &factory,
        tf2_ros::Buffer* tfBuffer) {
  ros::NodeHandle pnh("~/" + name);

  if(isInitialized()) {
    ROS_ERROR_STREAM("Failed to init TopoPlannerROS: already initialized.");
    return false;
  }

  std::string global_frame;
  if(!pnh.getParam("global_frame_id", global_frame)) {
    ROS_ERROR_STREAM("Failed to init TopoPlannerROS: provided global_frame_id param is empty.");
    return false;
  }

  if(!factory) {
    ROS_ERROR_STREAM("Failed to init TopoPlannerROS: provided factory is null.");
    return false;
  }

  _context.tfBuffer = tfBuffer;
  _context.globalFrame = global_frame;

  /**
   * First try to create all components and check if plugins are missing
   */
  try {
    _context.poseService = factory->getCreateModule<MapPoseInterface>();
    _context.nodePlanner = factory->getCreateModule<NodePlanningInterface>();
    _context.edgePlanner = factory->getCreateModule<EdgePlanningInterface>();

    _mapEditor = std::make_shared<MapEditor>(*factory, _context);

    std::string planner_type = pnh.param<std::string>("planner_type", "PathRepairTopoPlanner");
    TopoPlannerLoader planner_loader;
    _context.pathPlanner = planner_loader.loadTopoPlanner(planner_type, planner_type, *factory);

    _pathViz = std::make_shared<TopoPathVisualizer>(*factory, _context);
  } catch (std::exception const& ex) {
    ROS_ERROR_STREAM("Failed to init TopoPlannerROS: failed to create required componend. Caught exception: " << ex.what());
    return false;
  }

  auto topoMap = std::make_shared<TopoMap>();

  // then parse topo map
  std::string topo_map_name;
  if(!pnh.getParam("topo_map_file", topo_map_name)) {
    ROS_ERROR_STREAM("Failed to init TopoPlannerROS: no 'topo_map_file' parameter is provided.");
    return false;
  }

  try {
    TopoMapYamlStorage::parseMap(*topoMap, topo_map_name, *factory);
  } catch (std::exception const& ex) {
    ROS_ERROR_STREAM("Failed to init TopoPlannerROS: failed to parse topo map from '" << topo_map_name << "'. \nCaught exception: " << ex.what());
    return false;
  }

  // apply only if everything gone wright
  _context.topoMap = topoMap;
  _factory = factory;
  _mapEditor->startEditMap();

  cost_profile_ = factory->getCreateModule<CostsProfileInterface>();

  ROS_INFO("Topo Planning Ready!");

  return true;
}

void toponav_ros::TopoPlannerROS::setCostsProfile(std::string const &profile) {
  ROS_INFO_STREAM("TopoPlannerROS: cost profile set to '" << profile << "'");
  cost_profile_->setCostsProfile(profile.empty()? CommonCostProfiles::DEFAULT_PROFILE : profile);
}
