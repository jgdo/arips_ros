//
// Created by jgdo on 1/3/21.
//

#include <arips_navigation/Navigation.h>

#include <toponav_ros/TopoPlannerROS.h>
#include <arips_navigation/FlatGroundModule.h>
#include <toponav_ros/utils/NodeModuleContainer.h>
#include <arips_navigation/FlatNodeVisualizer.h>
#include <arips_navigation/FlatParser.h>
#include <toponav_ros/utils/EdgeModuleContainer.h>
#include <arips_navigation/StepEdgeModule.h>
#include <toponav_ros/utils/CostsProfileModuleContainer.h>

#include <memory>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace toponav_ros;
using namespace toponav_core;

Navigation::Navigation() {
    ros::NodeHandle nh("~");

    std::shared_ptr<ModuleContainer> factory = std::make_shared<ModuleContainer>();

    std::shared_ptr<CostsProfileModuleContainer> costsModules = std::make_shared<CostsProfileModuleContainer>();

    EdgeModuleContainerPtr edgeContainer = std::make_shared<EdgeModuleContainer>();
    {
        StepEdgeModulePtr stepModule = std::make_shared<StepEdgeModule>(m_TopoPlanner.getContext());
        edgeContainer->addPlanningModule("step", stepModule);
        edgeContainer->addStorageModule("step", stepModule);
        edgeContainer->addVisualizationModule("step", stepModule);
        costsModules->addModule(stepModule);
    }

    NodeModuleContainerPtr nodeContainer = std::make_shared<NodeModuleContainer>();
    {
        auto flatPlanner = FlatGroundModule::create(m_TopoPlanner.getContext());
        nodeContainer->addPlanningModule("flat", flatPlanner);
        nodeContainer->addMapPoseModule("flat", flatPlanner);
        nodeContainer->addVisualizationModule("flat", std::make_shared<FlatNodeVisualizer>(m_TopoPlanner.getContext(), "flat"));
        nodeContainer->addStorageModule("flat", std::make_shared<FlatParser>(m_TopoPlanner.getContext()));
    }

    factory->addModule<NodeStorageInterface>(nodeContainer);
    factory->addModule<EdgeStorageInterface>(edgeContainer);
    factory->addModule<NodePlanningInterface>(nodeContainer);
    factory->addModule<EdgePlanningInterface>(edgeContainer);
    factory->addModule<NodeVisualizationInterface>(nodeContainer);
    factory->addModule<EdgeVisualizationInterface>(edgeContainer);
    factory->addModule<MapPoseInterface>(nodeContainer);
    factory->addModule<CostsProfileInterface>(costsModules);

    m_TopoPlanner.init("topo_planner", factory, &m_tfBuffer);

    psub_nav = nh.subscribe("/topo_planner/nav_goal", 10, &Navigation::poseCallbackNavGoal, this);
}

// from https://github.com/strawlab/navigation/blob/master/move_base/src/move_base.cpp
static bool isQuaternionValid(const tf2::Quaternion& tf_q) {
    //first we need to check if the quaternion has nan's or infs
    if(!std::isfinite(tf_q.x()) || !std::isfinite(tf_q.y()) || !std::isfinite(tf_q.z()) || !std::isfinite(tf_q.w())){
        ROS_ERROR("Quaternion has nans or infs... discarding pose");
        return false;
    }

    //next, we need to check if the length of the quaternion is close to zero
    if(tf_q.length2() < 1e-6){
        ROS_ERROR("Quaternion has length close to zero... discarding pose");
        return false;
    }

    return true;
}

auto static createQuaternionFromYaw(double yaw)
{
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return q;
}

void Navigation::poseCallbackNavGoal(const geometry_msgs::PoseStamped &msg) {
    tf2::Stamped<tf2::Transform> pose;
    tf2::fromMsg(msg, pose);

    if(!isQuaternionValid(pose.getRotation()))
        return;

    tf2::Stamped<tf2::Transform> robotPose;
    robotPose.setRotation(createQuaternionFromYaw(0));
    robotPose.frame_id_ = "flat_layer_map1_frame";
    robotPose.stamp_ = ros::Time::now();

    GlobalPosition start = m_TopoPlanner.getContext().poseService->findGlobalPose(robotPose, *m_TopoPlanner.getContext().topoMap).first;
    ROS_INFO_STREAM("found start node for robot pose: " << (start.node ? start.node->getName() : std::string("<NOT FOUND>")));

    GlobalPosition goal = m_TopoPlanner.getContext().poseService->findGlobalPose(pose, *m_TopoPlanner.getContext().topoMap).first;
    ROS_INFO_STREAM("found goal node for pose: " << (goal.node ? goal.node->getName() : std::string("<NOT FOUND>")));

    if (start.node && goal.node) {
        try {
            TopoPath lastPlan;
            m_TopoPlanner.getContext().pathPlanner->plan(m_TopoPlanner.getContext().topoMap.get(), start, goal, &lastPlan, nullptr);
            m_TopoPlanner.getPathViz().visualizePath(lastPlan);
            m_TopoPlanner.getContext().planQueue.addPath(lastPlan);
        } catch (const std::exception &e) {
            ROS_ERROR_STREAM("Exception when calling plan: " << e.what());
        }
    }
}
