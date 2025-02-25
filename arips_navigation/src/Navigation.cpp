//
// Created by jgdo on 1/3/21.
//

#include <arips_navigation/Navigation.h>

#include <arips_navigation/FlatGroundModule.h>
#include <arips_navigation/FlatNodeVisualizer.h>
#include <arips_navigation/FlatParser.h>
#include <arips_navigation/StepEdgeModule.h>
#include <arips_navigation/TopoExecuter.h>
#include <std_msgs/Bool.h>
#include <toponav_ros/TopoPlannerROS.h>
#include <toponav_ros/utils/CostsProfileModuleContainer.h>
#include <toponav_ros/utils/EdgeModuleContainer.h>
#include <toponav_ros/utils/NodeModuleContainer.h>

#include <memory>

// #include "arips_navigation/utils/StepModuleFromTracker.h"
#include "arips_navigation/Navigation.h"
#include "arips_navigation/utils/StepCostmapLayer.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <arips_navigation/path_planning/Costmap2dView.h>

using namespace toponav_ros;
using namespace toponav_core;

Navigation::Navigation() {
    ros::NodeHandle nh("~");

    std::shared_ptr<ModuleContainer> factory = std::make_shared<ModuleContainer>();

    std::shared_ptr<CostsProfileModuleContainer> costsModules =
        std::make_shared<CostsProfileModuleContainer>();

    EdgeModuleContainerPtr edgeContainer = std::make_shared<EdgeModuleContainer>();
    {
        const bool dynamicStepsFromDetection = false;
        StepEdgeModulePtr stepModule =
            std::make_shared<StepEdgeModule>(m_TopoPlanner.getContext(), dynamicStepsFromDetection);
        edgeContainer->addPlanningModule("step", stepModule);
        edgeContainer->addStorageModule("step", stepModule);
        edgeContainer->addVisualizationModule("step", stepModule);
        costsModules->addModule(stepModule);

        // auto stepFromTracker = std::make_shared<StepModuleFromTracker>(
        //     m_TopoPlanner.getContext(), mContext->floorStepTracker, *stepModule);
        // edgeContainer->addVisualizationModule("__stepFromTracker__", stepFromTracker);
    }

    NodeModuleContainerPtr nodeContainer = std::make_shared<NodeModuleContainer>();
    {
        auto flatPlanner = FlatGroundModule::create(m_TopoPlanner.getContext());
        nodeContainer->addPlanningModule("flat", flatPlanner);
        nodeContainer->addMapPoseModule("flat", flatPlanner);
        nodeContainer->addVisualizationModule(
            "flat", std::make_shared<FlatNodeVisualizer>(m_TopoPlanner.getContext(), "flat"));
        nodeContainer->addStorageModule(
            "flat",
            std::make_shared<FlatParser>(m_TopoPlanner.getContext(), mContext, &mAripsPlanner));
    }

    factory->addModule<NodeStorageInterface>(nodeContainer);
    factory->addModule<EdgeStorageInterface>(edgeContainer);
    factory->addModule<NodePlanningInterface>(nodeContainer);
    factory->addModule<EdgePlanningInterface>(edgeContainer);
    factory->addModule<NodeVisualizationInterface>(nodeContainer);
    factory->addModule<EdgeVisualizationInterface>(edgeContainer);
    factory->addModule<MapPoseInterface>(nodeContainer);
    factory->addModule<CostsProfileInterface>(costsModules);

    m_TopoPlanner.init("topo_planner", factory, &mContext->tf);

    {
        auto layerPlugin = boost::make_shared<StepCostmapLayer>(mSemanticMapTracker);
        mContext->globalCostmap.getLayeredCostmap()->addPlugin(layerPlugin);
    }

    ros::Duration(0.5).sleep();
    FlatParser::segmentKnownNodes(m_TopoPlanner.getContext().topoMap.get());
    m_TopoPlanner.getContext().mapChanged();

    mDriveTo = std::make_unique<DriveTo>(*mContext, mLocomotion);

    mCrossDoor = std::make_unique<CrossDoor>(*mContext, *mDriveTo, mOpenDoor);

    m_TopoExec = std::make_unique<TopoExecuter>(*mContext, *mDriveTo, m_TopoPlanner, *mCrossDoor,
                                                mCrossStep);

    psub_nav = nh.subscribe("/topo_planner/nav_goal", 1, &Navigation::poseCallbackNavGoal, this);
    // hp_sub = nh.subscribe("/hp_goal", 1, &Navigation::poseCallbackHpGoal, this);
    clicked_sub = nh.subscribe("/clicked_point", 1, &Navigation::onClickedPoint, this);
    door_info_sub = nh.subscribe("/cross_door_info", 1, &Navigation::onDoorInfoReceived, this);

    mActivePub = nh.advertise<std_msgs::Bool>("/arips_navigation_active", 1, false);
    mTopoPathPub = nh.advertise<nav_msgs::Path>("/new_topo_path", 1);

    mControlTimer = nh.createTimer(ros::Duration(0.1), &Navigation::timerCallback, this);
}

void Navigation::poseCallbackNavGoal(const geometry_msgs::PoseStamped& goal) {
    // m_TopoExec->activate(msg);
    // mDrivingState = m_TopoExec.get();

    geometry_msgs::PoseStamped robotPose;
    if (!mContext->globalCostmap.getRobotPose(robotPose)) {
        ROS_WARN_STREAM("poseCallbackNavGoal(): Could not get robot pose");
        return;
    }

    try {
        const auto poseOnFloor =
            mContext->tf.transform(goal, mContext->globalCostmap.getGlobalFrameID());

        const auto topoPlan = mSemanticTopoPlanner.plan(
            Costmap2dView(mContext->globalCostmap), mSemanticMapTracker.getLastSemanticMap(),
            Pose2D::fromMsg(robotPose.pose), Pose2D::fromMsg(poseOnFloor.pose));

        if (topoPlan) {
            ROS_INFO_STREAM("topo planning passed");

            nav_msgs::Path navPath;

            navPath.header.frame_id = mContext->globalCostmap.getGlobalFrameID();
            navPath.header.stamp = ros::Time::now();

            ::LambdaPlanVisitor visitor(
                [&, this](::TopoPath::Movement const* mov) {
                    for (const auto& p : mov->pathPoints) {
                        geometry_msgs::PoseStamped poseStampedMsg;
                        poseStampedMsg.pose = p.toPoseMsg();
                        poseStampedMsg.header = navPath.header;
                        navPath.poses.push_back(poseStampedMsg);
                    }
                },
                [&, this](::TopoPath::Transition const* trans) {

                });

            topoPlan->visitPlan(visitor);

            mTopoPathPub.publish(navPath);
        }

        else {
            ROS_INFO_STREAM("topo planning failed");
        }
    } catch (const tf2::TransformException& ex) {
        ROS_WARN_STREAM("poseCallbackNavGoal(): " << ex.what());
    }
}

/*
void Navigation::poseCallbackHpGoal(const geometry_msgs::PoseStamped& msg) {
    mHPNav.setGoal(msg);
    mDrivingState = &mHPNav;
}*/

void Navigation::timerCallback(const ros::TimerEvent& e) {
    std_msgs::Bool active;
    active.data = (mDrivingState != nullptr);
    mActivePub.publish(active);

    if (!mDrivingState) {
        return;
    }

    if (mDrivingState->isActive()) {
        mDrivingState->runCycle();
    } else {
        ROS_INFO("Finished plan.");
        mDrivingState = nullptr;
    }
}

void Navigation::onClickedPoint(const geometry_msgs::PointStamped&) {
    mOpenDoor.init();
    mDrivingState = &mOpenDoor;
}

void Navigation::onDoorInfoReceived(const arips_navigation::CrossDoorInformation& doorInfo) {
    mCrossDoor->activate(doorInfo);
    mDrivingState = mCrossDoor.get();
}
