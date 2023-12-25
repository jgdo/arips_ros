#include <arips_navigation/utils/ApproachLineArea.h>
#include <toponav_ros/MapEditor.h>
#include <toponav_ros/TopoMapYamlStorage.h>

#include <arips_navigation/FlatGroundModule.h>
#include <arips_navigation/StepEdgeModule.h>
#include <toponav_ros/utils/CommonCostProfiles.h>
#include <toponav_ros/utils/YamlTools.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace toponav_ros {

using namespace toponav_core;

std::string findFreeStepName(StepEdgeModule::MapStepData const& stepsData) {
    size_t i = stepsData.steps.size();
    std::string name;
    do {
        name = "Step_" + std::to_string(i);
        i++; // FIXME: more efficient search
    } while (stepsData.steps.find(name) != stepsData.steps.end());
    return name;
}

GEN_COSTS_READER_WRITER(StepEdgeModuleCostsParser, BaseTraits<StepEdgeModule>::CostSettings,
                        {"time", &BaseTraits<StepEdgeModule>::CostSettings::time_costs},
                        {"energy", &BaseTraits<StepEdgeModule>::CostSettings::energy_costs},
                        {"distance", &BaseTraits<StepEdgeModule>::CostSettings::distance_costs});

using namespace visualization_msgs;
using namespace interactive_markers;

const std::string StepEdgeModule::className = "topo_nav::StepEdgeModule";

static std::string edgeType = "step";

StepEdgeModule::StepEdgeModule(PlanningContext& context) : ApproachExitVisualizer{context} {
    ros::NodeHandle nh;
    mFloorStepSub = nh.subscribe("floor_step", 10, &StepEdgeModule::onFloorStepCb, this);
}

void StepEdgeModule::onFloorStepCb(const geometry_msgs::PolygonStamped& msg) {
    const auto trans =
        tryLookupTransform(*_context.tfBuffer, _context.globalFrame, msg.header.frame_id);
    if (!trans) {
        return;
    }

    auto toP2d = [&trans](const geometry_msgs::Point32& p) {
        const auto pTrans = (*trans)(tf2::Vector3{p.x, p.y, p.z});
        return Point2d{pTrans.x(), pTrans.y()};
    };

    if (msg.polygon.points.size() == 4) {
        clearArea({toP2d(msg.polygon.points.at(0)), toP2d(msg.polygon.points.at(1)),
                   toP2d(msg.polygon.points.at(2)), toP2d(msg.polygon.points.at(3))});
    } else if (msg.polygon.points.size() == 6) {
        const FloorStep2d s{toP2d(msg.polygon.points.at(4)), toP2d(msg.polygon.points.at(5))};
        trackStep(s, msg.header.stamp);
    } else {
        ROS_WARN_STREAM("FloorStepTracker received polygon with size "
                        << msg.polygon.points.size() << ", but expected 4 or 6. Ignoring.");
        return;
    }
}

void StepEdgeModule::trackStep(const FloorStep2d& obsStep, ros::Time stamp) {
    MapStepData& stepData = getMapData(_context.topoMap.get());

    for (auto& stepEntry : stepData.steps) {
        // iterate only over tracked steps
        TrackedStepInfo* step = dynamic_cast<TrackedStepInfo*>(stepEntry.second.get());

        if (step->isWithinTolerance(obsStep, TRACK_TOLERANCE_M)) {
            step->track(obsStep);

            // collect edges of this step
            mapEditor->getMap()->foreachEdge([&](TopoMap::Edge* edge) {
                if (_context.isModuleType<StepEdgeModule>(edge)) {
                    auto& edgeData = getEdgeData(edge);
                    if (edgeData.stepName == step->name) {
                        setApproachFromStep(edge);

                    }
                }
            });

            _context.mapChanged();
            return;
        }
    }

    const auto newName = findFreeStepName(stepData);

    ROS_INFO_STREAM("Adding new step: " << newName);

    tf2::Stamped<tf2::Vector3> start(tf2::Vector3(obsStep[0].x(), obsStep[0].y(), 0.0), stamp,
                                     _context.globalFrame);
    tf2::Stamped<tf2::Vector3> end(tf2::Vector3(obsStep[1].x(), obsStep[1].y(), 0.0), stamp,
                                   _context.globalFrame);

    auto stepInfo = std::make_shared<TrackedStepInfo>(
        start, end, newName, stepData.defaultUpCosts, stepData.defaultDownCosts,
        stepData.defaultCrossOverCosts, stepData.defaultCrossGravelCosts);
    stepData.steps[newName] = stepInfo;
    createBothStepEdges(stepInfo.get());
}

void StepEdgeModule::clearArea(const std::vector<Point2d>& points) {
    // Clearing not implemented yet ...
}

void StepEdgeModule::beginParsing(TopoMap* map, YAML::Node const& parserConfig) {
    currentMap = map;
    currentStepData = MapStepData();

    if (!parserConfig.IsMap()) {
        ROS_WARN("StepEdgeModule: no step definitions in map specified");
        return;
    }

    YAML::Node const steps = parserConfig[edgeType];

    // collect steps
    if (!steps.IsSequence()) {
        ROS_WARN("StepEdgeModule: no step definitions in map specified");
        return;
    }

    StepEdgeModuleCostsParser parser;
    parser.parse(&currentStepData.defaultDownCosts, parserConfig["default_costs_down"], className,
                 "1.0");
    parser.parse(&currentStepData.defaultUpCosts, parserConfig["default_costs_up"], className, "1");
    parser.parse(&currentStepData.defaultCrossOverCosts, parserConfig["default_costs_cross_over"],
                 className, "1");
    parser.parse(&currentStepData.defaultCrossGravelCosts,
                 parserConfig["default_costs_cross_gravel"], className, "1");

    for (size_t i = 0; i < steps.size(); i++) {
        YAML::Node const stepDef = steps[i];

        std::string name = stepDef["name"].as<std::string>();
        tf2::Stamped<tf2::Vector3> start, end;
        start.setData(stepDef["start"].as<tf2::Vector3>());
        end.setData(stepDef["end"].as<tf2::Vector3>());
        std::string frame = YamlTools::parseFrameID(
            stepDef["frame_id"],
            "Error when parsing map step objects: frame_id for step '" + name + "' is empty");
        start.frame_id_ = frame;
        end.frame_id_ = frame;
        CostSettings upCosts = currentStepData.defaultUpCosts,
                     downCosts = currentStepData.defaultDownCosts,
                     crossCosts = currentStepData.defaultCrossOverCosts,
                     crossGravelCosts = currentStepData.defaultCrossGravelCosts;
        parser.parse(&downCosts, stepDef["costs_down"], className,
                     "stairs '" + name + "'down costs");
        parser.parse(&upCosts, stepDef["costs_up"], className, "stairs '" + name + "'up costs");
        parser.parse(&crossCosts, stepDef["costs_cross_over"], className,
                     "stairs '" + name + "'cross over costs");
        parser.parse(&crossGravelCosts, stepDef["costs_cross_gravel"], className,
                     "stairs '" + name + "'cross gravel costs");

        currentStepData.steps[name] = std::unique_ptr<StepInfo>(
            new StepInfo{start, end, name, upCosts, downCosts, crossCosts, crossGravelCosts});
    }

    goal_properties.readFromParams("~/modules/" + edgeType);
}

void StepEdgeModule::endParsing() {
    currentMap->propertyMap()[className] = std::move(currentStepData);
    currentMap = nullptr;
}

void StepEdgeModule::parseEdgeData(YAML::Node const& config, TopoMap::Edge* edge) {
    std::string stepName = config["step_name"].as<std::string>();
    auto type_str = config["step_type"].as<std::string>();
    EdgeStepData::StepType type = EdgeStepData::UP;
    if (type_str == "up")
        type = EdgeStepData::UP;
    else if (type_str == "down")
        type = EdgeStepData::DOWN;
    else if (type_str == "cross_over")
        type = EdgeStepData::CROSS_OVER;
    else if (type_str == "cross_gravel")
        type = EdgeStepData::CROSS_GRAVEL;
    else
        ROS_WARN_STREAM("step_type for step "
                        << stepName
                        << " has invalid value. Valid values are {'up', 'down', 'cross_over', "
                           "'cross_gravel'}. Using default value 'up'.");

    initEdgeData(edge, stepName, type);

    if (currentStepData.steps.find(stepName) == currentStepData.steps.end())
        ROS_WARN_STREAM("StepEdgeModule::parseEdgeData(): definition for step \""
                        << stepName << "\" could not be found");
}

std::pair<double, LocalPositionConstPtr>
StepEdgeModule::computeTransitionCost(const TopoMap::Edge* edge, const LocalPosition& approachPose,
                                      boost::any* pathData) {
    const MapStepData& mapData = getMapData(_context.topoMap.get());
    auto& data = getEdgeData(edge);
    auto& entry = *mapData.steps.at(data.stepName);
    double costs = getCosts(selectCosts(entry, data.type));

    const PositionData& approach = *dynamic_cast<const PositionData*>(&approachPose);
    return std::make_pair(
        costs, PositionDataPtr(new PositionData(getExitPoseFromApproach(entry, approach.pose))));

    /*
    if(ApproachLineAreaPtr approachLine = std::dynamic_pointer_cast<ApproachLineArea>(inData)) {
      tf2::Stamped<tf2::Transform> approach(tf2::Transform(approachLine->orientation,
    (approachLine->start + approachLine->end)*0.5), approachLine->orientation.stamp_,
    approachLine->orientation.frame_id_);

      return std::make_pair(costs, PositionDataPtr(new PositionData(approach)));
    } else if(FixedPositionPtr approachPose = std::dynamic_pointer_cast<FixedPosition>(inData)) {
      return std::make_pair(costs, PositionDataPtr(new PositionData(approachPose->pose)));
    } else {
      ROS_WARN_STREAM("Failed to create exit position for edge '" << edge->getName() << "' since
    exit area type is unknown"); return std::make_pair(costs, LocalPositionConstPtr()); // FIXME!!
    } */
}

void StepEdgeModule::visualizeEdge(const TopoMap::Edge* edge) {
    auto const& approachData = getApproachData(edge);
    auto const& exitData = getExitData(edge);

    auto markerNames = createEdgeMarkers(edge, approachData, exitData,
                                         visualization_msgs::InteractiveMarkerControl::MENU);

    if (!markerNames.empty()) {
        EdgeStepData& data = getEdgeData((TopoMap::Edge*)edge);

        EdgeStepData::MenuHandlerPtr& menu_handler = data.menu_handler;
        if (!menu_handler) {
            menu_handler = std::make_shared<interactive_markers::MenuHandler>();
            menu_handler->insert("Up", boost::bind(&StepEdgeModule::setEdgeUpDown, this, _1, edge));
            menu_handler->insert("Down",
                                 boost::bind(&StepEdgeModule::setEdgeUpDown, this, _1, edge));
            menu_handler->insert("Cross Over",
                                 boost::bind(&StepEdgeModule::setEdgeUpDown, this, _1, edge));
            menu_handler->insert("Cross Gravel",
                                 boost::bind(&StepEdgeModule::setEdgeUpDown, this, _1, edge));
            menu_handler->insert(
                "Retrieve position from step",
                boost::bind(&StepEdgeModule::setApproachFromStep, this, edge));
        }

        menu_handler->setCheckState(1, data.type == EdgeStepData::UP
                                           ? interactive_markers::MenuHandler::CHECKED
                                           : interactive_markers::MenuHandler::UNCHECKED);
        menu_handler->setCheckState(2, data.type == EdgeStepData::DOWN
                                           ? interactive_markers::MenuHandler::CHECKED
                                           : interactive_markers::MenuHandler::UNCHECKED);
        menu_handler->setCheckState(3, data.type == EdgeStepData::CROSS_OVER
                                           ? interactive_markers::MenuHandler::CHECKED
                                           : interactive_markers::MenuHandler::UNCHECKED);
        menu_handler->setCheckState(4, data.type == EdgeStepData::CROSS_GRAVEL
                                           ? interactive_markers::MenuHandler::CHECKED
                                           : interactive_markers::MenuHandler::UNCHECKED);

        for (auto const& name : markerNames) {
            menu_handler->apply(mapEditor->getMarkerServer(), name);
            // mapEditor->getMarkerServer().setCallback(name,
            // boost::bind(&StepEdgeModule::setEdgeUpDown, this, _1, &data),
            //                                          visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT);
        }
    }
}

void StepEdgeModule::appendTransitionToPlan(nav_msgs::Path* pathPtr, std::string transitionType,
                                            boost::any const& pathData) {
    // leave empty for now
}

void StepEdgeModule::beginMapVisualization() {
    MapStepData& stepData = getMapData(mapEditor->getMap());

    for (auto& entry : stepData.steps) {
        visualizeMapStep(entry.second.get());
    }
}

void StepEdgeModule::endMapVisualization() {}

void StepEdgeModule::createNewMapStep(TopoMap* map, std::string name,
                                      tf2::Stamped<tf2::Vector3> const& start,
                                      const tf2::Stamped<tf2::Vector3>& end) {
    MapStepData& stepData = getMapData(map);
    stepData.steps[name] = std::unique_ptr<StepInfo>{
        new StepInfo{start, end, name, stepData.defaultUpCosts, stepData.defaultDownCosts,
                     stepData.defaultCrossOverCosts, stepData.defaultCrossGravelCosts}};
}

void StepEdgeModule::initEdgeData(TopoMap::Edge* edge, std::string stepName,
                                  EdgeStepData::StepType stepType) {
    edge->edgeData() = EdgeStepData{stepName, stepType};
    edge->propertyMap()[TopoMap::ENTRY_DOT_COLOR] = std::string("orange");
}

void StepEdgeModule::initializeVisualization(MapEditor* editor) {
    EdgeVisualizationInterface::initializeVisualization(editor);
    markerColor = tf2::Vector3(1, 0.5, 0);

    mapEditor->getMenuHandler().insert(mapEditor->getCreateEdgesMenuHandle(), "Step Edge",
                                       boost::bind(&StepEdgeModule::createEdgeCB, this, _1));
    mapEditor->getMenuHandler().insert(mapEditor->getOtherMenuHandle(), "Create Map Step",
                                       boost::bind(&StepEdgeModule::createMapStepCB, this, _1));
}

void StepEdgeModule::selectMapStepCB(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
    if (vizState == WAITING_STEP_SELECTION) {
        selectedStepName = feedback->control_name;
        vizState = CREATING_EDGE_APPROACH;

        ROS_INFO_STREAM("Got step name " << selectedStepName << ", waiting aproach point");
    }
}

void StepEdgeModule::activate() {}

void StepEdgeModule::deactivate() { vizState = IDLE; }

void StepEdgeModule::poseCallback(geometry_msgs::PoseStamped const& msg) {
    if (vizState == CREATING_MAP_STEP) {
        // tf2::Vector3 scale(0.2, 1, 0.1);

        // add an offset in z direction such that step stands on the ground
        tf2::Stamped<tf2::Transform> newPose;
        tf2::fromMsg(msg, newPose);
        // newPose.getOrigin() += newPose.getBasis()*tf2::Vector3(0,0,scale.z()/2);

        tf2::Stamped<tf2::Vector3> start(newPose(tf2::Vector3(0, -0.5, 0.0)), newPose.stamp_,
                                         newPose.frame_id_);
        tf2::Stamped<tf2::Vector3> end(newPose(tf2::Vector3(0, 0.5, 0.0)), newPose.stamp_,
                                       newPose.frame_id_);

        auto& stepsData = getMapData(mapEditor->getMap());
        StepEdgeModule::createNewMapStep(mapEditor->getMap(), findFreeStepName(stepsData), start,
                                         end);

        _context.mapChanged();

        vizState = IDLE;
        mapEditor->deactivateEditorModule(this);

        ROS_INFO_STREAM("Got step pose, map step created, finished");
        return;
    }

    tf2::Stamped<tf2::Transform> pose;
    GlobalPosition gp;
    std::tie(gp, pose) = _context.poseService->findGlobalPose(msg, *mapEditor->getMap());
    if (!gp.node) {
        ROS_WARN("Could not find node for given pose, ignoring");
        return;
    }
    TopoMap::Node* node = mapEditor->getMap()->getNode(gp.node->getName());

    if (vizState == CREATING_EDGE_APPROACH) {
        lastApproachPose = pose;
        lastApproachNode = node;
        vizState = CREATING_EDGE_EXIT;
        ROS_INFO_STREAM("Got approach pose (on " << node->getName() << "), waiting exit pose");
    } else if (vizState == CREATING_EDGE_EXIT) {
        std::string nodeName = edgeType + "_" + std::to_string(mapEditor->getMap()->getNumEdges()) +
                               "_" + lastApproachNode->getName() + "_" + node->getName();
        auto edge = mapEditor->getMap()->addEdge(lastApproachNode, node, nodeName, edgeType);
        setApproachData(edge, ApproachLineArea::create(
                                  lastApproachPose.getOrigin(), lastApproachPose.getOrigin(),
                                  tf2::Stamped<tf2::Quaternion>(
                                      lastApproachPose.getRotation(), lastApproachPose.stamp_,
                                      lastApproachPose.frame_id_))); // FIXME usage of flat planner

        setExitData(edge,
                    ApproachLineArea::create(pose.getOrigin(), pose.getOrigin(),
                                             tf2::Stamped<tf2::Quaternion>(
                                                 pose.getRotation(), pose.stamp_, pose.frame_id_)));

        // if frames are different, default direction is UP
        EdgeStepData::StepType stepType =
            (pose.frame_id_ == lastApproachPose.frame_id_ &&
             pose.getOrigin().getZ() < lastApproachPose.getOrigin().getZ())
                ? EdgeStepData::DOWN
                : EdgeStepData::UP;
        initEdgeData(edge, selectedStepName, stepType);

        mapEditor->deactivateEditorModule(this);
        _context.mapChanged();

        vizState = IDLE;
        ROS_INFO_STREAM("Got exit pose (on " << node->getName() << "), edge created, finished");
    }
}

void StepEdgeModule::createEdgeCB(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
    vizState = WAITING_STEP_SELECTION;
    ROS_INFO_STREAM("Creating edge type " << edgeType << ", waiting step selection");

    mapEditor->activateEditorModule(this);
}

void StepEdgeModule::createMapStepCB(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
    vizState = CREATING_MAP_STEP;
    ROS_INFO_STREAM("Creating Map Step, waiting pose");

    mapEditor->activateEditorModule(this);
}

YAML::Node StepEdgeModule::beginSaving(TopoMap* map) {
    const MapStepData& stepData = getMapData(mapEditor->getMap());

    YAML::Node root;

    StepEdgeModuleCostsParser parser;
    root["default_costs_down"] = parser.storeAll(stepData.defaultDownCosts);
    root["default_costs_up"] = parser.storeAll(stepData.defaultUpCosts);
    root["default_costs_cross_over"] = parser.storeAll(stepData.defaultCrossOverCosts);
    root["default_costs_cross_gravel"] = parser.storeAll(stepData.defaultCrossGravelCosts);

    YAML::Node steps(YAML::NodeType::Sequence);
    for (auto& e : stepData.steps) {
        YAML::Node step;
        const auto& stepInfo = *e.second;

        step["name"] = stepInfo.name;
        step["start"] = (tf2::Vector3&)stepInfo.start;
        step["end"] = (tf2::Vector3&)stepInfo.end;
        step["frame_id"] = stepInfo.start.frame_id_;

        parser.storeDiff(step, "costs_up", stepInfo.upCosts, stepData.defaultUpCosts);
        parser.storeDiff(step, "costs_down", stepInfo.downCosts, stepData.defaultDownCosts);
        parser.storeDiff(step, "costs_cross_over", stepInfo.crossOverCosts,
                         stepData.defaultCrossOverCosts);
        parser.storeDiff(step, "costs_cross_gravel", stepInfo.crossGravelCosts,
                         stepData.defaultCrossGravelCosts);

        steps.push_back(step);
    }
    root[edgeType] = steps;

    return root;
}

YAML::Node StepEdgeModule::saveEdgeData(TopoMap::Edge const* edge) {
    EdgeStepData const& data = getEdgeData(edge);

    YAML::Node root;

    root["step_name"] = data.stepName;

    static const std::map<EdgeStepData::StepType, std::string> step_type_names{
        {EdgeStepData::UP, "up"},
        {EdgeStepData::DOWN, "down"},
        {EdgeStepData::CROSS_OVER, "cross_over"},
        {EdgeStepData::CROSS_GRAVEL, "cross_gravel"}};

    root["step_type"] = step_type_names.at(data.type);

    return root;
}

void StepEdgeModule::setEdgeUpDown(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback,
    const TopoMap::Edge* constEdge) {
    ROS_INFO_STREAM("Setting step type of edge " << constEdge->getName());

    TopoMap::Edge* edge = mapEditor->getMap()->getEdge(constEdge->getName());
    if (edge) {
        EdgeStepData& data = getEdgeData(edge);
        if (feedback->menu_entry_id == 1) { // UP, FIXME
            data.type = EdgeStepData::UP;
        } else if (feedback->menu_entry_id == 2) // down
            data.type = EdgeStepData::DOWN;
        else if (feedback->menu_entry_id == 3)
            data.type = EdgeStepData::CROSS_OVER;
        else if (feedback->menu_entry_id == 4)
            data.type = EdgeStepData::CROSS_GRAVEL;

        mapEditor->displayMap();
    }
}

bool StepEdgeModule::areEdgesCoupled(const TopoMap::Edge* edgeA, const TopoMap::Edge* edgeB) {
    if (edgeA->getTransitionType() != edgeType || edgeB->getTransitionType() != edgeType)
        return false;

    EdgeStepData const& dataA = getEdgeData(edgeA);
    EdgeStepData const& dataB = getEdgeData(edgeB);

    return dataA.stepName == dataB.stepName;
}

static visualization_msgs::InteractiveMarker
createEdgeTipMarker(tf2::Stamped<tf2::Vector3> pos, std::string name, std::string name_end) {
    geometry_msgs::PoseStamped poseStart;
    tf2::toMsg(pos, poseStart.pose.position);
    poseStart.header.stamp = ros::Time::now();
    poseStart.header.frame_id = pos.frame_id_;

    visualization_msgs::InteractiveMarker int_marker;

    int_marker.header = poseStart.header;
    int_marker.name = name + name_end;
    int_marker.scale = 1;
    int_marker.pose = poseStart.pose;

    visualization_msgs::InteractiveMarkerControl control;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
    // controlStart.orientation_mode = visualization_msgs::InteractiveMarkerControl::VIEW_FACING;
    control.always_visible = true;
    control.name = name;
    tf2::Quaternion rotY(tf2::Vector3(0, 1, 0), M_PI_2);
    control.orientation = tf2::toMsg(rotY);

    visualization_msgs::Marker marker;
    marker.header = poseStart.header;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = poseStart.pose;
    marker.color.r = 1;
    marker.color.g = 0.5;
    marker.color.b = 0;
    marker.color.a = 1;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    control.markers.push_back(marker);
    int_marker.controls.push_back(control);

    return int_marker;
}

static visualization_msgs::InteractiveMarker
createStepConnectionMarker(StepEdgeModule::StepInfo* step, std::string name) {
    visualization_msgs::InteractiveMarker int_marker;

    int_marker.header.frame_id = step->start.frame_id_;
    int_marker.header.stamp = ros::Time::now();
    int_marker.name = name;
    int_marker.scale = 1;
    tf2::toMsg(step->start, int_marker.pose.position);
    int_marker.pose.orientation.w = 1;

    visualization_msgs::InteractiveMarkerControl control;
    control.name = step->name;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    control.always_visible = true;

    visualization_msgs::Marker marker;
    marker.header = int_marker.header;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = 1;
    marker.color.g = 0.5;
    marker.color.b = 0;
    marker.color.a = 1;
    marker.scale.x = 0.05; // shaft diameter
    marker.scale.y = 0.1;  // head diameter
    marker.scale.z = 0.15; // head length
    marker.points.resize(2);
    tf2::toMsg(step->start, marker.points[0]);
    tf2::toMsg(step->end, marker.points[1]);

    control.markers.push_back(marker);
    int_marker.controls.push_back(control);

    return int_marker;
}

void StepEdgeModule::visualizeMapStep(StepEdgeModule::StepInfo* stepInfo) {
    auto start = createEdgeTipMarker(stepInfo->start, stepInfo->name, "_start");
    auto end = createEdgeTipMarker(stepInfo->end, stepInfo->name, "_end");

    visualization_msgs::InteractiveMarkerControl control;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    control.always_visible = true;
    control.name = stepInfo->name + "b1";
    start.controls.push_back(control);
    control.name = stepInfo->name + "b2";
    end.controls.push_back(control);

    mapEditor->getMarkerServer().insert(
        start, boost::bind(&StepEdgeModule::processMapStepCB, this, _1, stepInfo, 0));
    mapEditor->getMarkerServer().insert(
        end, boost::bind(&StepEdgeModule::processMapStepCB, this, _1, stepInfo, 1));

    auto stepMarker = createStepConnectionMarker(stepInfo, stepInfo->name + "_line");
    mapEditor->getMarkerServer().insert(
        stepMarker, boost::bind(&StepEdgeModule::processMapStepCB, this, _1, stepInfo, -1));

    if (!stepInfo->menuHandler) {
        stepInfo->menuHandler = std::make_shared<interactive_markers::MenuHandler>();
        stepInfo->menuHandler->insert(
            "Delete this step", boost::bind(&StepEdgeModule::deleteStepCB, this, _1, stepInfo));
        stepInfo->menuHandler->insert(
            "Create both step edges",
            boost::bind(&StepEdgeModule::createBothStepEdgesCB, this, _1, stepInfo));
    }

    stepInfo->menuHandler->apply(mapEditor->getMarkerServer(), stepMarker.name);
}

void StepEdgeModule::processMapStepCB(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback,
    StepEdgeModule::StepInfo* stepInfo, int spot) {
    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK)
        return selectMapStepCB(feedback);
    else if (feedback->event_type != visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
        return;

    const auto transform = _context.tfBuffer->lookupTransform(
        stepInfo->start.frame_id_, feedback->header.frame_id, ros::Time(0));

    geometry_msgs::Pose poseTransformedMsg;
    tf2::doTransform(feedback->pose, poseTransformedMsg, transform);

    tf2::Transform poseTransformed;
    tf2::fromMsg(poseTransformedMsg, poseTransformed);

    if (spot == 0) { // start
        stepInfo->start.setData(poseTransformed.getOrigin());
    } else if (spot == 1) { // end
        stepInfo->end.setData(poseTransformed.getOrigin());
    } else
        return;

    _context.mapChanged();
}

auto static createQuaternionFromYaw(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return q;
}

void StepEdgeModule::setApproachFromStep(const TopoMap::Edge* constEdge) {
    TopoMap::Edge* edge = _context.topoMap->getEdge(constEdge->getName());

    auto const& approach = getApproachData(edge);
    EdgeStepData const& edgeData = getEdgeData(edge);
    MapStepData& stepData = getMapData(mapEditor->getMap());
    StepInfo const& stepInfo = *stepData.steps.at(edgeData.stepName);

    const double robotWidth2 = 0.4; // FIXME
    const double robotLength2 = 0.4;

    const auto diff =
        tf2::quatRotate(createQuaternionFromYaw(-M_PI_2), stepInfo.start - stepInfo.end);
    const double yaw = atan2(diff.y(), diff.x());

    tf2::Transform startTrans(createQuaternionFromYaw(yaw), stepInfo.start);

    auto approachCenter = approach->getCenter();
    tf2::Vector3 offset(robotLength2, -robotWidth2, 0);
    if (startTrans.inverse()(approachCenter.getOrigin()).x() < 0) { // in backward direction
        offset.setX(-offset.x());
    }

    auto start = stepInfo.start + tf2::quatRotate(startTrans.getRotation(), offset);
    offset.setY(robotWidth2);
    auto end = stepInfo.end + tf2::quatRotate(startTrans.getRotation(), offset);
    auto rot = startTrans.getRotation();
    if (offset.x() > 0)
        rot = rot * createQuaternionFromYaw(M_PI);

    auto approachLine = std::make_shared<ApproachLineArea>(
        start, end, tf2::Stamped<tf2::Quaternion>(rot, ros::Time::now(), approachCenter.frame_id_));
    setApproachData(edge, approachLine);

    onEdgeApproachChanged(edge); // set exit data and update node connections
    _context.mapChanged();
}

double StepEdgeModule::getHeuristics(TopoMap::Edge const* edge) {
    const MapStepData& mapData = getMapData(_context.topoMap.get());
    auto& data = getEdgeData(edge);
    auto& entry = *mapData.steps.at(data.stepName);

    return getCosts(selectCosts(entry, data.type));
}

std::string StepEdgeModule::getModuleType(const TopoMap::Edge* edge) {
    return className;
    return className;
}

void StepEdgeModule::parseEdgeInData(YAML::Node const& config, TopoMap::Edge* edge) {
    auto const& approach = getApproachData(edge);
    approach->getCenter();
    auto& data = getEdgeData(edge);
    auto exitPose =
        getExitPoseFromApproach(*currentStepData.steps.at(data.stepName), approach->getCenter());
    setExitData(edge, std::make_shared<FixedPosition>(exitPose));
}

tf2::Stamped<tf2::Transform>
StepEdgeModule::getExitPoseFromApproach(StepInfo const& stepInfo,
                                        tf2::Stamped<tf2::Transform> const& approachPose) {
    tf2::Stamped<tf2::Transform> approachTransformed =
        _context.tfBuffer->transform(approachPose, stepInfo.start.frame_id_);

    // mirror the approach point at step line
    tf2::Vector3 normal = (stepInfo.end - stepInfo.start).normalized();
    tf2::Vector3 on_line =
        stepInfo.start + normal * (approachTransformed.getOrigin() - stepInfo.start).dot(normal);
    tf2::Vector3 final = on_line + (on_line - approachTransformed.getOrigin());

    approachTransformed.setOrigin(final);
    return approachTransformed;
}

YAML::Node StepEdgeModule::saveEdgeInData(TopoMap::Edge const* edge) {
    YAML::Node root; // exit data is determined by approach only
    return root;
}

void StepEdgeModule::onEdgeApproachChanged(const TopoMap::Edge* constEdge) {
    TopoMap::Edge* edge = _context.topoMap->getEdge(constEdge->getName());
    auto const& approach = getApproachData(edge);

    {
        GlobalPosition aproachPosition =
            _context.poseService->findGlobalPose(approach->getCenter(), *_context.topoMap).first;
        if (aproachPosition.node) {
            edge->setSrc(_context.topoMap->getNode(aproachPosition.node->getName()));
        } else {
            ROS_WARN("StepEdgeModule::onEdgeApproachChanged(): edge approach center could not be "
                     "mapped to a node");
        }
    }

    EdgeStepData const& edgeData = getEdgeData(edge);
    MapStepData& stepData = getMapData(mapEditor->getMap());
    StepInfo const& stepInfo = *stepData.steps.at(edgeData.stepName);

    auto exitPose = getExitPoseFromApproach(stepInfo, approach->getCenter());
    setExitData(edge, std::make_shared<FixedPosition>(exitPose));

    {
        GlobalPosition exitPosition =
            _context.poseService->findGlobalPose(exitPose, *_context.topoMap).first;
        if (exitPosition.node) {
            edge->setDst(_context.topoMap->getNode(exitPosition.node->getName()));
        } else {
            ROS_WARN(
                "StepEdgeModule::onEdgeApproachChanged(): edge exit could not be mapped to a node");
        }
    }
}

void StepEdgeModule::deleteStepCB(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback,
    BaseTraits<StepEdgeModule>::StepInfo* stepInfo) {
    ROS_INFO_STREAM("StepEdgeModule: deleting step '" << stepInfo->name << "' and all its edges");

    // collect edges of this step
    std::vector<TopoMap::Edge*> edges;
    mapEditor->getMap()->foreachEdge([&](TopoMap::Edge* edge) {
        if (_context.isModuleType<StepEdgeModule>(edge)) {
            auto& edgeData = getEdgeData(edge);
            if (edgeData.stepName == stepInfo->name) {
                edges.push_back(edge);
            }
        }
    });

    // delete those edges
    for (auto edge : edges) {
        mapEditor->getMap()->removeEdge(edge);
    }

    // delete step
    auto& data = getMapData(mapEditor->getMap());
    data.steps.erase(stepInfo->name);

    _context.mapChanged();
}

void StepEdgeModule::createBothStepEdgesCB(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback, StepInfo* stepInfo) {

    createBothStepEdges(stepInfo);
}

void StepEdgeModule::createBothStepEdges(StepInfo* stepInfo) {
    static constexpr auto distFromStep = 0.4;

    // get approach and exit points based on map step
    const auto centerOffset =
        tf2::quatRotate(createQuaternionFromYaw(M_PI_2), stepInfo->end - stepInfo->start)
            .normalized() *
        distFromStep;

    const auto center = (stepInfo->start + stepInfo->end) * 0.5;
    const auto posA = center + centerOffset;
    const auto posB = center - centerOffset;

    const double yaw = atan2(centerOffset.y(), centerOffset.x());
    const auto rotBA = tf2::Stamped<tf2::Quaternion>{createQuaternionFromYaw(yaw), ros::Time::now(),
                                                     _context.globalFrame};
    const auto rotAB = tf2::Stamped<tf2::Quaternion>{createQuaternionFromYaw(yaw + M_PI),
                                                     ros::Time::now(), _context.globalFrame};

    // get nodes of A and B
    tf2::Stamped<tf2::Transform> stampedPoseA;
    GlobalPosition gpA;
    std::tie(gpA, stampedPoseA) = _context.poseService->findGlobalPose(
        tf2::Stamped<tf2::Transform>{tf2::Transform{rotAB, posA}, ros::Time::now(),
                                     _context.globalFrame},
        *mapEditor->getMap());
    if (!gpA.node) {
        ROS_WARN("Could not find node pose A for creating edge from map step, returning.");
        return;
    }
    TopoMap::Node* nodeA = mapEditor->getMap()->getNode(gpA.node->getName());

    tf2::Stamped<tf2::Transform> stampedPoseB;
    GlobalPosition gpB;
    std::tie(gpB, stampedPoseB) = _context.poseService->findGlobalPose(
        tf2::Stamped<tf2::Transform>{tf2::Transform{rotBA, posB}, ros::Time::now(),
                                     _context.globalFrame},
        *mapEditor->getMap());
    if (!gpB.node) {
        ROS_WARN("Could not find node pose B for creating edge from map step, returning.");
        return;
    }
    TopoMap::Node* nodeB = mapEditor->getMap()->getNode(gpB.node->getName());

    {
        const auto nameAB = edgeType + "_" + std::to_string(mapEditor->getMap()->getNumEdges()) +
                            "_" + gpA.node->getName() + "_" + gpB.node->getName();
        auto edgeAB = mapEditor->getMap()->addEdge(nodeA, nodeB, nameAB, edgeType);
        setApproachData(edgeAB, ApproachLineArea::create(posA, posA, rotAB));
        setExitData(edgeAB, ApproachLineArea::create(posB, posB, rotAB));
        initEdgeData(edgeAB, stepInfo->name, EdgeStepData::StepType::CROSS_OVER);
    }

    {
        const auto nameBA = edgeType + "_" + std::to_string(mapEditor->getMap()->getNumEdges()) +
                            "_" + gpB.node->getName() + "_" + gpA.node->getName();
        auto edgeBA = mapEditor->getMap()->addEdge(nodeB, nodeA, nameBA, edgeType);
        setApproachData(edgeBA, ApproachLineArea::create(posB, posB, rotBA));
        setExitData(edgeBA, ApproachLineArea::create(posA, posA, rotBA));
        initEdgeData(edgeBA, stepInfo->name, EdgeStepData::StepType::CROSS_OVER);
    }

    _context.mapChanged();
}

double StepEdgeModule::getCosts(const BaseTraits<StepEdgeModule>::CostSettings& settings) {
    std::map<std::string, double> costs;
    for (auto& varname : costs_comp_.getVariables()) {
        double cost;
        if (varname == CommonCostProfiles::TIME_COSTS ||
            varname == CommonCostProfiles::DEFAULT_PROFILE) {
            cost = settings.time_costs;
        } else if (varname == CommonCostProfiles::ENERGY_COSTS) {
            cost = settings.energy_costs;
        } else if (varname == CommonCostProfiles::DISTANCE_COSTS) {
            cost = settings.distance_costs;
        } else {
            cost = 1;
            ROS_WARN_STREAM(className << " error: unsupported cost profile variable '" << varname
                                      << "', using distance as costs.");
        }

        costs[varname] = cost;
    }

    return costs_comp_.computeCosts(costs, className);
}

} // namespace toponav_ros
