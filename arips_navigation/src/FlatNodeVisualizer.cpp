#include <arips_navigation/FlatGroundModule.h>
#include <arips_navigation/FlatNodeVisualizer.h>

#include <arips_navigation/utils/ApproachLineArea.h>
#include <geometry_msgs/PoseStamped.h>

#include <toponav_ros/MapEditor.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace toponav_ros {

using namespace toponav_core;

FlatNodeVisualizer::FlatNodeVisualizer(PlanningContext &context, std::string const &nodeType)
    : _context(context), own_node_type_(nodeType) {
    ros::NodeHandle nh;
    m_nodeMarkerPub = nh.advertise<visualization_msgs::Marker>("node_segment_markers", 10, true);
}

void FlatNodeVisualizer::initializeVisualization(MapEditor *editor) {
    NodeVisualizationInterface::initializeVisualization(editor);

    auto segmentHandle = mapEditor->getMenuHandler().insert(mapEditor->getOtherMenuHandle(),
                                                            "Auto-segment flat layer");
    mapEditor->getMenuHandler().insert(mapEditor->getOtherMenuHandle(),
                                       "Add flat node at position...",
                                       boost::bind(&FlatNodeVisualizer::addNodeFeedback, this, _1));

    for (auto &entry : FlatGroundModule::getMapData(mapEditor->getMap())) {
        mapEditor->getMenuHandler().insert(
            segmentHandle, entry.first,
            boost::bind(&FlatNodeVisualizer::segmentLayerFeedback, this, _1, entry.first));
    }
}

void FlatNodeVisualizer::appendRegionEdgeToPlan(nav_msgs::Path *pathPtr, std::string regionType,
                                                boost::any const &pathData) {
    const std::vector<geometry_msgs::PoseStamped> *plan =
        boost::any_cast<std::vector<geometry_msgs::PoseStamped>>(&pathData);
    pathPtr->poses.insert(pathPtr->poses.end(), plan->begin(), plan->end());

    //	if(plan && plan->size() > 0) {
    //		visualization_msgs::Marker marker;
    //
    //		marker.type = visualization_msgs::Marker::LINE_STRIP;
    //		marker.header.frame_id = plan->at(0).header.frame_id;
    //		marker.header.stamp = ros::Time::now();
    //		marker.action = visualization_msgs::Marker::ADD;
    //		marker.points.reserve(plan->size());
    //		marker.scale.x = 0.1;
    //		marker.ns = "FlatNodeVisualizer";
    //		marker.id = pathPtr->markers.size();
    //		marker.color.a = 1;
    //		marker.color.r = 1;
    //		marker.color.g = 0;
    //		marker.color.b = 0;
    //
    //		for(auto& poseConst: *plan) {
    //			geometry_msgs::PoseStamped pose = poseConst;
    //			pose.header.stamp = ros::Time::now(); // TODO better solution?
    //
    //			geometry_msgs::PoseStamped poseOut;
    //			tf.transformPose(marker.header.frame_id, pose, poseOut);
    //			marker.points.push_back(pose.pose.position);
    //		}
    //
    //		pathPtr->markers.push_back(marker);
    //	}
}

void FlatNodeVisualizer::vizualizeNode(const TopoMap::Node *node) {
    if (m_nodeMarkerPub.getNumSubscribers() == 0)
        return;

    // Visualize flat costmap segments with different colors

    const FlatGroundModule::NodeData &data = FlatGroundModule::getNodeData(node);
    const float res = data.planner->getMap().getCostmap()->getResolution();

    visualization_msgs::Marker nodeMarkers;
    nodeMarkers.header.stamp = ros::Time::now();
    nodeMarkers.header.frame_id = data.planner->getMap().getGlobalFrameID();
    nodeMarkers.type = visualization_msgs::Marker::POINTS;
    nodeMarkers.action = visualization_msgs::Marker::ADD;
    nodeMarkers.ns = "topo_node";
    nodeMarkers.scale.x = res;
    nodeMarkers.scale.y = res;

    using Color = std::array<uint8_t, 3>;
    const std::vector<Color> allColors = {
        {255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}, {255, 0, 255}, {0, 255, 255},
    };

    std::map<const TopoMap::Node *, Color> nodeColors;

    const double x_off = data.planner->getMap().getCostmap()->getOriginX();
    const double y_off = data.planner->getMap().getCostmap()->getOriginY();

    const FlatGroundModule::NodeMatrix &mat = (*data.nodeMatrix);
    for (int x = 0; x < mat.rows(); x++) { // rows = x coordinate, see regionGrow() impl
        for (int y = 0; y < mat.cols(); y++) {
            auto n = mat(x, y);
            if (!n) {
                continue;
            }

            geometry_msgs::Point p;
            p.x = x * res + x_off;
            p.y = y * res + y_off;

            std_msgs::ColorRGBA colorMsg;

            auto colorIter = nodeColors.find(n);
            if (colorIter == nodeColors.end()) {
                // if no color is set for this node yet, pick next color or generate a random one
                const Color newColor =
                    nodeColors.size() < allColors.size()
                        ? allColors.at(nodeColors.size())
                        : Color{(uint8_t)rand(), (uint8_t)rand(), (uint8_t)rand()};

                colorIter = nodeColors.insert({n, newColor}).first;
            }

            const auto color = colorIter->second;
            colorMsg.r = color[0] / 255.0;
            colorMsg.g = color[1] / 255.0;
            colorMsg.b = color[2] / 255.0;
            colorMsg.a = 1;

            nodeMarkers.points.push_back(p);
            nodeMarkers.colors.push_back(colorMsg);
        }
    }

    m_nodeMarkerPub.publish(nodeMarkers);
}

void FlatNodeVisualizer::segmentLayerFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, std::string mapName) {
    FlatGroundModule::segmentAllNodes(mapEditor->getMap(), mapName, own_node_type_,
                                      50); // FIXME hardcoded number
}
void FlatNodeVisualizer::addNodeFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {

    mapEditor->activateEditorModule(this);

    ROS_INFO_STREAM("Waiting node pose selection");
}
void FlatNodeVisualizer::activate() {}

void FlatNodeVisualizer::deactivate() {}

void FlatNodeVisualizer::poseCallback(const geometry_msgs::PoseStamped &msg) {
    tf2::Stamped<tf2::Transform> pose;
    tf2::fromMsg(msg, pose);

    const size_t minNumCells = 50; // TODO Hardcoded

    if (pose.frame_id_.empty()) {
        ROS_ERROR("FlatNodeVisualizer::poseCallback(): pose cannot have an empty frame_id");
        mapEditor->deactivateEditorModule(this);
        return;
    }

    auto map = mapEditor->getMap();

    for (auto &mapEntry : NodeModuleHelperBase<FlatGroundModule>::getMapData(map)) {
        const auto &costmap_ros = mapEntry.second.planner->getMap();

        tf2::Stamped<tf2::Transform> localPose;
        try {
            localPose = _context.tfBuffer->transform(pose, costmap_ros.getGlobalFrameID());
        } catch (const tf2::TransformException &ex) {
            ROS_WARN("FlatNodeVisualizer::poseCallback(): %s", ex.what());
            continue;
        }

        auto &mat = *mapEntry.second.nodeMatrix;

        if (std::abs(localPose.getOrigin().z()) <
            0.5) { // FIXME: better check if this is the costmap
            unsigned int mx, my;

            // returns false if outside costmap bounds
            if (costmap_ros.getCostmap()->worldToMap(localPose.getOrigin().x(),
                                                     localPose.getOrigin().y(), mx, my)) {
                ROS_INFO_STREAM("Checking flat pose at xy: " << mx << ", " << my);
                TopoMap::Node const *realNode = mat(mx, my);
                if (realNode) {
                    ROS_ERROR_STREAM(
                        "There is already a flat node at given pose, cannnot create new node");
                    mapEditor->deactivateEditorModule(this);
                    return;
                }

                // find free node name
                std::string name;
                size_t nodeCount = map->getNumNodes();
                do {
                    name = mapEntry.first + "_node_" + std::to_string(nodeCount++);
                } while (map->getNode(name));

                TopoMap::Node *node = map->addNode(name, own_node_type_);

                BaseTraits<FlatGroundModule>::NodeMatrix local_mat =
                    BaseTraits<FlatGroundModule>::NodeMatrix::Constant(mat.rows(), mat.cols(),
                                                                       nullptr);

                size_t numCells = FlatGroundModule::regionGrow(node, costmap_ros.getCostmap(), mx,
                                                               my, &local_mat);

                if (minNumCells == 0 || numCells >= minNumCells) {
                    FlatGroundModule::combineNodeMatrix(mat, local_mat); // merge both matrices

                    FlatGroundModule::initNodeData(node, mapEntry.second.planner,
                                                   mapEntry.second.nodeMatrix,
                                                   mapEntry.second.tfBuffer, mx, mx, mapEntry.first,
                                                   0); // default height is 0
                } else {                               // area has to few cells, ignore
                    map->removeNode(node);
                    mapEditor->deactivateEditorModule(this);

                    ROS_WARN_STREAM(
                        "Could not create new flat node at give pose, since regionGrow found only "
                        << numCells << " cells");
                    return;
                }
            }
        }
    }

    _context.mapChanged();
    ROS_INFO_STREAM("Created flat node");
    mapEditor->deactivateEditorModule(this);
}

} // namespace toponav_ros
