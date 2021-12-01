#include <arips_navigation/path_planning/PotentialMapVisualizer.h>

#include <visualization_msgs/MarkerArray.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

PotentialMapVisualizer::PotentialMapVisualizer() {
    ros::NodeHandle nh("~");
    mPub = nh.advertise<sensor_msgs::PointCloud2>("potential_map", 1, true);
    mPotentialGradPub = nh.advertise<visualization_msgs::MarkerArray>("gradient_map", 1);
}

void PotentialMapVisualizer::showGradients(const PotentialMap& map, const Pose2D& robotPose) {
    if(mPotentialGradPub.getNumSubscribers() == 0) {
        return;
    }

    const auto* costmap = map.costmap().getCostmap();

    unsigned int cx, cy;
    if (!costmap->worldToMap(robotPose.x(), robotPose.y(), cx, cy)) {
        return;
    }

    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker marker;
    marker.header.frame_id = map.costmap().getGlobalFrameID();
    marker.header.stamp = ros::Time();
    marker.ns = "potential";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.001;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;

    for (int x = cx - 30; x < cx + 30; x++) {
        for (int y = cy - 30; y < cy + 30; y++) {
            const auto optGrad = map.getGradient({x, y});
            if (!optGrad) {
                continue;
            }

            double wx, wy;
            costmap->mapToWorld(x, y, wx, wy);
            geometry_msgs::Point p;
            p.x = wx;
            p.y = wy;
            marker.points.push_back(p);

            p.x += cos(*optGrad) * costmap->getResolution();
            p.y += sin(*optGrad) * costmap->getResolution();

            marker.points.push_back(p);
        }
    }

    markerArray.markers.push_back(marker);
    mPotentialGradPub.publish(markerArray);
}

void PotentialMapVisualizer::showPotential(const PotentialMap& map) {
    if(mPub.getNumSubscribers() == 0) {
        return;
    }

    sensor_msgs::PointCloud2 cost_cloud;
    cost_cloud.header.frame_id = map.getCostmapRos().getGlobalFrameID();
    cost_cloud.header.stamp = ros::Time::now();

    sensor_msgs::PointCloud2Modifier cloud_mod(cost_cloud);
    // clang-format off
    cloud_mod.setPointCloud2Fields(5,
                                   "x", 1, sensor_msgs::PointField::FLOAT32,
                                   "y", 1, sensor_msgs::PointField::FLOAT32,
                                   "z", 1, sensor_msgs::PointField::FLOAT32,
                                   "goal_cost", 1, sensor_msgs::PointField::FLOAT32,
                                   "costmap", 1, sensor_msgs::PointField::FLOAT32);
    // clang-format on

    const auto* costmap_p_ = map.getCostmapRos().getCostmap();
    unsigned int x_size = costmap_p_->getSizeInCellsX();
    unsigned int y_size = costmap_p_->getSizeInCellsY();
    double z_coord = 0.0;
    double x_coord, y_coord;

    cloud_mod.resize(x_size * y_size);
    sensor_msgs::PointCloud2Iterator<float> iter_x(cost_cloud, "x");

    size_t pointCount = 0;
    for (unsigned int cx = 0; cx < x_size; cx++) {
        for (unsigned int cy = 0; cy < y_size; cy++) {
            costmap_p_->mapToWorld(cx, cy, x_coord, y_coord);
            const auto goalDist = map.getGoalDistance(cx, cy);
            if (goalDist) {
                iter_x[0] = x_coord;
                iter_x[1] = y_coord;
                iter_x[2] = *goalDist;
                iter_x[3] = *goalDist;
                iter_x[4] = costmap_p_->getCost(cx, cy);
                ++iter_x;
                pointCount++;
            }
        }
    }

    cloud_mod.resize(pointCount);
    mPub.publish(cost_cloud);
}

void PotentialMapVisualizer::showMap(const PotentialMap& map, const Pose2D& robotPose) {
    showPotential(map);
    showGradients(map, robotPose);
}
