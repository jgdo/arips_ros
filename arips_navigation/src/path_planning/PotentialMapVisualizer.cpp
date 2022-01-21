#include <arips_navigation/path_planning/PotentialMapVisualizer.h>

#include <visualization_msgs/MarkerArray.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <nav_msgs/Path.h>

PotentialMapVisualizer::PotentialMapVisualizer() {
    ros::NodeHandle nh("~");
    mPub = nh.advertise<sensor_msgs::PointCloud2>("potential_map", 1, true);
    mPotentialGradPub = nh.advertise<visualization_msgs::MarkerArray>("gradient_map", 1);
    mPathPub = nh.advertise<nav_msgs::Path>("current_path", 1);
}

void PotentialMapVisualizer::showGradients(const PotentialMap& map, const Pose2D& robotPose) {
    if(mPotentialGradPub.getNumSubscribers() == 0) {
        return;
    }

    const auto index = map.toMap(robotPose.point);
    if(!index) {
        return;
    }

    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker marker;
    marker.header.frame_id = map.frameId();
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

    for (int x = index->x() - 30; x < index->x() + 30; x++) {
        for (int y = index->y() - 30; y < index->y() + 30; y++) {
            const auto optGrad = map.getGradient({x, y});
            if (!optGrad) {
                continue;
            }

            const auto v = map.toWorld({x, y});
            geometry_msgs::Point p;
            p.x = v.x();
            p.y = v.y();
            marker.points.push_back(p);

            p.x += cos(*optGrad) * map.resolution();
            p.y += sin(*optGrad) * map.resolution();

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
    cost_cloud.header.frame_id = map.frameId();
    cost_cloud.header.stamp = ros::Time::now();

    sensor_msgs::PointCloud2Modifier cloud_mod(cost_cloud);
    // clang-format off
    cloud_mod.setPointCloud2Fields(4,
                                   "x", 1, sensor_msgs::PointField::FLOAT32,
                                   "y", 1, sensor_msgs::PointField::FLOAT32,
                                   "z", 1, sensor_msgs::PointField::FLOAT32,
                                   "goal_cost", 1, sensor_msgs::PointField::FLOAT32 /* TODO,
                                   "costmap", 1, sensor_msgs::PointField::FLOAT32 */);
    // clang-format on

    unsigned int x_size = map.width();
    unsigned int y_size = map.height();
    double z_coord = 0.0;

    cloud_mod.resize(x_size * y_size);
    sensor_msgs::PointCloud2Iterator<float> iter_x(cost_cloud, "x");

    size_t pointCount = 0;
    for (unsigned int cx = 0; cx < x_size; cx++) {
        for (unsigned int cy = 0; cy < y_size; cy++) {
            const auto w = map.toWorld({cx, cy});
            const auto goalDist = map.at({cx, cy});
            if (goalDist) {
                iter_x[0] = w.x();
                iter_x[1] = w.y();
                iter_x[2] = 0;
                iter_x[3] = *goalDist;
                // TODOiter_x[4] = map.at->getCost(cx, cy);
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
    showPath(map, robotPose);
}

void PotentialMapVisualizer::showPath(const PotentialMap& map, const Pose2D& robotPose) {
    if(mPathPub.getNumSubscribers() == 0) {
        return;
    }

    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = map.frameId();

    const auto path2d = map.traceCurrentPath(robotPose);
    geometry_msgs::PoseStamped pathPoint;
    pathPoint.header = path.header;

    for(const auto& p2d: path2d) {
        pathPoint.pose = p2d.toPoseMsg();
        // header stays same
        path.poses.emplace_back(pathPoint);
    }

    mPathPub.publish(path);
}
