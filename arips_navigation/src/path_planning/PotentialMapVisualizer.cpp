#include <arips_navigation/path_planning/PotentialMapVisualizer.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

PotentialMapVisualizer::PotentialMapVisualizer(const std::string& name) {
    ros::NodeHandle nh("~");
    mPub = nh.advertise<sensor_msgs::PointCloud2>(name, 1, true);
}

void PotentialMapVisualizer::visualizeMap(const PotentialMap& map) {
    if(mPub.getNumSubscribers() == 0) {
        return;
    }

    sensor_msgs::PointCloud2 cost_cloud;
    cost_cloud.header.frame_id = map.getCostmapRos().getGlobalFrameID();
    cost_cloud.header.stamp = ros::Time::now();

    sensor_msgs::PointCloud2Modifier cloud_mod(cost_cloud);
    // clang-format off
    cloud_mod.setPointCloud2Fields(4,
                                   "x", 1, sensor_msgs::PointField::FLOAT32,
                                   "y", 1, sensor_msgs::PointField::FLOAT32,
                                   "z", 1, sensor_msgs::PointField::FLOAT32,
                                   "goal_cost", 1, sensor_msgs::PointField::FLOAT32);
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
            const double goalDist = map.goalDistance(cx, cy);
            if (goalDist >= 0) {
                iter_x[0] = x_coord;
                iter_x[1] = y_coord;
                iter_x[2] = goalDist;
                iter_x[3] = goalDist;
                ++iter_x;
                pointCount++;
            }
        }
    }

    cloud_mod.resize(pointCount);
    mPub.publish(cost_cloud);
}
