#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf2/LinearMath/Vector3.h>
#include <pcl/ModelCoefficients.h>

struct PlaneSegementationParameters {
    const float distThreshold = 0.01;
};

struct PlaneSegmentationResult {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr nonCloud;

    pcl::ModelCoefficients modelCoefficients;

    tf2::Vector3 direction; // will be 0,0,0 if no plane found
    float distance;
};

PlaneSegmentationResult segmentPlane(pcl::PointCloud<pcl::PointXYZ>::ConstPtr const& input,
                        PlaneSegementationParameters const& params = PlaneSegementationParameters{});
