//
// Created by jgdo on 10/26/20.
//

#ifndef ARIPS_SCENE_DETECTION_GROUND_SEGMENTATION_H
#define ARIPS_SCENE_DETECTION_GROUND_SEGMENTATION_H

#include <opencv2/core/core.hpp>
#include <image_geometry/stereo_camera_model.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct GroundPlaneFilterResult {
    pcl::PointCloud<pcl::PointXYZ> planeCloud;
    pcl::PointCloud<pcl::PointXYZ> nonPlaneCloud;

    std::vector<pcl::PointXYZ> objectPoints;
};

bool extractGroundPlane(cv::Mat img, const image_geometry::PinholeCameraModel& model, GroundPlaneFilterResult& result);

#endif //ARIPS_SCENE_DETECTION_GROUND_SEGMENTATION_H
