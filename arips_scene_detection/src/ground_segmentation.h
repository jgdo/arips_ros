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

    std::vector<std::vector<int>> allObjectIndices; // set of indices per object
    std::vector<float> planeCoefficients;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFull;

    std::string frame_id;
};

struct segmentationInput {
    const cv::Mat& depthImage;
    const image_geometry::PinholeCameraModel& model;
    // if true, ignore non-plane points which are farther away than the plane itself,
    // i.e. keep only points which are between camera and plane
    bool nearerOnly;

    int minObjectWidth = 5;
    int maxObjectWidth = 50;

    int minObjectHeight = 5;
    int maxObjectHeight = 30;
};

bool extractGroundPlane(const segmentationInput& input, GroundPlaneFilterResult& result);

Eigen::Vector4f getPlaneCoefficientsToCamera(const std::vector<float>& coefficients);

#endif //ARIPS_SCENE_DETECTION_GROUND_SEGMENTATION_H
