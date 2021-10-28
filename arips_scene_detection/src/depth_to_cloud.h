#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/mat.hpp>
#include <image_geometry/pinhole_camera_model.h>

/**
 * Convert depth image to cloud
 *
 * @param depthImage
 * @param model
 * @param pixelInc
 * @return null on error
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr depthToCloud(const cv::Mat &depthImage, std_msgs::Header const& depthHeader,
                                                 const image_geometry::PinholeCameraModel &model, int pixelInc, bool forceStructured = false);
