#pragma once

#include <tf2/LinearMath/Transform.h>
#include <opencv2/core/core.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/transform_datatypes.h>

#include "plane_segmentation.h"

struct ObjectSegmentationInput
{
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointcloud;
  pcl::ModelCoefficients modelCoefficients;

  const cv::Mat& depthImage;
  const cv::Mat& colorImage;
  const tf2::Transform& cameraToBase;

  const float maxRadiusAroundBase = 0.3;
  const float maxCenterHeightAboveFloor = 0.10;

  int minObjectWidth = 8;
  int maxObjectWidth = 50;

  int minObjectHeight = 8;
  int maxObjectHeight = 50;
};

struct ObjectInformation
{
  tf2::Stamped<tf2::Transform> position;
  std_msgs::ColorRGBA meanColor;
};

struct ObjectSegmentationOutput
{
  std::vector<ObjectInformation> detectedObjects;
};

// will put some visualization hints into markerArray if not null
ObjectSegmentationOutput detectObjectsInScene(
    const ObjectSegmentationInput&, visualization_msgs::MarkerArray* markerArray = nullptr);
