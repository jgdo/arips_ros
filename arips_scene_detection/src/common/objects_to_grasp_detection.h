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

  float minObjectArea = 0.02*0.02;
  float maxObjectArea = 0.05*0.05;
};

struct ObjectInformation
{
  tf2::Stamped<tf2::Transform> position;
  tf2::Vector3 size; // x is the longer side, y is shorter side, z is height
  std_msgs::ColorRGBA meanColor;
};

struct ObjectSegmentationOutput
{
  std::vector<ObjectInformation> detectedObjects;
};

// will put some visualization hints into markerArray if not null
ObjectSegmentationOutput detectObjectsInScene(
    const ObjectSegmentationInput&, visualization_msgs::MarkerArray* markerArray = nullptr);
