#pragma once

#include <tf2/LinearMath/Transform.h>
#include <opencv2/core/core.hpp>
#include <visualization_msgs/MarkerArray.h>

#include "plane_segmentation.h"

struct ObjectSegmentationInput {
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointcloud;
  pcl::ModelCoefficients modelCoefficients;

  const cv::Mat& depthImage;

  int minObjectWidth = 5;
  int maxObjectWidth = 50;

  int minObjectHeight = 5;
  int maxObjectHeight = 30;
};

struct ObjectInformation {
  tf2::Transform position;
  tf2::Vector3 meanColor;
};

struct ObjectSegmentationOutput {
  std::vector<ObjectInformation> detectedObjects;
};

// will put some visualization hints into markerArray if not null
ObjectSegmentationOutput detectObjectsInScene(const ObjectSegmentationInput&, visualization_msgs::MarkerArray* markerArray= nullptr);
