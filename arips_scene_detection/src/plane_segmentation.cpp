#include "plane_segmentation.h"

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/sac_model_plane.h>

PlaneSegmentationResult segmentPlane(pcl::PointCloud<pcl::PointXYZ>::ConstPtr const& input,
                                      PlaneSegementationParameters const& params) {
    PlaneSegmentationResult result;

    pcl::ModelCoefficients coefficients;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (params.distThreshold);
    seg.setInputCloud (input);
    seg.segment (*inliers, coefficients);
    if(coefficients.values.size() == 4) {
        result.direction = tf2::Vector3{coefficients.values.at(0),
                                             coefficients.values.at(1),
                                             coefficients.values.at(2)};
        result.distance = -coefficients.values.at(3);
    } else {
        result.direction.setZero();
        result.distance = 0;
    }

    result.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*result.cloud);

    result.nonCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setNegative(true);
    extract.filter(*result.nonCloud);

    return result;
}