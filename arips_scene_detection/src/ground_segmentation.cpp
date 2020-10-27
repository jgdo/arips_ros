//
// Created by jgdo on 10/26/20.
//

#include <ros/ros.h>

#include "ground_segmentation.h"

#include <depth_image_proc/depth_conversions.h>


#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <opencv2/highgui.hpp>

template<class T>
struct DepthTo3d {
    DepthTo3d(const image_geometry::PinholeCameraModel& model):
            center_x(model.cx()),
            center_y(model.cy()),
            unit_scaling(depth_image_proc::DepthTraits<T>::toMeters( T(1) )),
            constant_x(unit_scaling / model.fx()),
            constant_y(unit_scaling / model.fy())
    {
    }

    template<class P>
    inline P getPoint(int x, int y, T depth) {
        return P {
                (x - center_x) * depth * constant_x,
                (y - center_y) * depth * constant_y,
                depth_image_proc::DepthTraits<T>::toMeters(depth)
            };
    }

    // Use correct principal point from calibration
    const float center_x;
    const float center_y;

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    const double unit_scaling;
    const float constant_x;
    const float constant_y;
};

/**
 * Convert a depth image to a structured XYZ point cloud. Skip every pixel_inc pixel.
 *
 * @tparam T depth type (float or uint16)
 * @param img
 * @param model
 * @param pixel_inc
 * @return
 */
template<class T>
pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthToPointcloud(cv::Mat img, const image_geometry::PinholeCameraModel& model, int pixel_inc) {
    assert(pixel_inc > 0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->reserve(cloud->width * cloud->height);
    cloud->is_dense = false; // points might be invalid

    DepthTo3d<T> d3d {model};

    const float bad_float = std::numeric_limits<float>::quiet_NaN();
    const pcl::PointXYZ bad_point {bad_float, bad_float, bad_float};

    for(int y = pixel_inc/2; y < img.rows; y+=pixel_inc) {
        for(int x = pixel_inc/2; x < img.cols; x+=pixel_inc) {
            const T depth = img.at<T>(y, x);

            // Missing points denoted by NaNs
            if (depth_image_proc::DepthTraits<T>::valid(depth))
            {
                // Fill in XYZ
                pcl::PointXYZ point = d3d.template getPoint<pcl::PointXYZ>(x,y,depth);

                cloud->push_back(point);
            } else {
                cloud->push_back(bad_point);
            }
        }
    }

    cloud->width = img.cols / pixel_inc;
    cloud->height = img.rows / pixel_inc;
    return cloud;
}

/**
 * Convert a depth image to an unstructured XYZ point cloud. Skip every pixel_inc pixel.
 *
 * @tparam T depth type (float or uint16)
 * @param img
 * @param model
 * @param pixel_inc
 * @return
 */
template<class T>
pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthToUnstructuredSkip(cv::Mat img, const image_geometry::PinholeCameraModel& model, int pixel_inc) {
    assert(pixel_inc > 0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    DepthTo3d<T> d3d {model};

    for(int y = pixel_inc/2; y < img.rows; y+=pixel_inc) {
        for(int x = pixel_inc/2; x < img.cols; x+=pixel_inc) {
            const T depth = img.at<T>(y, x);

            // skip invalid points
            if (depth_image_proc::DepthTraits<T>::valid(depth))
            {
                // Fill in XYZ
                pcl::PointXYZ point = d3d.template getPoint<pcl::PointXYZ>(x,y,depth);

                const uint32_t index = y * img.cols + x;
                point.data[3] = *reinterpret_cast<const float*>(&index);

                cloud->push_back(point);
            }
        }
    }

    return cloud;
}


bool extractGroundPlane(cv::Mat img, const image_geometry::PinholeCameraModel& model, GroundPlaneFilterResult& result) {
    const int pixel_inc = 8;
    const float distThreshold = 0.01;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSimple, cloudFull;
    if(img.type() == CV_32F) {
        cloudSimple = convertDepthToUnstructuredSkip<float>(img, model, pixel_inc);
        cloudFull = convertDepthToPointcloud<float>(img, model, 2);
    } else if (img.type() == CV_16U) {
        cloudSimple = convertDepthToUnstructuredSkip<uint16_t>(img, model, pixel_inc);
        cloudFull = convertDepthToPointcloud<uint16_t>(img, model, 2);
    } else {
        ROS_ERROR_STREAM("Depth image is neither float nor u16, ignoring");
        return false;
    }

    pcl::ModelCoefficients coefficients;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (distThreshold);
    seg.setInputCloud (cloudSimple);
    seg.segment (*inliers, coefficients);

    result.planeCloud.clear();
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloudSimple);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(result.planeCloud);


    pcl::PointIndices::Ptr outlierIndices (new pcl::PointIndices ());
    pcl::SampleConsensusModelPlane<pcl::PointXYZ> modelPlane(cloudFull);
    modelPlane.selectWithinDistance(Eigen::Vector4f(coefficients.values.data()), distThreshold, outlierIndices->indices);

    result.nonPlaneCloud.clear();
    extract.setInputCloud(cloudFull);
    extract.setIndices(outlierIndices);
    extract.setNegative(true);
    extract.filter(result.nonPlaneCloud);

    //cv::Mat imgMask;
    //cv::resize(img, imgMask, cv::Size(img.cols/2,img.rows/2), 0, 0, cv::INTER_NEAREST);

    cv::Mat imgMask = cv::Mat(cv::Size{img.cols/2,img.rows/2}, CV_8U, cv::Scalar(255));

    for(int i: outlierIndices->indices) {
        imgMask.data[i] = 0;
    }

    //cv::erode(imgMask, imgMask, cv::Mat());
    //cv::dilate(imgMask, imgMask, cv::Mat());

    cv::Mat element3 = cv::getStructuringElement( cv::MorphShapes::MORPH_ELLIPSE, {3, 3});
    cv::morphologyEx(imgMask, imgMask, cv::MorphTypes::MORPH_OPEN, element3);

    cv::Mat element5 = cv::getStructuringElement( cv::MorphShapes::MORPH_ELLIPSE, {5, 5});
    cv::morphologyEx(imgMask, imgMask, cv::MorphTypes::MORPH_CLOSE, element5);

    cv::Mat labels, stats, centroids;
    int label_count = cv::connectedComponentsWithStats(imgMask, labels, stats, centroids, 8);

    cv::Mat labeledMask;
    cv::cvtColor(imgMask, labeledMask, cv::COLOR_GRAY2BGR);

    result.objectPoints.clear();

    for (int i = 0; i < label_count; i++)
    {
        int x = stats.at<int>(i, cv::CC_STAT_LEFT);
        int y = stats.at<int>(i, cv::CC_STAT_TOP);
        int w = stats.at<int>(i, cv::CC_STAT_WIDTH);
        int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        int cx = std::round(centroids.at<double>(i, 0));
        int cy = std::round(centroids.at<double>(i, 1));

        if(w >= 5 && w < 30 && h >= 5 && h < 30) {
            cv::rectangle(labeledMask, {x, y}, {x + w - 1, y + h - 1}, {0, 0, 255}, 3);

            const auto point = cloudFull->points.at(cy * cloudFull->width + cx);
            if(!std::isnan(point.z)) {
                result.objectPoints.push_back(point);
            }
        }
    }

    cv::imshow("labeledMask", labeledMask);
    cv::waitKey(3);

    ROS_INFO_STREAM("found objects: " << result.objectPoints.size());

    return result.planeCloud.size() > 0;
}
