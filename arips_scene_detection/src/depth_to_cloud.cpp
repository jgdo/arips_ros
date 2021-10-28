//
// Created by jgdo on 13.04.21.
//

#include <cstdint>
#include <vector>
#include <cmath>

#include <depth_image_proc/depth_traits.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>

#include "depth_to_cloud.h"


template<class T>
struct DepthTo3d {
    explicit DepthTo3d(const image_geometry::PinholeCameraModel& model):
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
 * Convert a depth image to an unstructured XYZ point cloud. Skip every pixel_inc pixel.
 *
 * @tparam T depth type (float or uint16)
 * @param img
 * @param model
 * @param pixel_inc
 * @return
 */
template<class T>
pcl::PointCloud<pcl::PointXYZ>::Ptr
convertDepthToPointcloudSkip(cv::Mat img, const image_geometry::PinholeCameraModel &model, int pixel_inc, bool forceStructured) {
    assert(pixel_inc > 0);

    const float bad_float = std::numeric_limits<float>::quiet_NaN();
    const pcl::PointXYZ bad_point {bad_float, bad_float, bad_float};

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->reserve(img.total());
    cloud->is_dense = false; // points might be invalid

    DepthTo3d <T> d3d{model};

    for (int y = pixel_inc / 2; y < img.rows; y += pixel_inc) {
        for (int x = pixel_inc / 2; x < img.cols; x += pixel_inc) {
            const T depth = img.at<T>(y, x);

            // skip invalid points
            if (depth_image_proc::DepthTraits<T>::valid(depth)) {
                // Fill in XYZ
                pcl::PointXYZ point = d3d.template getPoint<pcl::PointXYZ>(x, y, depth);

                const uint32_t index = y * img.cols + x;
                point.data[3] = *reinterpret_cast<const float *>(&index);

                cloud->push_back(point);
            } else if(forceStructured){
              cloud->push_back(bad_point);
            }
        }
    }

    if(forceStructured) {
      cloud->width = img.cols / pixel_inc;
      cloud->height = img.rows / pixel_inc;
    }

    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
depthToCloud(const cv::Mat &depthImage, std_msgs::Header const& depthHeader, const image_geometry::PinholeCameraModel &model, int pixelInc, bool forceStructured) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSimple;
    if (depthImage.type() == CV_32F) {
        cloudSimple = convertDepthToPointcloudSkip<float>(depthImage, model, pixelInc, forceStructured);
    } else if (depthImage.type() == CV_16U) {
        cloudSimple = convertDepthToPointcloudSkip<uint16_t>(depthImage, model, pixelInc, forceStructured);
    } else {
        ROS_ERROR_STREAM("Depth image is neither float nor u16, ignoring");
        return nullptr;
    }

    cloudSimple->header = pcl_conversions::toPCL(depthHeader);

    return cloudSimple;
}