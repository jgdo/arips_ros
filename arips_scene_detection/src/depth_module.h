#pragma once

#include <tf2_ros/buffer.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <visualization_msgs/MarkerArray.h>

enum class CloudResolution {
    Low, // 160x120
    Medium, // 320x240
    High, // 640x480 and above
};

class DepthModule {
public:
    DepthModule(tf2_ros::Buffer &tf)
            : mTfBuffer{tf} {
    }

    virtual CloudResolution preferredResolution() = 0;

    /**
     * @param input cloud some resolution, will try to mach preferred resolution
     * @param markerArray can be used to add custom markers
     */
    virtual void processPointcloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr const &input,
                                   visualization_msgs::MarkerArray &markerArray) = 0;

    /**
     * @param enable disable if false
     */
    virtual void enable(bool enable) = 0;

    virtual ~DepthModule() {}

protected:
    tf2_ros::Buffer &mTfBuffer;
};