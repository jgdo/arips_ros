#pragma once

#include "depth_module.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

/**
 * Detect a door in front of the robot. Door can be open, half-open or closed.
 *
 * Detector keeps extracting RANSAC plane from pointcloud until a door plane around pivot point is found.
 * Pivot point is provided by last message on topic topic door_pivot_pose (PoseStamped).
 *
 * Publish PoseStamped over detected_door_pose which indicates the door center. X points towards open direction
 */
class DoorDetector: public DepthModule {
public:
    DoorDetector(tf2_ros::Buffer &tf);

    CloudResolution preferredResolution() override;

    void processPointcloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &input, visualization_msgs::MarkerArray& markerArray) override;

    void enable(bool enable) override;

private:
    ros::Subscriber mPivotSub;
    ros::Publisher mDoorPosePub;

    geometry_msgs::PoseStamped mLastPivotPose;

    void onPivotPose(const geometry_msgs::PoseStamped& pose);
};
