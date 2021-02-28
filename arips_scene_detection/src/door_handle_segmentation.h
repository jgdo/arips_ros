#pragma once

#include "ground_segmentation.h"

#include <tf2_ros/buffer.h>
#include <ros/ros.h>

class DoorHandleSegmentation {
public:
    DoorHandleSegmentation(tf2_ros::Buffer& tf, const std::string& global_frame);

    void getDoorHandlePosition(const GroundPlaneFilterResult& filterResult);

    void setApproxDoorPos(const tf2::Vector3& posGlobal) {
        mApproxDoorHandlePos = posGlobal;
    }

    float MaxPoseDeviation = 0.3;
    float DoorHandleDist = 0.23; // distance of door handle from door (robot center)

private:
    tf2_ros::Buffer& mTfBuffer;
    std::string mGlobalFrame;

    tf2::Vector3 mApproxDoorHandlePos= {1.1, 0.12, 1.04}; // in global_frame

    ros::Publisher markerPub, posePub;
};
