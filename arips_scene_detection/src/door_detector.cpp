//
// Created by jgdo on 11.04.21.
//

#include "door_detector.h"
#include "plane_segmentation.h"

#include <pcl_conversions/pcl_conversions.h>
#include <angles/angles.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

DoorDetector::DoorDetector(tf2_ros::Buffer &tf) :
        DepthModule(tf) {
    ros::NodeHandle nh;

    mDoorPosePub = nh.advertise<geometry_msgs::PoseStamped>("detected_door_pose", 2);
    mPivotSub = nh.subscribe("door_pivot_pose", 1, &DoorDetector::onPivotPose, this);
}

CloudResolution DoorDetector::preferredResolution() {
    return CloudResolution::Low;
}

void DoorDetector::processPointcloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &input,
                                     visualization_msgs::MarkerArray &markerArray) {
    const float distanceTolerance = 0.2;
    const float angleTolerance = angles::from_degrees(10);
    const float doorRadius = 0.8;
    const int minDoorPoints = 150;

    std_msgs::Header inputHeader;
    pcl_conversions::fromPCL(input->header, inputHeader);

    if (mLastPivotPose.header.frame_id.empty()) {
        ROS_WARN("processPointcloud(): No door pivot pose received yet");

        // no pivot point received yet
        return;
    }

    tf2::Transform pivotTransform;

    try {
        geometry_msgs::PoseStamped pivotPose;
        mTfBuffer.transform(mLastPivotPose, pivotPose, input->header.frame_id,
                            inputHeader.stamp, mLastPivotPose.header.frame_id,
                            ros::Duration(0.5));

        tf2::fromMsg(pivotPose.pose, pivotTransform);
    }
    catch (const tf2::TransformException &ex) {
        ROS_WARN("processPointcloud(): %s", ex.what());
        return;
    }

    auto projectOnDoorPivotPlane = [&pivotTransform](const tf2::Vector3 pt) {
        const tf2::Vector3 pivotToPt = pt - pivotTransform.getOrigin();

        // project point on  pivot x/y plane
        const tf2::Vector3 onPivotPlane = pivotTransform.getOrigin()
                                      + pivotTransform.getBasis().getColumn(0) *
                                        pivotTransform.getBasis().getColumn(0).dot(pivotToPt)
                                      + pivotTransform.getBasis().getColumn(1) *
                                        pivotTransform.getBasis().getColumn(1).dot(pivotToPt);

        return onPivotPlane;
    };

    // filter for points inside a cylinder around door pivot axis
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered { new pcl::PointCloud<pcl::PointXYZ>() };
    cloudFiltered->reserve(input->size());
    for(const auto pt: *input) {
        if(projectOnDoorPivotPlane({pt.x, pt.y, pt.z}).distance2(pivotTransform.getOrigin()) <= doorRadius*doorRadius) {
            cloudFiltered->push_back(pt);
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::ConstPtr remainingCloud = cloudFiltered;

    for(int i = 0; i < 3; i++) {
        auto planeResult = segmentPlane(remainingCloud);
        remainingCloud = planeResult.nonCloud;

        if (planeResult.direction.isZero()) {
            break;
        }

        if(planeResult.cloud->size() < minDoorPoints) {
            break;
        }

        // check if plane is within angle tolerance
        const float zAngle = angles::normalize_angle(planeResult.direction.angle(pivotTransform.getBasis().getColumn(2))
                - angles::from_degrees(90));
        if (std::abs(zAngle) > angleTolerance) {
            continue;
        }

        // check if plane is close to pivot point
        const float pivotDistance = planeResult.direction.dot(pivotTransform.getOrigin()) - planeResult.distance;
        if (std::abs(pivotDistance) > distanceTolerance) {
            continue;
        }

        // compute plane pose
        const tf2::Vector3 doorBase = projectOnDoorPivotPlane(planeResult.direction * planeResult.distance);

        // compute door rotation basis such that plane hessian is x-axis and pivot z is approximately door z
        const tf2::Vector3 doorDirX = planeResult.direction;
        const tf2::Vector3 doorDirY = -doorDirX.cross(pivotTransform.getBasis().getColumn(2));
        const tf2::Vector3 doorDirZ = doorDirX.cross(doorDirY);

        const tf2::Transform doorBasePose{tf2::Matrix3x3{doorDirX.x(), doorDirY.x(), doorDirZ.x(),
                                                         doorDirX.y(), doorDirY.y(), doorDirZ.y(),
                                                         doorDirX.z(), doorDirY.z(), doorDirZ.z()}, doorBase};

        // publish plane pose
        geometry_msgs::PoseStamped doorPoseMsg;
        tf2::toMsg(doorBasePose, doorPoseMsg.pose);
        doorPoseMsg.header = inputHeader;
        mDoorPosePub.publish(doorPoseMsg);

        // create markers
        visualization_msgs::Marker points;
        points.header = inputHeader;
        points.ns = "door_detector";
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.id = 0;
        points.lifetime = ros::Duration(0.3);
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 0.02;
        points.scale.y = 0.02;
        points.color.g = 1.0f;
        points.color.a = 1.0;
        for(const auto& xyz: *planeResult.cloud) {
            geometry_msgs::Point p;
            p.x = xyz.x;
            p.y = xyz.y;
            p.z = xyz.z;
            points.points.push_back(p);
        }

        markerArray.markers.push_back(points);

        return;
    }

    ROS_WARN("processPointcloud(): could not find door plane after multiple iterations");
}

void DoorDetector::enable(bool enable) {
    mLastPivotPose = geometry_msgs::PoseStamped{};
}

void DoorDetector::onPivotPose(const geometry_msgs::PoseStamped &pose) {
    ROS_INFO_STREAM("Received door pivot pose");
    mLastPivotPose = pose;
}
