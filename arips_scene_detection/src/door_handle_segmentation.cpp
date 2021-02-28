#include "door_handle_segmentation.h"

#include "ground_segmentation.h"
#include <pcl/common/centroid.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>

DoorHandleSegmentation::DoorHandleSegmentation(tf2_ros::Buffer& tf, const std::string& global_frame)
: mTfBuffer(tf)
, mGlobalFrame(global_frame)

{
    ros::NodeHandle nh;
    markerPub = nh.advertise<visualization_msgs::Marker>("door_handle_marker", 1);
    posePub = nh.advertise<geometry_msgs::PoseStamped>("door_handle_pose", 1);
}

tf2::Vector3 fromPcl(const pcl::PointXYZ& p) {
    return tf2::Vector3(p.x, p.y, p.z);
}

void DoorHandleSegmentation::getDoorHandlePosition(const GroundPlaneFilterResult& filterResult) {
    try {

        const auto toWorldMsg = mTfBuffer.lookupTransform(mGlobalFrame, filterResult.frame_id, ros::Time(0));
        tf2::Transform toWorld;
        tf2::fromMsg(toWorldMsg.transform, toWorld);

        for (const auto &indices: filterResult.allObjectIndices) {
            pcl::CentroidPoint<pcl::PointXYZ> centroid;

            for (auto index: indices) {
                centroid.add(filterResult.cloudFull->points.at(index));
            }

            pcl::PointXYZ centerPcl;
            centroid.get(centerPcl);

            const tf2::Vector3 centerCamera = fromPcl(centerPcl);
            const tf2::Vector3 centerWorld = toWorld(centerCamera);

            if (centerWorld.distance(mApproxDoorHandlePos) < MaxPoseDeviation) {
                // assume for now there is alwayS at most 1 detected door handle

                const Eigen::Vector4f pt(centerPcl.x, centerPcl.y, centerPcl.z, 1.0f);
                const Eigen::Vector4f planeCoeff = getPlaneCoefficientsToCamera(filterResult.planeCoefficients);
                const float doorDist = pt.dot(planeCoeff);

                const tf2::Vector3 doorNormal(filterResult.planeCoefficients[0], filterResult.planeCoefficients[1],
                                              filterResult.planeCoefficients[2]);
                const tf2::Vector3 handleComputedCamera = centerCamera + doorNormal * (doorDist - DoorHandleDist);

                {
                    const Eigen::Vector4f pt(handleComputedCamera.x(), handleComputedCamera.y(), handleComputedCamera.z(), 1.0f);
                    const float doorDist = pt.dot(planeCoeff);
                    ROS_INFO_STREAM("Target door dist: " << doorDist);
                }

                const tf2::Vector3 handleComputedWorld = toWorld(handleComputedCamera);

                std::cout << "Door handle world coordinate " << handleComputedWorld << std::endl;

                geometry_msgs::PoseStamped approachPose;
                approachPose.header.frame_id = mGlobalFrame;
                approachPose.header.stamp = ros::Time();
                approachPose.pose.position.x = handleComputedWorld.x();
                approachPose.pose.position.y = handleComputedWorld.y();
                approachPose.pose.position.z = 0;
                approachPose.pose.orientation.x = 0.0;
                approachPose.pose.orientation.y = 0.0;
                approachPose.pose.orientation.z = 0.0;
                approachPose.pose.orientation.w = 1.0;
                posePub.publish(approachPose);

                visualization_msgs::Marker marker;
                marker.header.frame_id = mGlobalFrame;
                marker.header.stamp = ros::Time();
                marker.ns = "my_namespace";
                marker.id = 0;
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = centerWorld.x();
                marker.pose.position.y = centerWorld.y();
                marker.pose.position.z = centerWorld.z();
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;
                marker.color.a = 1.0; // Don't forget to set the alpha!
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                markerPub.publish( marker );
            }
        }
    } catch(const tf2::TransformException& ex)  {
        ROS_WARN_STREAM("Failed to get door handle position: " << ex.what());
    }
}
