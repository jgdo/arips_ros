#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <visualization_msgs/MarkerArray.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <moveit_msgs/PlanningScene.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "depth_to_cloud.h"
#include "common/objects_to_grasp_detection.h"

// see also
// https://answers.ros.org/question/46280/reg-subscribing-to-depth-and-rgb-image-at-the-same-time/

class RgbdPipeline
{
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>
      MySyncPolicy;

public:
  RgbdPipeline(tf2_ros::Buffer& tf) : mTfBuffer(tf)
  {
    mCameraInfoSub = mNode.subscribe<sensor_msgs::CameraInfo>(
        "/kinect/depth_registered/camera_info", 1, &RgbdPipeline::onCameraInfoReceived, this);

    sync.registerCallback(boost::bind(&RgbdPipeline::depthRgbCallback, this, _1, _2));

    mMarkerPub = mNode.advertise<visualization_msgs::MarkerArray>("scene_objects", 1);
    mPlanningScenePublisher = mNode.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  }

private:
  void depthRgbCallback(const sensor_msgs::ImageConstPtr& msg_rgb,
                        const sensor_msgs::ImageConstPtr& msg_depth)
  {
    if (!mCameraModel.initialized())
    {
      return;
    }

    cv_bridge::CvImagePtr cv_depth_ptr, cv_rgb_ptr;
    try
    {
      cv_depth_ptr = cv_bridge::toCvCopy(msg_depth);
      cv_rgb_ptr = cv_bridge::toCvCopy(msg_rgb);
    }
    catch (const cv_bridge::Exception& e)
    {
      ROS_ERROR("onDepthReceived(): cv_bridge exception: %s", e.what());
      return;
    }

    auto fullCloud = depthToCloud(cv_depth_ptr->image, cv_depth_ptr->header, mCameraModel, 1, true);
    auto simpleCloud = depthToCloud(cv_depth_ptr->image, cv_depth_ptr->header, mCameraModel, 8);

    if (!fullCloud || !simpleCloud)
    {
      return;
    }

    const auto plane = segmentPlane(simpleCloud);
    visualization_msgs::MarkerArray markers;
    const auto detectedObjects =
        detectObjectsInScene({ fullCloud, plane.modelCoefficients, cv_depth_ptr->image }, &markers);

    mMarkerPub.publish(markers);

    moveit_msgs::PlanningScene planning_scene;
    {
      moveit_msgs::CollisionObject object;
      object.operation = moveit_msgs::CollisionObject::REMOVE;
      planning_scene.world.collision_objects.push_back(object);
    }

    geometry_msgs::TransformStamped arm_transform;
    try
    {
      arm_transform =
          mTfBuffer.lookupTransform("arm_base_link", msg_depth->header.frame_id, ros::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
      ROS_ERROR("%s", ex.what());
      return;
    }

    constexpr float maxDist_m = 0.3;

    for (int i = 0; i < detectedObjects.detectedObjects.size(); i++)
    {
      const auto center = detectedObjects.detectedObjects.at(i).position.getOrigin();
      geometry_msgs::Point centerMsg;
      centerMsg.x = center.x();
      centerMsg.y = center.y();
      centerMsg.z = center.z();

      geometry_msgs::Point centerBase;
      tf2::doTransform(centerMsg, centerBase, arm_transform);
      // const auto centerBase = arm_transform(tfc);

      if ((centerBase.x * centerBase.x + centerBase.y * centerBase.y) > maxDist_m * maxDist_m ||
          centerBase.z > 0.07)
      {
        ROS_INFO_STREAM("Ignoring object at " << centerBase);
        continue;
      }

      std_msgs::ColorRGBA color;
      color.r = 1.0f;
      color.g = 1.0f;
      color.b = 0.0f;
      color.a = 0.3;

      geometry_msgs::Pose objectPose;
      objectPose.position = centerBase;
      objectPose.position.z -= 0.015;
      objectPose.orientation.w = 1;

          moveit_msgs::CollisionObject object;
      object.header.frame_id = "arm_base_link";
      object.header.stamp = msg_depth->header.stamp;

      object.operation = moveit_msgs::CollisionObject::ADD;
      object.id = std::to_string(i);

      /* Define a box to be attached */
      shape_msgs::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      primitive.dimensions = { 0.03, 0.03, 0.03 };

      object.primitives.push_back(primitive);

      object.primitive_poses.push_back(objectPose);

      planning_scene.world.collision_objects.push_back(object);
    }

    planning_scene.is_diff = true;
    mPlanningScenePublisher.publish(planning_scene);
  }

  void onCameraInfoReceived(const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    mCameraModel.fromCameraInfo(info_msg);
  }

  tf2_ros::Buffer& mTfBuffer;
  ros::NodeHandle mNode;

  image_geometry::PinholeCameraModel mCameraModel;

  message_filters::Subscriber<sensor_msgs::Image> subscriber_depth{ mNode,
                                                                    "/kinect/depth_registered/"
                                                                    "hw_registered/image_rect_raw",
                                                                    1 };
  message_filters::Subscriber<sensor_msgs::Image> subscriber_rgb{ mNode,
                                                                  "/kinect/rgb/image_rect_color",
                                                                  1 };
  message_filters::Synchronizer<MySyncPolicy> sync{ MySyncPolicy(10), subscriber_rgb,
                                                    subscriber_depth };
  ros::Subscriber mCameraInfoSub;

  ros::Publisher mMarkerPub;

  ros::Publisher mPlanningScenePublisher;
};

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "arips_scene_detection");
  ros::NodeHandle nh;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  RgbdPipeline depthPipeline{ tfBuffer };
  // Spin
  ros::spin();
}
