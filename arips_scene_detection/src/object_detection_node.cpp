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

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <angles/angles.h>

struct HSV
{
  float h, s, v;
};

// https://wisotop.de/rgb-nach-hsv.php
HSV rgbToHsv(const std_msgs::ColorRGBA& rgb)
{
  const auto min = std::min({ rgb.r, rgb.g, rgb.b });
  const auto max = std::max({ rgb.r, rgb.g, rgb.b });
  const auto delta = max - min;

  // black
  if (max == 0.0)
  {
    return { 0, 0, 0 };
  }

  // pure gray
  if (max == min)
  {
    return { 0, 0, max };
  }

  const auto s = delta / max;

  float h = 0.0;
  if (rgb.r == max)
  {
    h = (rgb.g - rgb.b) / delta;
  }
  else if (rgb.g == max)
  {
    h = 2 + (rgb.b - rgb.r) / delta;
  }
  else
  {
    h = 4 + (rgb.r - rgb.g) / delta;
  }

  return { static_cast<float>(angles::normalize_angle_positive(h * M_PI / 3.0)), s, max };
}

// see also
// https://answers.ros.org/question/46280/reg-subscribing-to-depth-and-rgb-image-at-the-same-time/

class RgbdPipeline
{
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>
      MySyncPolicy;

public:
  RgbdPipeline(const std::shared_ptr<tf2_ros::Buffer>& tf) : mTfBuffer(tf)
  {
    mCameraInfoSub = mNode.subscribe<sensor_msgs::CameraInfo>(
        "/kinect/depth_registered/camera_info", 1, &RgbdPipeline::onCameraInfoReceived, this);

    sync.registerCallback(boost::bind(&RgbdPipeline::depthRgbCallback, this, _1, _2));

    mMarkerPub = mNode.advertise<visualization_msgs::MarkerArray>("scene_objects", 1);

    mPsm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description",
                                                                          mTfBuffer);
    mPsm->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                       "/planning_scene");

    // mPsm->startSceneMonitor();
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

    assert(cv_rgb_ptr->image.type() == CV_8UC3);

    auto fullCloud = depthToCloud(cv_depth_ptr->image, cv_depth_ptr->header, mCameraModel, 1, true);
    auto simpleCloud = depthToCloud(cv_depth_ptr->image, cv_depth_ptr->header, mCameraModel, 8);

    if (!fullCloud || !simpleCloud)
    {
      return;
    }

    geometry_msgs::TransformStamped arm_transform;
    try
    {
      arm_transform =
          mTfBuffer->lookupTransform("arm_base_link", msg_depth->header.frame_id, ros::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
      ROS_ERROR("%s", ex.what());
      return;
    }

    auto convertTransform = [](const geometry_msgs::TransformStamped& msg) {
      tf2::Stamped<tf2::Transform> out;
      tf2::fromMsg(msg, out);
      return out;
    };

    const auto armTransformTf = convertTransform(arm_transform);

    const auto plane = segmentPlane(simpleCloud);
    visualization_msgs::MarkerArray markers;
    const auto detectedObjects =
        detectObjectsInScene({ fullCloud, plane.modelCoefficients, cv_depth_ptr->image,
                               cv_rgb_ptr->image, armTransformTf },
                             &markers);

    mMarkerPub.publish(markers);

    {
      planning_scene_monitor::LockedPlanningSceneRW scene(mPsm);
      scene->removeAllCollisionObjects();

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
          ROS_DEBUG_STREAM("Ignoring object at " << centerBase);
          continue;
        }

        geometry_msgs::Pose objectPose;
        objectPose.position = centerBase;
        objectPose.position.z -= 0.015;
        objectPose.orientation.w = 1;

        moveit_msgs::CollisionObject object;
        object.header.frame_id = "arm_base_link";
        object.header.stamp = msg_depth->header.stamp;

        object.operation = moveit_msgs::CollisionObject::ADD;
        object.id = std::to_string(i);

        const auto meanColor = detectedObjects.detectedObjects.at(i).meanColor;
        const auto hsv = rgbToHsv(meanColor);

        if(hsv.v < 0.3 && hsv.s < 0.2) {
          object.type.key = "black";
        }
        else if(hsv.s < 0.25) {
          object.type.key = "white";
        } else if (std::abs(angles::normalize_angle(hsv.h - 0.0)) < 0.7) {
          object.type.key = "red";
        } else if (std::abs(angles::normalize_angle(hsv.h - 2.1)) < 0.7) {
          object.type.key = "green";
        } else if (std::abs(angles::normalize_angle(hsv.h - 4.18879)) < 0.7) {
          object.type.key = "blue";
        }

        /* Define a box to be attached */
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = { 0.03, 0.03, 0.03 };

        object.primitives.push_back(primitive);

        object.primitive_poses.push_back(objectPose);

        scene->processCollisionObjectMsg(object);
        scene->setObjectColor(object.id, meanColor);
      }
    }

    mPsm->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
  }

  void onCameraInfoReceived(const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    mCameraModel.fromCameraInfo(info_msg);
  }

  std::shared_ptr<tf2_ros::Buffer> mTfBuffer;
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

  planning_scene_monitor::PlanningSceneMonitorPtr mPsm;
};

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "arips_scene_detection");
  ros::NodeHandle nh;

  auto tfBuffer = std::make_shared<tf2_ros::Buffer>();
  tf2_ros::TransformListener tfListener(*tfBuffer);

  RgbdPipeline depthPipeline{ tfBuffer };
  // Spin
  ros::spin();
}
