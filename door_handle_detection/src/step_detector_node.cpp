#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PolygonStamped.h>

#include "DoorHandleDetector.h"

class DoorHandleDetectorNode {
public:
  const std::string AripsBaseFrame = "arips_base";
  const float FloorHeight = 0.0F;

    explicit DoorHandleDetectorNode(tf2_ros::Buffer &tf)
            : mTfBuffer(tf)
    {
        // TODO compressed
        mCameraSub = mImageTransport.subscribeCamera("/kinect/rgb/image_color", 30, &DoorHandleDetectorNode::imageCb, this,
                                                     image_transport::TransportHints("compressed"));
        mImagePub = mImageTransport.advertise("/kinect/rgb/door_handle_image", 1);
        mStepPub = mNodeHandle.advertise<geometry_msgs::PolygonStamped>("floor_step", 1);
    }

    void imageCb(const sensor_msgs::ImageConstPtr &image_msg,
                 const sensor_msgs::CameraInfoConstPtr &info_msg) {

        mCamModel.fromCameraInfo(info_msg);

        const float xFactor = image_msg->width / 384.0f;
        const float yFactor = image_msg->height / 288.0f;

        cv::Mat image;
        try {
            cv_bridge::CvImagePtr input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
            cv::resize(input_bridge->image, image, cv::Size{384, 288}, CV_INTER_LINEAR);
        }
        catch (cv_bridge::Exception &ex) {
            ROS_ERROR("[draw_frames] Failed to convert image");
            return;
        }

        const auto floorEdge = mDetector.detect(image);
        DoorHandleDetector::annotateDetected(image, floorEdge);

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(image_msg->header, "bgr8", image).toImageMsg();
        mImagePub.publish(msg);


        constexpr int margin = 5;
        std::vector<std::optional<tf2::Vector3>> points =
        {
            calc3dPoseFromPoint(margin, margin, image_msg->header.frame_id),
            calc3dPoseFromPoint(margin, image_msg->height-margin, image_msg->header.frame_id),
            calc3dPoseFromPoint(image_msg->width-margin, image_msg->height-margin, image_msg->header.frame_id),
            calc3dPoseFromPoint(image_msg->width-margin, margin, image_msg->header.frame_id),
        };

        // camera polygon not available
        if(!std::all_of(points.begin()+2, points.end(), [](const auto& p) { return p.has_value(); })) {
          return;
        }

        // polygon will have 4 points of no floor step detected, 6 if detected
        if(floorEdge) {
          points.push_back(calc3dPoseFromPoint(floorEdge->handleStart.x*xFactor, floorEdge->handleStart.y*yFactor, image_msg->header.frame_id));
          points.push_back(calc3dPoseFromPoint(floorEdge->handleEnd.x*xFactor, floorEdge->handleEnd.y*yFactor, image_msg->header.frame_id));
        }

        geometry_msgs::PolygonStamped polygon;
        polygon.header.frame_id = AripsBaseFrame;
        polygon.header.stamp = image_msg->header.stamp;

        for(const auto& p: points) {
          geometry_msgs::Point32 pointMsg;
          pointMsg.x = p->x();
          pointMsg.y = p->y();
          pointMsg.z = p->z();
          polygon.polygon.points.push_back(pointMsg);
        }
        mStepPub.publish(polygon);
    }

    std::optional<tf2::Vector3> calc3dPoseFromPoint(int x, int y, const std::string& image_frame_id) {
      const auto rayCam = mCamModel.projectPixelTo3dRay({(double)x, (double)y});

      geometry_msgs::TransformStamped transformMsg;

      try {
        transformMsg = mTfBuffer.lookupTransform(AripsBaseFrame, image_frame_id,
                                                 /*image_msg->header.stamp*/ ros::Time(0));
      } catch (const tf2::TransformException &ex) {
        ROS_WARN_STREAM("Door handle detector TF error: " << ex.what());
        return {};
      }

      tf2::Stamped<tf2::Transform> transform;
      tf2::fromMsg(transformMsg, transform);

      const auto rayBase = transform.getBasis() * tf2::Vector3(rayCam.x, rayCam.y, rayCam.z);
      const auto camBase = transform.getOrigin();

      if (rayBase.z() > -0.3F) {
        ROS_WARN("Camera seem not to point down, could not compute floor step pose.");
        return {};
      }

      // how much to scale ray such that it will have z == doorHandleHeight
      const float toDoorScale = (FloorHeight - camBase.z()) / rayBase.z();

      const auto point3d = camBase + rayBase * toDoorScale;
      return point3d;
    }

private:
    ros::NodeHandle mNodeHandle;
    tf2_ros::Buffer &mTfBuffer;

    image_transport::ImageTransport mImageTransport{mNodeHandle};
    image_transport::CameraSubscriber mCameraSub;
    image_transport::Publisher mImagePub;
    ros::Publisher mStepPub;
    image_geometry::PinholeCameraModel mCamModel;

    DoorHandleDetector mDetector {"/home/jgdo/catkin_ws/src/arips_ros/door_handle_detection/pytorch/models/floor_door_edge.onnx"};
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "door_handle_detector");

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    DoorHandleDetectorNode detector{tfBuffer};

    while (ros::ok()) {
        ros::spin();
    }
}
