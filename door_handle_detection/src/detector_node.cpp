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

#include "DoorHandleDetector.h"

class DoorHandleDetectorNode {
public:
    const std::string AripsBaseFrame = "arips_base";
    const float doorHandleHeight = 1.04F;



    explicit DoorHandleDetectorNode(tf2_ros::Buffer &tf)
            : mTfBuffer(tf)
    {
        // TODO compressed
        mCameraSub = mImageTransport.subscribeCamera("/kinect/rgb/image_color", 30, &DoorHandleDetectorNode::imageCb, this,
                                                     image_transport::TransportHints("compressed"));
        mImagePub = mImageTransport.advertise("/kinect/rgb/door_handle_image", 1);
        mHandlePub = mNodeHandle.advertise<geometry_msgs::PoseStamped>("door_handle/pose", 1);
    }

    void imageCb(const sensor_msgs::ImageConstPtr &image_msg,
                 const sensor_msgs::CameraInfoConstPtr &info_msg) {

        if(image_msg->width != 320 || image_msg->height != 240) {
            ROS_WARN("Wrong image size for door handle detection, only 320x240 supported");
            return;
        }

        cv::Mat image;
        try {
            cv_bridge::CvImagePtr input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
            image = input_bridge->image;
        }
        catch (cv_bridge::Exception &ex) {
            ROS_ERROR("[draw_frames] Failed to convert image");
            return;
        }

        const auto doorHandle = mDetector.detect(image);
        DoorHandleDetector::annotateDetected(image, doorHandle);

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(image_msg->header, "bgr8", image).toImageMsg();
        mImagePub.publish(msg);

        if (doorHandle) {
            mCamModel.fromCameraInfo(info_msg);

            const auto rayCam = mCamModel.projectPixelTo3dRay({(double)doorHandle->handleEnd.x, (double)doorHandle->handleEnd.y});

            geometry_msgs::TransformStamped transformMsg;
            try {
                transformMsg = mTfBuffer.lookupTransform(AripsBaseFrame, image_msg->header.frame_id,
                                                         /*image_msg->header.stamp*/ ros::Time(0));
            } catch (const tf2::TransformException &ex) {
                ROS_WARN_STREAM("Door handle detector TF error: " << ex.what());
                return;
            }

            tf2::Stamped<tf2::Transform> transform;
            tf2::fromMsg(transformMsg, transform);

            const auto rayBase = transform.getBasis() * tf2::Vector3(rayCam.x, rayCam.y, rayCam.z);
            const auto camBase = transform.getOrigin();

            if (rayBase.z() < 0.3F) {
                ROS_WARN("Camera seem not to point up, could not compute door handle pose.");
                return;
            }

            // how much to scale ray such that it will have z == doorHandleHeight
            const float toDoorScale = (doorHandleHeight - camBase.z()) / rayBase.z();

            const auto handleBase = camBase + rayBase * toDoorScale;

            geometry_msgs::PoseStamped handlePose;
            handlePose.header.frame_id = AripsBaseFrame;
            handlePose.header.stamp = image_msg->header.stamp;

            tf2::toMsg(handleBase, handlePose.pose.position);
            handlePose.pose.orientation.w = 1.0F; // TODO: correct orientation
            mHandlePub.publish(handlePose);
        }
    }

private:
    ros::NodeHandle mNodeHandle;
    tf2_ros::Buffer &mTfBuffer;

    image_transport::ImageTransport mImageTransport{mNodeHandle};
    image_transport::CameraSubscriber mCameraSub;
    image_transport::Publisher mImagePub;
    ros::Publisher mHandlePub;
    image_geometry::PinholeCameraModel mCamModel;

    DoorHandleDetector mDetector;
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
