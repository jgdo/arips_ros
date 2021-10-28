#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf2_ros/transform_listener.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include "depth_to_cloud.h"
#include "door_detector.h"

#if 0
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <moveit_msgs/PlanningScene.h>

#include <pcl/filters/voxel_grid.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/filters/extract_indices.h>
#include <object_recognition_msgs/TableArray.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Bool.h>

#include <cv_bridge/cv_bridge.h>

#include "ground_segmentation.h"
#include "door_handle_segmentation.h"

#include <dynamic_reconfigure/server.h>
#include <arips_scene_detection/SceneDetectionConfig.h>

#endif

#include "door_detector.h"

#if 0

using namespace visualization_msgs;


ros::Publisher pub;
ros::Publisher marker_pub;
ros::Publisher pub_cloud;
ros::Publisher pub_cloud_neg;
ros::Publisher pub_table;
ros::Publisher pick_pose_pub;
ros::Publisher kinect_correction_angle_pub, planning_scene_diff_publisher;


ros::Publisher scene_pub;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;

std::shared_ptr<tf::TransformListener> listener;

struct DetectedObject {
    tf::Vector3 pos = tf::Vector3{0,0,0};
    int count = 0;
    bool updated = false;
    int id = rand();
};

std::list<DetectedObject> detectedObjects;

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    std::ostringstream s;
    s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

    std::ostringstream mouse_point_ss;
    if( feedback->mouse_point_valid )
    {
        mouse_point_ss << " at " << feedback->mouse_point.x
                       << ", " << feedback->mouse_point.y
                       << ", " << feedback->mouse_point.z
                       << " in frame " << feedback->header.frame_id;
    }

    switch ( feedback->event_type )
    {
        case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
            ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
            {
                tf::Vector3 pose, closestPose;
                double closestDist = -1;
                
                tf::pointMsgToTF(feedback->mouse_point, pose);
                
                for(auto& obj: detectedObjects) {
                    if(obj.count >= 3) {
                        double dist = (pose - obj.pos).length();
                        
                        if(closestDist < 0 || dist < closestDist) {
                            closestDist = dist;
                            closestPose = obj.pos;
                        }
                    }
                }
                
                geometry_msgs::Point selectedPose;
                tf::pointTFToMsg(closestPose, selectedPose);
                
                pick_pose_pub.publish(selectedPose);
            }
            break;

        case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
            ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
            break;

        case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
            ROS_INFO_STREAM( s.str() << ": pose changed"
                                     << "\nposition = "
                                     << feedback->pose.position.x
                                     << ", " << feedback->pose.position.y
                                     << ", " << feedback->pose.position.z
                                     << "\norientation = "
                                     << feedback->pose.orientation.w
                                     << ", " << feedback->pose.orientation.x
                                     << ", " << feedback->pose.orientation.y
                                     << ", " << feedback->pose.orientation.z
                                     << "\nframe: " << feedback->header.frame_id
                                     << " time: " << feedback->header.stamp.sec << "sec, "
                                     << feedback->header.stamp.nsec << " nsec" );

            break;

        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
            ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
            break;

        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
            ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
            break;
    }

    server->applyChanges();
}

image_geometry::PinholeCameraModel depth_camera_model;

std::unique_ptr<DoorHandleSegmentation> door_seg;

tf2_ros::Buffer tfBuffer;

bool doorHandleEnabled = false;

void dr_callback(arips_scene_detection::SceneDetectionConfig &config, uint32_t level) {
    door_seg->DoorHandleDist = config.door_approach_dist;
}

void enable_cb(const std_msgs::Bool& enable) {
    doorHandleEnabled = enable.data;
    std::cout << "Enabling door processing: " << doorHandleEnabled << std::endl;
}

void clicked_cb(const geometry_msgs::PointStamped& point) {
    auto pointWorld = tfBuffer.transform(point, "map", ros::Duration(0.5));
    pointWorld.point.z = 1.04; // handle hight
    tf2::Vector3 handle(pointWorld.point.x, pointWorld.point.y, pointWorld.point.z);
    door_seg->setApproxDoorPos(handle);
    std::cout << "Set door handle " << pointWorld << std::endl;
}

void info_cb(const sensor_msgs::CameraInfoConstPtr& info_msg) {
    depth_camera_model.fromCameraInfo(info_msg);
}

void depth_cb(const sensor_msgs::ImageConstPtr& depth_msg) {
    if(!doorHandleEnabled) {
        return;
    }

    // ROS_INFO_STREAM("depth_cb");

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(depth_msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    GroundPlaneFilterResult result;
    extractGroundPlane({cv_ptr->image, depth_camera_model, true}, result);

    result.planeCloud.header.frame_id = depth_msg->header.frame_id;
    pcl_conversions::toPCL(depth_msg->header.stamp, result.planeCloud.header.stamp);
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(result.planeCloud, output);
    pub_cloud.publish(output);

    result.nonPlaneCloud.header.frame_id = depth_msg->header.frame_id;
    pcl_conversions::toPCL(depth_msg->header.stamp, result.nonPlaneCloud.header.stamp);
    pcl::toROSMsg(result.nonPlaneCloud, output);
    pub_cloud_neg.publish(output);

    door_seg->getDoorHandlePosition(result);

#ifdef PUBLISH_SCENE_OBJECTS
    tf::StampedTransform arm_transform;
    try {
        listener->lookupTransform("arm_base_link", depth_msg->header.frame_id,
                                  ros::Time(0), arm_transform);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        return;
    }

    constexpr float maxDist_m = 0.3;

    moveit_msgs::PlanningScene planning_scene;
    {
        moveit_msgs::CollisionObject object;
        object.operation = moveit_msgs::CollisionObject::REMOVE;
        planning_scene.world.collision_objects.push_back(object);
    }

    visualization_msgs::MarkerArray scene_objects;
    {
        visualization_msgs::Marker marker;
        marker.action = visualization_msgs::Marker::DELETEALL;
        scene_objects.markers.push_back(marker);
    }

    for(int i = 0; i < result.objectPoints.size(); i++) {
        const auto center = result.objectPoints.at(i);
        tf::Vector3 tfc(center.x, center.y, center.z);
        tf::Vector3 centerBase = arm_transform(tfc);

        if(centerBase.length() > maxDist_m || centerBase.z() > 0.1) {
            ROS_INFO_STREAM("Ignoring object at " << centerBase);
            continue;
        }

        std_msgs::ColorRGBA color;
        color.r = 1.0f;
        color.g = 1.0f;
        color.b = 0.0f;
        color.a = 0.3;

        geometry_msgs::Point point;
        tf::pointTFToMsg(centerBase, point);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "arm_base_link";
        marker.header.stamp = depth_msg->header.stamp;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.pose.position = point;
        marker.pose.orientation.w = 1;
        marker.id = i;
        marker.color = color;
        marker.scale.x = 0.03;
        marker.scale.y = 0.03;
        marker.scale.z = 0.03;
        scene_objects.markers.push_back(marker);


        moveit_msgs::CollisionObject object;
        object.header = marker.header;

        object.operation = moveit_msgs::CollisionObject::ADD;
        object.id = std::to_string(i);

        /* Define a box to be attached */
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = {0.00, 0.00, 0.00};

        object.primitives.push_back(primitive);

        object.primitive_poses.push_back(marker.pose);

        planning_scene.world.collision_objects.push_back(object);
    }

    scene_pub.publish(scene_objects);

    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);
#endif
}

#endif

class RGBDPipeline
{
public:
  RGBDPipeline(tf2_ros::Buffer& tf):
        mTfBuffer(tf),
        mDoorDetector{tf}
    {
        mDepthSub = mImageTransport.subscribe("/kinect/depth_registered/image_raw", 2, &RGBDPipeline::onDepthReceived, this);
        mCameraInfoSub = mNode.subscribe<sensor_msgs::CameraInfo>("/kinect/depth_registered/camera_info", 1, &RGBDPipeline::onCameraInfoReceived, this);
        mMarkerPub = mNode.advertise<visualization_msgs::MarkerArray>("scene_objects", 1);

        mDoorDetector.enable(true);
    }

private:
    void onDepthReceived(const sensor_msgs::ImageConstPtr& depth_msg) {
        if(!mCameraModel.initialized()) {
            return;
        }

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(depth_msg);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("onDepthReceived(): cv_bridge exception: %s", e.what());
            return;
        }

        auto cloud = depthToCloud(cv_ptr->image, depth_msg->header, mCameraModel, 8);
        if(!cloud) {
            return;
        }

        visualization_msgs::MarkerArray markers;
        mDoorDetector.processPointcloud(cloud, markers);
        mMarkerPub.publish(markers);
    }

    void onCameraInfoReceived(const sensor_msgs::CameraInfoConstPtr& info_msg) {
        mCameraModel.fromCameraInfo(info_msg);
    }

    tf2_ros::Buffer& mTfBuffer;
    ros::NodeHandle mNode;

    image_geometry::PinholeCameraModel mCameraModel;

    image_transport::ImageTransport mImageTransport {mNode};
    // image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    image_transport::Subscriber mDepthSub;
    ros::Subscriber mCameraInfoSub;

    ros::Publisher mMarkerPub;

    DoorDetector mDoorDetector;
};

int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "arips_scene_detection");
    ros::NodeHandle nh;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);


    DoorDetector doorDetector {tfBuffer};


    //door_seg.reset(new DoorHandleSegmentation(tfBuffer, "map"));


    // Create a ROS publisher for the output point cloud
    //pub = nh.advertise<pcl_msgs::ModelCoefficients> ("coefficients", 1);
    //  pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("plane_cloud", 1);
    //  pub_cloud_neg = nh.advertise<sensor_msgs::PointCloud2> ("plane_cloud_neg", 1);
    // TODO marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // TODO scene_pub = nh.advertise<visualization_msgs::MarkerArray>("scene_objects", 1);

    // TODO server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls","",false) );

    // TODO pub_table = nh.advertise<object_recognition_msgs::TableArray>("/table_array", 1);

    // TODO pick_pose_pub = nh.advertise<geometry_msgs::Point>("/pick_pose", 1);

    // TODO kinect_correction_angle_pub = nh.advertise<std_msgs::Float32>("/kinect_correction_angle", 1);

    // Create a ROS subscriber for the input point cloud
    // ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/kinect/depth_registered/points", 1, cloud_cb);

#if 0
    ros::Subscriber sub_info_ = nh.subscribe<sensor_msgs::CameraInfo>("/kinect/depth_registered/camera_info", 1, info_cb);

    ros::Subscriber clicked_sub = nh.subscribe("approx_door_handle_pos", 1, clicked_cb);

    ros::Subscriber enable_sub = nh.subscribe("enable_door_handle", 1, enable_cb);

    image_transport::ImageTransport it(nh);
    // image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    image_transport::Subscriber sub_depth = it.subscribe("/kinect/depth_registered/image_raw", 2, depth_cb);

    // TODO planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    dynamic_reconfigure::Server<arips_scene_detection::SceneDetectionConfig> server;
    dynamic_reconfigure::Server<arips_scene_detection::SceneDetectionConfig>::CallbackType f = boost::bind(dr_callback, _1, _2);;
    server.setCallback(f);

#endif
    RGBDPipeline depthPipeline {tfBuffer};
    // Spin
    ros::spin();
}

