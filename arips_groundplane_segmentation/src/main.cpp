#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/filters/extract_indices.h>
#include <object_recognition_msgs/TableArray.h>
#include <tf/tf.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

using namespace visualization_msgs;


ros::Publisher pub;
ros::Publisher marker_pub;
ros::Publisher pub_cloud;
ros::Publisher pub_cloud_neg;
ros::Publisher pub_table;
ros::Publisher pick_pose_pub;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;

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
            pick_pose_pub.publish(feedback->mouse_point);
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

Marker makeBox( InteractiveMarker &msg )
{
    Marker marker;

    marker.type = Marker::CUBE;
    marker.scale.x = msg.scale * 0.05;
    marker.scale.y = msg.scale * 0.05;
    marker.scale.z = msg.scale * 0.05;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;

    return marker;
}

void makeButtonMarker( const tf::Vector3& position, std_msgs::Header const& header )
{
    InteractiveMarker int_marker;
    int_marker.header = header;
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 1;

    int_marker.name = "Objects";
    int_marker.description = "Button\n(Left Click)";

    InteractiveMarkerControl control;

    control.interaction_mode = InteractiveMarkerControl::BUTTON;
    control.name = "button_control";

    Marker marker = makeBox( int_marker );
    control.markers.push_back( marker );
    control.always_visible = true;
    int_marker.controls.push_back(control);

    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
}

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    //pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
   pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg (*cloud_msg, cloud);

    pcl::ModelCoefficients coefficients;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.015);

    seg.setInputCloud (cloud.makeShared ());
    seg.segment (*inliers, coefficients);


    if (inliers->indices.size () == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    } else {
        pcl::PointCloud<pcl::PointXYZ> cloud_inliers;
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud.makeShared ());
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (cloud_inliers);

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(cloud_inliers, output);
        pub_cloud.publish(output);

        extract.setNegative (true);
        extract.filter (cloud_inliers);
        pcl::toROSMsg(cloud_inliers, output);
        pub_cloud_neg.publish(output);

        // Publish the model coefficients
        pcl_msgs::ModelCoefficients ros_coefficients;
        pcl_conversions::fromPCL(coefficients, ros_coefficients);
        pub.publish(ros_coefficients);

        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        vg.setInputCloud (cloud_inliers.makeShared());
        vg.setLeafSize (0.01f, 0.01f, 0.01f);
        vg.filter (*cloud_filtered);

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud_filtered);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.015); // 1cm
        ec.setMinClusterSize (15);
        ec.setMaxClusterSize (5000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_filtered);
        ec.extract (cluster_indices);


        visualization_msgs::Marker object;
        object.header.frame_id = cloud_msg->header.frame_id;
        object.header.stamp = ros::Time::now();
        object.ns = "clustered_objects";
        object.id = 0;
        object.type = visualization_msgs::Marker::POINTS;
        object.action = visualization_msgs::Marker::ADD;

        int j = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            pcl::CentroidPoint<pcl::PointXYZ> centroid;
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
                centroid.add(cloud_filtered->points[*pit]);
            }
            pcl::PointXYZ center;
            centroid.get(center);

             // ROS_INFO_STREAM("Found cluster " << j << " containing " << it->indices.size() << " points: " << center.x << " " << center.y << " " << center.z);


            geometry_msgs::Point p;
            p.x = center.x;
            p.y = center.y;
            p.z = center.z;
            object.points.push_back(p);

            std_msgs::ColorRGBA color;
            color.r = 0.0f;
            color.g = 1.0f;
            color.b = 0.0f;
            color.a = 1.0;
            object.colors.push_back(color);

            /*
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true; */

            j++;
        }

        object.scale.x = 0.1;
        object.scale.y = 0.1;
        object.scale.z = 0.1;


        InteractiveMarker int_marker;
        int_marker.header = cloud_msg->header;
        int_marker.header.stamp = ros::Time::now();
        int_marker.scale = 1;

        int_marker.name = "Objects";
        int_marker.description = "Button\n(Left Click)";

        InteractiveMarkerControl control;

        control.interaction_mode = InteractiveMarkerControl::BUTTON;
        control.name = "button_control";

        control.markers.push_back( object );
        control.always_visible = true;
        int_marker.controls.push_back(control);

        server->insert(int_marker, processFeedback);
        server->applyChanges();

        visualization_msgs::Marker marker;
        marker.header.frame_id = cloud_msg->header.frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.05;
        marker.scale.y = 0.1;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.points.resize(2);
        marker.points[0].x = coefficients.values.at(0) * -coefficients.values.at(3);
        marker.points[0].y = coefficients.values.at(1) * -coefficients.values.at(3);
        marker.points[0].z = coefficients.values.at(2) * -coefficients.values.at(3);
        marker.points[1].x = marker.points[0].x + coefficients.values.at(0);
        marker.points[1].y = marker.points[0].y + coefficients.values.at(1);
        marker.points[1].z = marker.points[0].z + coefficients.values.at(2);

        marker_pub.publish(marker);

        tf::Vector3 z = tf::Vector3(coefficients.values.at(0), coefficients.values.at(1), coefficients.values.at(2)).normalized();
        tf::Vector3 someVec(marker.points[0].x +10, marker.points[0].y, marker.points[0].z);
        tf::Vector3 y = z.cross(someVec).normalized();
        tf::Vector3 x = y.cross(z).normalized();

        tf::Matrix3x3 mat(x.x(), x.y(), x.z(), y.x(), y.y(), y.z(), z.x(),x.y(),z.z());
        tf::Quaternion quat;
        mat.getRotation(quat);

        tf::Vector3 center(marker.points[0].x, marker.points[0].y, marker.points[0].z);
        tf::Vector3 p1 = center + -0.5*x + -0.5*y;
        tf::Vector3 p2 = center + -0.5*x + 0.5*y;
        tf::Vector3 p3 = center + 0.5*x + 0.5*y;
        tf::Vector3 p4 = center + 0.5*x + -0.5*y;

        object_recognition_msgs::TableArray tables;
        tables.header = marker.header;
        tables.tables.resize(1);
        tables.tables.at(0).header = tables.header;
        tables.tables.at(0).pose.position = marker.points[0];
        tf::quaternionTFToMsg(quat.normalized(), tables.tables.at(0).pose.orientation);

        tables.tables.at(0).convex_hull.resize(4);
        tf::pointTFToMsg(p1, tables.tables.at(0).convex_hull.at(0));
        tf::pointTFToMsg(p2, tables.tables.at(0).convex_hull.at(1));
        tf::pointTFToMsg(p3, tables.tables.at(0).convex_hull.at(2));
        tf::pointTFToMsg(p4, tables.tables.at(0).convex_hull.at(3));

        pub_table.publish(tables);
    }
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl_msgs::ModelCoefficients> ("coefficients", 1);
    pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("plane_cloud", 1);
    pub_cloud_neg = nh.advertise<sensor_msgs::PointCloud2> ("plane_cloud_neg", 1);
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls","",false) );

    pub_table = nh.advertise<object_recognition_msgs::TableArray>("/table_array", 1);

    pick_pose_pub = nh.advertise<geometry_msgs::Point>("/pick_pose", 1);

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/kinect/depth_registered/points", 1, cloud_cb);

  // Spin
  ros::spin ();
}
