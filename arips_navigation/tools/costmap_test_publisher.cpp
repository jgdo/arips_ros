#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <nav_msgs/OccupancyGrid.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "costmap_test_publisher");
    ros::NodeHandle nh;

    tf2_ros::TransformBroadcaster br;

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "my_base";
    transformStamped.transform.translation.x = 1.0;
    transformStamped.transform.translation.y = 0.5;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;

    ros::Publisher pub =
        nh.advertise<nav_msgs::OccupancyGrid>("/collision_costs_service/local_costmap/costmap", 2);

    nav_msgs::OccupancyGrid grid;
    grid.header = transformStamped.header;
    grid.info.width = 20;
    grid.info.height = 15;
    grid.info.resolution = 0.1;
    grid.info.origin.position.x = 0.3;
    grid.info.origin.position.y = 0.7;
    q.setRPY(0, 0, -M_PI_4);
    grid.info.origin.orientation.x = q.x();
    grid.info.origin.orientation.y = q.y();
    grid.info.origin.orientation.z = q.z();
    grid.info.origin.orientation.w = q.w();

    grid.data.resize(grid.info.width * grid.info.height, 0);
    for (int x = 0; x < grid.info.width / 2; x++) {
        grid.data.at(7 * grid.info.width + x) = 250u;
    }
    for (int y = 0; y < 7; y++) {
        grid.data.at(y * grid.info.width + grid.info.width / 2) = 250u;
    }

    ros::Rate rate(5);

    double yaw = 0.60;
    while (ros::ok()) {
        transformStamped.header.stamp = ros::Time::now();
        q.setRPY(0, 0, yaw);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        br.sendTransform(transformStamped);

        grid.header.stamp = transformStamped.header.stamp;
        pub.publish(grid);

        yaw += 0.02;

        ros::spinOnce();
        rate.sleep();
    }
}
