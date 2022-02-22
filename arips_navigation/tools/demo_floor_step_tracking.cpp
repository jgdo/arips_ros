#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <arips_navigation/utils/FloorStepTracker.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "arips_navigation_node");

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener{tfBuffer};

    FloorStepTracker tracker {tfBuffer, "odom"};

    while (ros::ok()) {
        ros::spin();
    }
}
