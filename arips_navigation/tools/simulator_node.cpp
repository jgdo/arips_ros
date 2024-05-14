#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include "simulator/Simulator.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "simulator_node");

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener{tfBuffer};

    Simulator simulator{tfBuffer};

    while (ros::ok()) {
        ros::spin();
    }
}