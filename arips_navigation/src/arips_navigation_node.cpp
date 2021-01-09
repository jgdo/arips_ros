//
// Created by jgdo on 1/3/21.
//

#include <ros/ros.h>
#include <arips_navigation/Navigation.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "arips_navigation_node");

    Navigation navigation;

    while (ros::ok()){
        ros::spin();
    }
}
