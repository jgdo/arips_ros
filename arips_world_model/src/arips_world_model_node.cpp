#include <arips_world_model/AripsWorldModel.h>

#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "arips_world_model");

    AripsWorldModel worldModel;

    while (ros::ok()) {
        ros::spin();
    }
}
