#include <arips_world_model/world_model.h>

#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "arips_world_model");

    WorldModel worldModel;

    while (ros::ok()) {
        ros::spin();
    }
}
