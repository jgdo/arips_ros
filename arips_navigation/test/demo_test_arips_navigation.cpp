#include <arips_navigation/DriveUntilCollision.h>

#include <gtest/gtest.h>

TEST(DriveUntilCollision, DriveUntilCollision) {
    NavigationContext context;

    // ros::spin();

    DriveUntilCollision driveUntilCollision {context};

    geometry_msgs::Twist cmdVel;
    cmdVel.linear.x = 0.1;
    cmdVel.angular.z = 0.0;
    driveUntilCollision.activate(cmdVel, 0.1, ros::Duration(1000000));

    ros::Rate rate {10};
    while (driveUntilCollision.isActive()) {
        driveUntilCollision.runCycle();
        ros::spinOnce();
        rate.sleep();
    }
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    ros::init(argc, argv, "planning_context");
    ros::NodeHandle nh;

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
