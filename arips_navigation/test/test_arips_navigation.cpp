//
// Created by jgdo on 22.03.21.
//

#include <gtest/gtest.h>

#include <arips_navigation/local_planner/VelocityPlanner.h>

struct VelocityPlannerTest: public ::testing::Test {

    VelocityPlanner planner {0.03, 0.03};
    geometry_msgs::Twist cmd_vel;
};

TEST_F(VelocityPlannerTest, Stop) {
    EXPECT_FALSE(planner.computeVelocity(0.01, 0.01, cmd_vel));
    EXPECT_EQ(cmd_vel.linear.x, 0);
    EXPECT_EQ(cmd_vel.angular.z, 0);
}

TEST_F(VelocityPlannerTest, Backward) {
    EXPECT_TRUE(planner.computeVelocity(-0.5, 0.00, cmd_vel));
    EXPECT_LT(cmd_vel.linear.x, 0);
    EXPECT_EQ(cmd_vel.angular.z, 0);

    EXPECT_TRUE(planner.computeVelocity(-0.5, -0.3, cmd_vel));
    EXPECT_LT(cmd_vel.linear.x, 0);
    EXPECT_GT(cmd_vel.angular.z, 0);

    EXPECT_TRUE(planner.computeVelocity(-0.5, 0.3, cmd_vel));
    EXPECT_LT(cmd_vel.linear.x, 0);
    EXPECT_LT(cmd_vel.angular.z, 0);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

