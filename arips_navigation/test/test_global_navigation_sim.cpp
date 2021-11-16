#include <gtest/gtest.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <arips_navigation/path_planning/Locomotion.h>
#include <arips_navigation/path_planning/PotentialMapVisualizer.h>

#include <visualization_msgs/MarkerArray.h>

#include <arips_navigation/utils/transforms.h>
#include <geometry_msgs/Twist.h>

#include "../tools/simulator/Simulator.h"
#include <angles/angles.h>

#include <thread>

// instantiate Locomotion and Simulator
// extend simulator to trigger simulation steps externally and get current robot pose
// run some navigations goals for a fixed amount of steps (at max simulation speed)

TEST(GlobalPlanning, GlobalPlanning) {
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener{tfBuffer};

    Simulator simulator{tfBuffer, false};
    simulator.makeStep(0.1, ros::Time::now());
    costmap_2d::Costmap2DROS costmap{"global_costmap", tfBuffer};
    Locomotion locomotion{costmap};

    const std::vector<Pose2D> waypoints = {
        {{4.483781337738037, 3.3605000972747803}, 0.0},
        {{8.784196853637695, -1.44256258}, 0.0},
        {{1.161303, -5.60}, 0.0},
        {{3.26448, 0.3883}, 0.0},
        {{-3.26448, 0.3883}, 0.0},
        {{0.0, 0.0}, 0.0},
    };

    for(const auto& wp: waypoints) {
        ASSERT_TRUE(locomotion.setGoal(simulator.getRobotPose(), wp));

        for (auto robotPose = simulator.getRobotPose(); !locomotion.goalReached(robotPose);
             robotPose = simulator.getRobotPose()) {
            const auto optVel = locomotion.computeVelocityCommands(robotPose);
            ASSERT_TRUE(optVel);
            simulator.setSpeed(*optVel);
            //ros::Rate rate(5000.0);
            for (int i = 0; i < 5; i++) {
                simulator.makeStep(0.02, ros::Time::now());
              //  rate.sleep();
            }
        }
    }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
    ros::init(argc, argv, "planning_context");
    ros::NodeHandle nh;

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
