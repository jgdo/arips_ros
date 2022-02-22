#include <arips_navigation/path_planning/Locomotion.h>
#include <arips_navigation/path_planning/PotentialMapVisualizer.h>
#include <geometry_msgs/PoseStamped.h>
#include <global_planner/planner_core.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <arips_navigation/utils/transforms.h>
#include <geometry_msgs/Twist.h>

#include <angles/angles.h>
#include <nav_msgs/Odometry.h>

#include "arips_navigation/path_planning/Costmap2dView.h"
#include "arips_navigation/utils/FloorStepTracker.h"
#include "simulator/Simulator.h"

struct DemoNode {
    ros::Subscriber mTwistSub;
    ros::Subscriber mClickedPointSub;

    nav_msgs::Odometry mLastOdom;
    const ros::Rate mControlRate{10};

    explicit DemoNode(tf2_ros::Buffer& tfBuffer) : tfBuffer{tfBuffer} {
        ros::NodeHandle nh;
        sub = nh.subscribe("/topo_planner/nav_goal", 1, &DemoNode::poseCallback, this);
        cmdVelPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        loopTimer = nh.createTimer(mControlRate, &DemoNode::doLoop, this);

        mTwistSub = nh.subscribe<nav_msgs::Odometry>(
            "/odom", 1, [this](const nav_msgs::OdometryConstPtr& msg) { mLastOdom = *msg; });

        mClickedPointSub = nh.subscribe("/clicked_point", 1, &DemoNode::onPointClicked, this);
    }

    void poseCallback(const geometry_msgs::PoseStamped& msg) {
        if (msg.header.frame_id != costmap.getGlobalFrameID()) {
            ROS_WARN_STREAM("Expected goal to be in frame '" << costmap.getGlobalFrameID()
                                                             << "', got '" << msg.header.frame_id
                                                             << "'");
            return;
        }

        const auto goalPose = tryTransform(tfBuffer, msg, costmap.getGlobalFrameID());

        if (!goalPose) {
            ROS_WARN_STREAM("Could not transform goal from frame '"
                            << msg.header.frame_id << "' to '" << costmap.getGlobalFrameID()
                            << "'.");
            locomotion.cancel(); // make sure robot stops instead of using the old goal
            return;
        }

        geometry_msgs::PoseStamped robotPoseMsg;
        if (!costmap.getRobotPose(robotPoseMsg)) {
            ROS_WARN_STREAM("Could not get robot pose");
            locomotion.cancel(); // make sure robot stops instead of using the old goal
            return;
        }

        const auto robotPose = Pose2D::fromMsg(robotPoseMsg.pose);

        locomotion.setGoal(Costmap2dView{costmap, robotPose, mStepTracker.allSteps()}, robotPose,
                           Pose2D::fromMsg(goalPose->pose));

        if (locomotion.hasGoal()) {
            mapViz.showPath(*locomotion.potentialMap(), robotPose);
        }
    }

    void onPointClicked(const geometry_msgs::PointStamped& msg) const {
        if (const auto* potmap = locomotion.potentialMap()) {
            if (const auto index = potmap->toMap({msg.point.x, msg.point.y})) {
                const auto grad = potmap->getGradient(*index);
                if (grad) {
                    ROS_INFO_STREAM("GRADIENT is " << *grad);
                }
            }
        }
    }

    void doLoop(const ros::TimerEvent& ev) {
        ROS_INFO("############################ loop");
        mStepTracker.visualizeSteps();

        if (!locomotion.hasGoal()) {
            return;
        }

        geometry_msgs::PoseStamped robotPoseMsg;
        if (!costmap.getRobotPose(robotPoseMsg)) {
            ROS_WARN_STREAM("Could not get robot pose");
            locomotion.cancel();
            return;
        }
        const auto robotPose = Pose2D::fromMsg(robotPoseMsg.pose);

        geometry_msgs::Twist cmdVel;
        if (locomotion.goalReached(robotPose)) {
            locomotion.cancel();
        } else {
            const auto optTwist = locomotion.computeVelocityCommands(
                Costmap2dView{costmap, robotPose, mStepTracker.allSteps()},
                {robotPose, Pose2D::fromMsg(mLastOdom.twist.twist)},
                mControlRate.expectedCycleTime().toSec());
            if (optTwist) {
                cmdVel.linear.x = optTwist->x();
                cmdVel.linear.y = optTwist->y();
                cmdVel.angular.z = optTwist->theta;
            } else {
                ROS_WARN("Could not compute velocity command");
            }
        }
        cmdVelPub.publish(cmdVel);

        if (const auto* potmap = locomotion.potentialMap()) {
            mapViz.showMap(*potmap, robotPose);
        }
    }

    tf2_ros::Buffer& tfBuffer;
    costmap_2d::Costmap2DROS costmap{"global_costmap", tfBuffer};
    PotentialMapVisualizer mapViz;
    ros::Subscriber sub;
    ros::Publisher cmdVelPub;

    FloorStepTracker mStepTracker{
        tfBuffer, costmap.getGlobalFrameID(), {{{{2.74, -1.281}, {3.15, -2.2}}}}};

    Locomotion locomotion;

    ros::Timer loopTimer;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "arips_navigation_node");

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener{tfBuffer};

    std::unique_ptr<Simulator> simulator;
    if (ros::NodeHandle{"~"}.param("start_simulator", false)) {
        ROS_INFO("Starting global planning demo with simulator");
        simulator = std::make_unique<Simulator>(tfBuffer);
    } else {
        ROS_INFO("Starting global planning demo without simulator");
    }

    DemoNode demo{tfBuffer};

    while (ros::ok()) {
        ros::spin();
    }
}
