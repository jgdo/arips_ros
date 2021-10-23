#include <geometry_msgs/PoseStamped.h>
#include <global_planner/planner_core.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <arips_navigation/path_planning/Locomotion.h>
#include <arips_navigation/path_planning/PotentialMapVisualizer.h>

#include <visualization_msgs/MarkerArray.h>

#include <arips_navigation/utils/transforms.h>
#include <geometry_msgs/Twist.h>

#include <angles/angles.h>

#include "simulator/Simulator.h"

Pose2D fromStampedPose(geometry_msgs::PoseStamped const& msg) {
    return {{msg.pose.position.x, msg.pose.position.y}, getYawFromQuaternion(msg.pose.orientation)};
}

struct DemoNode {
    explicit DemoNode(tf2_ros::Buffer& tfBuffer) : tfBuffer{tfBuffer} {
        ros::NodeHandle nh;
        global_planner.initialize("global_planner", &costmap);
        sub = nh.subscribe("/topo_planner/nav_goal", 1, &DemoNode::poseCallback, this);
        pathPub = nh.advertise<nav_msgs::Path>("global_path", 1, true);
        gradPub = nh.advertise<visualization_msgs::Marker>("gradient", 1);
        potentialGradPub = nh.advertise<visualization_msgs::MarkerArray>("gradient_map", 1);
        cmdVelPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        loopTimer = nh.createTimer(ros::Rate(10), &DemoNode::doLoop, this);
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

        geometry_msgs::PoseStamped robotPose;
        if (!costmap.getRobotPose(robotPose)) {
            ROS_WARN_STREAM("Could not get robot pose");
            locomotion.cancel(); // make sure robot stops instead of using the old goal
            return;
        }

        locomotion.setGoal(fromStampedPose(robotPose), fromStampedPose(*goalPose));
    }

    void doLoop(const ros::TimerEvent& ev) {
        if (!locomotion.hasGoal()) {
            return;
        }

        geometry_msgs::PoseStamped robotPoseMsg;
        if (!costmap.getRobotPose(robotPoseMsg)) {
            ROS_WARN_STREAM("Could not get robot pose");
            locomotion.cancel();
            return;
        }
        const auto robotPose = fromStampedPose(robotPoseMsg);

        geometry_msgs::Twist cmdVel;
        if (locomotion.goalReached(robotPose)) {
            locomotion.cancel();
        } else {
            const auto optTwist = locomotion.computeVelocityCommands(robotPose);
            if (optTwist) {
                cmdVel.linear.x = optTwist->x();
                cmdVel.linear.y = optTwist->y();
                cmdVel.angular.z = optTwist->theta;
            } else {
                ROS_WARN("Could not compute velocity command");
            }
        }
        cmdVelPub.publish(cmdVel);

        // publish path, ignore orientation for now

        nav_msgs::Path path;
        path.header.stamp = ros::Time::now();
        path.header.frame_id = costmap.getGlobalFrameID();
        auto pathPose = robotPoseMsg;
        path.poses.push_back(pathPose);
        unsigned int cx, cy;
        if (costmap.getCostmap()->worldToMap(robotPose.x(), robotPose.y(), cx, cy)) {
            const auto& potentialMap = locomotion.potentialMap();

            CellIndex index{cx, cy};
            while (potentialMap.findNeighborLowerCost(index)) {
                costmap.getCostmap()->mapToWorld(index.x(), index.y(), pathPose.pose.position.x,
                                                 pathPose.pose.position.y);

                path.poses.push_back(pathPose);
            }

            pathPub.publish(path);


            visualization_msgs::MarkerArray markerArray;
            visualization_msgs::Marker marker;
            marker.header.frame_id = costmap.getGlobalFrameID();
            marker.header.stamp = ros::Time();
            marker.ns = "potential";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::LINE_LIST;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1;
            marker.scale.x = 0.001;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;

            for (int x = cx - 30; x < cx + 30; x++) {
                for (int y = cy - 30; y < cy + 30; y++) {
                    const auto optGrad = potentialMap.getGradient({x, y});
                    if (!optGrad) {
                        continue;
                    }

                    double wx, wy;
                    costmap.getCostmap()->mapToWorld(x, y, wx, wy);
                    geometry_msgs::Point p;
                    p.x = wx;
                    p.y = wy;
                    marker.points.push_back(p);

                    p.x += cos(*optGrad) * costmap.getCostmap()->getResolution();
                    p.y += sin(*optGrad) * costmap.getCostmap()->getResolution();

                    marker.points.push_back(p);
                }
            }

            markerArray.markers.push_back(marker);
            potentialGradPub.publish(markerArray);

            mapViz.visualizeMap(potentialMap);
        } else {
            ROS_WARN_STREAM("robot coordinates not inside costmap");
        }
    }

    tf2_ros::Buffer& tfBuffer;
    costmap_2d::Costmap2DROS costmap{"global_costmap", tfBuffer};
    PotentialMapVisualizer mapViz{"global_potential"};
    ros::Subscriber sub;
    ros::Publisher pathPub;
    ros::Publisher gradPub;
    ros::Publisher potentialGradPub;
    global_planner::GlobalPlanner global_planner;
    ros::Publisher cmdVelPub;

    Locomotion locomotion{costmap};

    ros::Timer loopTimer;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "demo_global_planning");

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener{tfBuffer};

    Simulator simulator{tfBuffer};
    DemoNode demo{tfBuffer};

    while (ros::ok()) {
        ros::spin();
    }
}
