#include <geometry_msgs/PoseStamped.h>
#include <global_planner/planner_core.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <arips_navigation/path_planning/PotentialMap.h>
#include <arips_navigation/path_planning/PotentialMapVisualizer.h>
#include <arips_navigation/path_planning/MotionController.h>

#include <visualization_msgs/MarkerArray.h>

#include <arips_navigation/utils/transforms.h>
#include <geometry_msgs/Twist.h>

#include <angles/angles.h>

#include "simulator/Simulator.h"

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

        goalMsg = msg;
    }



    void doLoop(const ros::TimerEvent& ev) {
        if (goalMsg.header.frame_id.empty()) {
            return;
        }

        unsigned int goalCx = 0, goalCy = 0;
        if (!costmap.getCostmap()->worldToMap(goalMsg.pose.position.x, goalMsg.pose.position.y, goalCx,
                                              goalCy)) {
            ROS_WARN_STREAM("Recieved goal coordinates not inside costmap");
            return;
        }

        {
            // publish path, ignore orientation for now
            geometry_msgs::PoseStamped robotPose;
            if (costmap.getRobotPose(robotPose)) {
                const auto startPose = robotPose;
                nav_msgs::Path path;
                path.header.stamp = ros::Time::now();
                path.header.frame_id = costmap.getGlobalFrameID();
                path.poses.push_back(robotPose);
                unsigned int cx, cy;
                if (costmap.getCostmap()->worldToMap(robotPose.pose.position.x,
                                                     robotPose.pose.position.y, cx, cy)) {

                    const auto initialRobotCost = costmap.getCostmap()->getCost(cx, cy);
                    costmap.getCostmap()->setCost(cx, cy, costmap_2d::FREE_SPACE);

                    {
                        const auto begin = ros::WallTime::now();
                        potentialMap.computeDijkstra({goalCx, goalCy});
                        const auto end = ros::WallTime::now();
                        ROS_INFO_STREAM("My potential map potential took ms "
                                            << (end - begin).toSec() * 1000);
                    }

                    // costmap.getCostmap()->setCost(cx, cy, initialRobotCost);

                    CellIndex index{cx, cy};
                    while (potentialMap.findNeighborLowerCost(index)) {
                        costmap.getCostmap()->mapToWorld(index.x(), index.y(),
                                                         robotPose.pose.position.x,
                                                         robotPose.pose.position.y);

                        path.poses.push_back(robotPose);
                    }

                    pathPub.publish(path);

                    geometry_msgs::Twist cmdVel;
                    if(motionController.goalReached(goalMsg)) {
                        goalMsg.header.frame_id.clear();
                    } else {
                        const auto optCdVel = motionController.computeVelocity(goalMsg);
                        if(optCdVel) {
                            cmdVel = *optCdVel;
                        } else {
                            ROS_WARN("Could not compute velocity command");
                        }
                    }
                    cmdVelPub.publish(cmdVel);


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
                } else {
                    ROS_WARN_STREAM("robot coordinates not inside costmap");
                }

                /*
                {
                    std::vector<geometry_msgs::PoseStamped> plan;
                    const auto begin = ros::WallTime::now();
                    global_planner.makePlan(startPose, goalMsg, plan);
                    const auto end = ros::WallTime::now();
                    ROS_INFO_STREAM("Navfn potential took ms " << (end - begin).toSec() * 1000);
                }
                */
            } else {
                ROS_WARN("Could not get robot pose");
            }
        }
        mapViz.visualizeMap(potentialMap);
    }

    tf2_ros::Buffer& tfBuffer;
    costmap_2d::Costmap2DROS costmap{"global_costmap", tfBuffer};
    CostFunction costFunction;
    PotentialMap potentialMap{costmap, costFunction};
    PotentialMapVisualizer mapViz{"global_potential"};
    ros::Subscriber sub;
    ros::Publisher pathPub;
    ros::Publisher gradPub;
    ros::Publisher potentialGradPub;
    global_planner::GlobalPlanner global_planner;
    MotionController motionController{potentialMap};
    ros::Publisher cmdVelPub;

    ros::Timer loopTimer;

    geometry_msgs::PoseStamped goalMsg;
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
