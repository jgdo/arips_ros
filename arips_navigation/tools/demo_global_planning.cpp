#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <global_planner/planner_core.h>

#include <arips_navigation/path_planning/PotentialMap.h>
#include <arips_navigation/path_planning/PotentialMapVisualizer.h>

#include "simulator/Simulator.h"

struct DemoNode {
    explicit DemoNode(tf2_ros::Buffer& tfBuffer) : tfBuffer{tfBuffer} {
        ros::NodeHandle nh;
        global_planner.initialize("global_planner", &costmap);
        sub = nh.subscribe("/topo_planner/nav_goal", 1, &DemoNode::poseCallback, this);
        pathPub = nh.advertise<nav_msgs::Path>("global_path", 1, true);

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

        unsigned int cx = 0, cy = 0;
        if (!costmap.getCostmap()->worldToMap(goalMsg.pose.position.x, goalMsg.pose.position.y, cx, cy)) {
            ROS_WARN_STREAM("Recieved goal coordinates not inside costmap");
            return;
        }

        {
            {
                const auto begin = ros::WallTime::now();
                potentialMap.computeDijkstra({cx, cy});
                const auto end = ros::WallTime::now();
                ROS_INFO_STREAM("My potential map potential took ms "
                                    << (end - begin).toSec() * 1000);
            }

            // publish path, ignore orientation for now
            geometry_msgs::PoseStamped robotPose;
            if (costmap.getRobotPose(robotPose)) {
                const auto startPose = robotPose;
                nav_msgs::Path path;
                path.header.stamp = ros::Time::now();
                path.header.frame_id = costmap.getGlobalFrameID();
                path.poses.push_back(robotPose);
                if (costmap.getCostmap()->worldToMap(robotPose.pose.position.x,
                                                     robotPose.pose.position.y, cx, cy)) {
                    CellIndex index{cx, cy};
                    while (potentialMap.findNeighborLowerCost(index)) {
                        costmap.getCostmap()->mapToWorld(index.x(), index.y(),
                                                         robotPose.pose.position.x,
                                                         robotPose.pose.position.y);

                        path.poses.push_back(robotPose);
                    }

                    pathPub.publish(path);
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
    PotentialMap potentialMap{costmap};
    PotentialMapVisualizer mapViz{"global_potential"};
    ros::Subscriber sub;
    ros::Publisher pathPub;
    global_planner::GlobalPlanner global_planner;

    ros::Timer loopTimer;

    geometry_msgs::PoseStamped goalMsg;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "demo_global_planning");

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener{tfBuffer};

    Simulator simulator {tfBuffer};
    DemoNode demo{tfBuffer};

    while (ros::ok()) {
        ros::spin();
    }
}
