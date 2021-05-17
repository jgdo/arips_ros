#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <navfn/navfn_ros.h>

#include <arips_navigation/path_planning/PotentialMap.h>
#include <arips_navigation/path_planning/PotentialMapVisualizer.h>

struct DemoNode {
    DemoNode() {
        ros::NodeHandle nh;
        navfn.initialize("navfn", &costmap);
        sub = nh.subscribe("/topo_planner/nav_goal", 1, &DemoNode::poseCallback, this);
    }

    void poseCallback(const geometry_msgs::PoseStamped& msg) {
        if (msg.header.frame_id != costmap.getGlobalFrameID()) {
            ROS_WARN_STREAM("Expected goal to be in frame '" << costmap.getGlobalFrameID()
                                                             << "', got '" << msg.header.frame_id
                                                             << "'");
            return;
        }

        unsigned int cx = 0, cy = 0;
        if (!costmap.getCostmap()->worldToMap(msg.pose.position.x, msg.pose.position.y, cx, cy)) {
            ROS_WARN_STREAM("Recieved goal coordinates not inside costmap");
            return;
        }

        potentialMap.computeDijkstra({cx, cy});

        const auto begin = ros::WallTime::now();
        navfn.computePotential(msg.pose.position);
        const auto end = ros::WallTime::now();
        ROS_INFO_STREAM("Navfn potential took " << (end-begin));

        mapViz.visualizeMap(potentialMap);
    }

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener{tfBuffer};
    costmap_2d::Costmap2DROS costmap{"global_costmap", tfBuffer};
    PotentialMap potentialMap{costmap};
    PotentialMapVisualizer mapViz{"global_potential"};
    ros::Subscriber sub;
    navfn::NavfnROS navfn;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "demo_global_planning");

    DemoNode demo;

    while (ros::ok()) {
        ros::spin();
    }
}
