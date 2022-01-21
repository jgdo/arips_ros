#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <costmap_2d/costmap_2d_ros.h>

#include <arips_navigation/ScoreTwist.h>
#include <arips_navigation/path_planning/MotionController.h>
#include <arips_navigation/path_planning/PlanningMath.h>
#include "arips_navigation/path_planning/Costmap2dView.h"

class TwistScorer {
public:
    explicit TwistScorer(tf2_ros::Buffer& tf) : tf{tf} {
        ros::NodeHandle nh;
        service = nh.advertiseService("score_twist", &TwistScorer::scoreTwist, this);
    }

    bool scoreTwist(arips_navigation::ScoreTwist::Request& req,
                    arips_navigation::ScoreTwist::Response& res) {
        geometry_msgs::PoseStamped robotPose;
        if (!costmap.getRobotPose(robotPose)) {
            ROS_WARN_STREAM("Could not get robot pose");
            return false;
        }

        const auto traj =
            generateTrajectory(Pose2D::fromMsg(robotPose.pose), Twist2D::fromMsg(req.twist),
                               req.sampleDuration, req.sampleDt);
        const auto cost = scoreTrajectory(traj);

        res.cost = cost ? *cost : -1;
        return true;
    }

    [[nodiscard]] std::optional<float> scoreTrajectory(const Trajectory& traj) const {
        if (traj.empty()) {
            return {};
        }

        const auto frontPos = traj.back().pose.point;
        unsigned int cx, cy;
        if (!costmap.getCostmap()->worldToMap(frontPos.x(), frontPos.y(), cx, cy)) {
            return {};
        }

        double trajectoryCost = 0;
        for (size_t i = 1; i < traj.size(); i++) {
            const auto& a = traj.at(i - 1);
            const auto& b = traj.at(i);

            if (!costmap.getCostmap()->worldToMap(a.pose.point.x(), a.pose.point.y(), cx, cy)) {
                return {};
            }

            const auto cellCost = costmap.getCostmap()->getCost(cx, cy);
            if (!Costmap2dView::isValidCellCost(cellCost)) {
                return {};
            }

            const auto meterDist = (b.pose.point - a.pose.point).norm();
            const auto dCost = meterDist * cellCost;
            const auto rCost =
                0; // std::abs(angles::shortest_angular_distance(a.pose.theta, b.pose.theta));
            trajectoryCost += dCost + rCost;
        }

        return trajectoryCost;
    }

private:
    tf2_ros::Buffer& tf;
    costmap_2d::Costmap2DROS costmap{"local_costmap", tf};
    ros::ServiceServer service;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "collision_costs_service");

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener{tfBuffer};

    TwistScorer scorer{tfBuffer};
    ROS_INFO("Ready compute twist collision costs");
    ros::spin();
}
