#pragma once

#include <arips_navigation/DrivingState.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Transform.h>

#include <nav_core/base_local_planner.h>
#include <toponav_core/TopoMap.h>

#include <arips_navigation/FlatNavigationConfig.h>
#include <dynamic_reconfigure/server.h>

class DriveTo : public DrivingStateProto {
public:
    DriveTo(tf2_ros::Buffer& tf, ros::Publisher& cmdVelPub, toponav_core::TopoMapPtr topoMap,
            costmap_2d::Costmap2DROS& localCostmap);
    ~DriveTo() override = default;

    /**
     * Plan and drive drive to path. Will become active on success
     * @param goal must be directly reachable from current node
     * @return true if planning was successful and driving can start, otherwise false
     */
    bool driveTo(tf2::Stamped<tf2::Transform> const& goal);

    /**
     *
     * @param goal must be directly reachable from current node
     * @return Path if planning was successful otherwise false
     */
    std::vector<geometry_msgs::PoseStamped> planTo(tf2::Stamped<tf2::Transform> const& goal);

    /**
     * Drive according to path. Will become active if path not empty
     * @return true if planning was successful and driving can start, otherwise false
     */
    bool followPath(std::vector<geometry_msgs::PoseStamped> const& path);

    bool isActive() override;
    void runCycle() override;

private:
    toponav_core::TopoMapPtr mTopoMap;

    std::unique_ptr<nav_core::BaseLocalPlanner> mLocalPlanner;
    costmap_2d::Costmap2DROS& mLocalCostmap;

    std::vector<geometry_msgs::PoseStamped> mCurrentPath;

    arips_navigation::FlatNavigationConfig mConfig;
    dynamic_reconfigure::Server<arips_navigation::FlatNavigationConfig> mConfigServer{
        ros::NodeHandle{"~/FlatNavigation"}};

    ros::Time mLastControllerSuccessfulTime {0};

    void onDynamicReconfigure(arips_navigation::FlatNavigationConfig &config, uint32_t level);

    void doRecovery();
};
