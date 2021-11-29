#pragma once

#include <arips_navigation/DrivingState.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Transform.h>

#include <nav_core/base_local_planner.h>
#include <toponav_core/TopoMap.h>

#include <arips_navigation/FlatNavigationConfig.h>
#include <dynamic_reconfigure/server.h>

#include <arips_navigation/path_planning/Locomotion.h>

class DriveTo : public DrivingStateProto {
public:
    DriveTo(NavigationContext& context, Locomotion& locomotion);
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
     * @return costs if planning successful
     */
    std::optional<double> planTo(tf2::Stamped<tf2::Transform> const& goal);

    bool isActive() override;
    void runCycle() override;

private:
    Locomotion& mLocomotion;

    arips_navigation::FlatNavigationConfig mConfig;
    dynamic_reconfigure::Server<arips_navigation::FlatNavigationConfig> mConfigServer{
        ros::NodeHandle{"~/FlatNavigation"}};

    ros::Time mLastControllerSuccessfulTime {0};

    void onDynamicReconfigure(arips_navigation::FlatNavigationConfig &config, uint32_t level);

    void doRecovery();
};
