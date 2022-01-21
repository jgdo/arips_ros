#pragma once

#include "FlatGroundModule.h"
#include "NavigationContext.h"

#include <arips_navigation/path_planning/Locomotion.h>

namespace toponav_ros {

class AripsFlatPlanner : public FlatGroundModule::CostsPlanner {
public:
    explicit inline AripsFlatPlanner(NavigationContext& context, Locomotion& locomotion)
        : mContext{context}, mLocomotion{locomotion} {}

    std::optional<double> computeCosts(const geometry_msgs::PoseStamped& start,
                                       ApproachExit3DPtr const& goal,
                                       tf2::Stamped<tf2::Transform>* actualApproachPose, std::vector<geometry_msgs::PoseStamped>* path) override;

    const costmap_2d::Costmap2DROS& getMap() override;

protected:
    NavigationContext& mContext;
    Locomotion& mLocomotion;
};

} // namespace toponav_ros
