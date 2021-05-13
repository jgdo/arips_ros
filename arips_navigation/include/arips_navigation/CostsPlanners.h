#pragma once

#include "FlatGroundModule.h"

#include <navfn/navfn_ros.h>

namespace toponav_ros {

class AripsFlatPlanner : public FlatGroundModule::CostsPlanner {
public:
    AripsFlatPlanner(costmap_2d::Costmap2DROS& globalCostmap,
                     navfn::NavfnROS& planner, tf2_ros::Buffer& tfBuffer);

    bool makePlan(const geometry_msgs::PoseStamped& start, ApproachExit3DPtr const& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan, double* costs,
                  tf2::Stamped<tf2::Transform>* actualApproachPose) override;

    costmap_2d::Costmap2DROS& getMap() override;

protected:
    costmap_2d::Costmap2DROS& _costmap;
    navfn::NavfnROS& _planner;
    tf2_ros::Buffer& mTfBuffer;
};

} // namespace toponav_ros
