#pragma once

#include "FlatGroundModule.h"

#include <navfn/navfn_ros.h>

namespace toponav_ros {

class NavfnCostsPlanner : public FlatGroundModule::CostsPlanner {
public:
    NavfnCostsPlanner(std::shared_ptr<costmap_2d::Costmap2DROS> const &costmap,
                      std::shared_ptr<navfn::NavfnROS> const &planner, std::string const &name,
                      tf2_ros::Buffer& tfBuffer);

    bool makePlan(const geometry_msgs::PoseStamped &start, ApproachExit3DPtr const &goal,
                  std::vector<geometry_msgs::PoseStamped> &plan, double *costs,
                  tf2::Stamped<tf2::Transform> *actualApproachPose) override;

    costmap_2d::Costmap2DROS const &getMap() override;

    std::string getMapName() const override;

protected:
    std::shared_ptr<costmap_2d::Costmap2DROS> _costmap;
    std::shared_ptr<navfn::NavfnROS> _planner;

    std::string mapName_;

    tf2_ros::Buffer& mTfBuffer;
};

} // namespace toponav_ros
