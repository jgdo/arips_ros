#pragma once

#include <arips_navigation/DrivingState.h>
#include <costmap_2d/costmap_2d_ros.h>

/**
 * Dock into the charging station 
 **/
class AutoDocker: public DrivingState {
public:
    AutoDocker(costmap_2d::Costmap2DROS& costmap, ros::Publisher& cmdVelPub);
  
    bool isActive() override;

    void runCycle() override;

private:
    costmap_2d::Costmap2DROS& mCostmap;
    ros::Publisher mCmdVelPub;

    bool mIsDocking = true;
};
