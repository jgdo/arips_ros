#pragma once

#include <ros/publisher.h>

#include "PotentialMap.h"

class PotentialMapVisualizer {
public:
    explicit PotentialMapVisualizer();

    void showGradients(const PotentialMap& map, const Pose2D& robotPose);
    void showPotential(const PotentialMap& map);
    void showPath(const PotentialMap& map, const Pose2D& robotPose);
    void showMap(const PotentialMap& map, const Pose2D& robotPose);

private:
    ros::Publisher mPub;
    ros::Publisher mPotentialGradPub;
    ros::Publisher mPathPub;
};
