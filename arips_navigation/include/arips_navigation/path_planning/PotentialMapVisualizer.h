#pragma once

#include <ros/ros.h>

#include "PotentialMap.h"

class PotentialMapVisualizer {
public:
    explicit PotentialMapVisualizer(const std::string& name);

    void visualizeMap(const PotentialMap& map);
private:
    ros::Publisher mPub;
};
