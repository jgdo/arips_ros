//
// Created by jgdo on 2/28/21.
//

#pragma once

#include "DrivingState.h"

#include <ros/ros.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <memory>

class OpenDoor: public DrivingStateProto
{
public:
    OpenDoor(tf2_ros::Buffer& tf, ros::Publisher& cmdVelPub, costmap_2d::Costmap2DROS& costmap);
    ~OpenDoor() override;

    void init();

    bool isActive() override;

    void runCycle() override;

private:
    struct Pimpl;
    std::unique_ptr<Pimpl> pimpl;

    friend class Pimpl;
};



