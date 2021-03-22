//
// Created by jgdo on 2/28/21.
//

#pragma once

#include "DrivingState.h"

#include <ros/ros.h>

#include <memory>

class OpenDoor: public DrivingStateProto
{
public:
    OpenDoor(tf2_ros::Buffer& tf, ros::Publisher& cmdVelPub);
    ~OpenDoor() override;

    void init();

    bool isActive() override;

    void runCycle() override;

private:
    struct Pimpl;
    std::unique_ptr<Pimpl> pimpl;

    friend class Pimpl;
};



