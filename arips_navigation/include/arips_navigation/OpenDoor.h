//
// Created by jgdo on 2/28/21.
//

#pragma once

#include "DrivingState.h"

#include <arips_navigation/DriveUntilCollision.h>

#include <ros/ros.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <memory>

class OpenDoor: public DrivingState
{
public:
    explicit OpenDoor(NavigationContext& context, DriveUntilCollision& driveUntilCollision);
    ~OpenDoor() override;

    void init();

    bool isActive() override;

    void runCycle() override;

private:
    struct Pimpl;
    std::unique_ptr<Pimpl> pimpl;

    friend class Pimpl;
};



