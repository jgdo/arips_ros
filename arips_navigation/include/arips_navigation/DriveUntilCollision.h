#pragma once

#include "DrivingState.h"
#include <memory>

class DriveUntilCollision: public DrivingState
{
public:
    explicit DriveUntilCollision(NavigationContext& context);
    ~DriveUntilCollision() override;

    // collisionDistance must be >= 0. When < 0, state becomes immediately finished
    void activate(geometry_msgs::Twist const& cmd_vel, double collisionDistance, ros::Duration timeout);

    bool isActive() override;

    void runCycle() override;

private:
    struct Pimpl;
    std::unique_ptr<Pimpl> pimpl;
};

