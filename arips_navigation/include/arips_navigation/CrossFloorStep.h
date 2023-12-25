#pragma once

#include "DrivingState.h"
#include <memory>

#include <arips_navigation/path_planning/PlanningMath.h>

using FloorStep = std::array<Point2d, 2>;

class CrossFloorStep: public DrivingState
{
public:
    explicit CrossFloorStep(NavigationContext& context);
    ~CrossFloorStep() override;

    void activate(const FloorStep& step);

    bool isActive() override;
    void runCycle() override;

private:
    struct Pimpl;
    std::unique_ptr<Pimpl> pimpl;
};

