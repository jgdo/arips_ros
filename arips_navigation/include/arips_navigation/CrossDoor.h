#pragma once

#include <arips_navigation/CrossDoorInformation.h>

#include "DrivingState.h"
#include "DriveTo.h"
#include "OpenDoor.h"

class CrossDoor: public DrivingState {
public:
    CrossDoor(NavigationContext& ctx, DriveTo& driveTo, OpenDoor& openDoor);

    void activate(arips_navigation::CrossDoorInformation const& doorInfo);

    ~CrossDoor() override;

    bool isActive() override;

    void runCycle() override;

protected:
    class Pimpl;
    std::unique_ptr<Pimpl> pimpl;
};
