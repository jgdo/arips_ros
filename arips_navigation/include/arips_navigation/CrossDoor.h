#pragma once

#include <arips_navigation/CrossDoorInformation.h>

#include "DrivingState.h"
#include "TopoExecuter.h"
#include "OpenDoor.h"

class CrossDoor: public DrivingState {
public:
    CrossDoor(tf2_ros::Buffer& tf, ros::Publisher& cmdVelPub, TopoExecuter& topoExec, OpenDoor& openDoor);

    void activate(arips_navigation::CrossDoorInformation const& doorInfo);

    ~CrossDoor() override;

    bool isActive() override;

    void runCycle() override;

protected:
    class Pimpl;
    std::unique_ptr<Pimpl> pimpl;
};
