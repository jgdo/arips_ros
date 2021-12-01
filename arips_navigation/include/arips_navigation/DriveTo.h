#pragma once

#include <arips_navigation/DrivingState.h>

#include <tf2/LinearMath/Transform.h>

class Locomotion;

class DriveTo : public DrivingStateProto {
public:
    DriveTo(NavigationContext& context, Locomotion& locomotion);
    ~DriveTo() override;

    /**
     * Plan and drive drive to path. Will become active on success
     * @param goal must be directly reachable from current node
     * @return true if planning was successful and driving can start, otherwise false
     */
    bool driveTo(tf2::Stamped<tf2::Transform> const& goal);

    /**
     *
     * @param goal must be directly reachable from current node
     * @return costs if planning successful
     */
    // std::optional<double> planTo(tf2::Stamped<tf2::Transform> const& goal);

    bool isActive() override;
    void runCycle() override;

private:
    struct Pimpl;
    friend struct Pimpl;
    std::unique_ptr<Pimpl> pimpl;

};
