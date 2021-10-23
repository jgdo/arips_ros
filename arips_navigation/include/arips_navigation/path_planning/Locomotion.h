#pragma once

#include <optional>

#include <costmap_2d/costmap_2d_ros.h>

#include "PlanningMath.h"
#include "PotentialMap.h"

class Locomotion {
public:
    // costmap lifetime must be valid for this object's lifetime
    explicit Locomotion(costmap_2d::Costmap2DROS& costmap);

    ~Locomotion();

    // goal must be in costmaps global frame
    // will block until planning finished
    // return true if planning successful
    bool setGoal(const Pose2D& robotPose, const Pose2D& goal);

    [[nodiscard]] bool hasGoal() const;

    // will return true if no goal is set
    bool goalReached(const Pose2D& robotPose);

    // If nullopt returned, no valid velocity could be sampled
    // If goal is reached, zero velocity is returned.
    std::optional<Twist2D> computeVelocityCommands(const Pose2D& robotPose);

    // cancel navigation
    void cancel();

    const PotentialMap& potentialMap() const;

private:
    struct Pimpl;
    std::unique_ptr<Pimpl> mPimpl;
};
