#pragma once

#include <optional>

#include "PlanningMath.h"
#include "DijkstraPotentialComputation.h"

class Locomotion {
public:
    // costmap lifetime must be valid for this object's lifetime
    explicit Locomotion();

    ~Locomotion();

    // Return costs on success, otherwise nullopt
    // std::optional<double> makePlan(const Costmap& costmap, const Pose2D& robotPose, const Pose2D& goal);

    // goal must be in costmaps global frame
    // will block until planning finished
    // return true if planning successful
    bool setGoal(const Costmap& costmap, const Pose2D& robotPose, const Pose2D& goal);

    [[nodiscard]] std::optional<Pose2D> currentGoal() const;

    [[nodiscard]] inline bool hasGoal() const {
        return static_cast<bool>(currentGoal());
    }

    // will return true if no goal is set
    bool goalReached(const Pose2D& robotPose);

    // If nullopt returned, no valid velocity could be sampled
    // If goal is reached, zero velocity is returned.
    std::optional<Twist2D> computeVelocityCommands(const Costmap& costmap, const Odom2D& robotPose);

    // cancel navigation
    void cancel();

    // [[nodiscard]] const DijkstraPotentialComputation& potentialMap() const;

private:
    struct Pimpl;
    std::unique_ptr<Pimpl> mPimpl;
};
