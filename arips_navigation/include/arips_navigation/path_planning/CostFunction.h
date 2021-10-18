#pragma once

#include <cstdint>

#include <costmap_2d/costmap_2d_ros.h>

class CostFunction {
public:
    // having costmap cost of c compute the maximum speed of a wheel
    [[nodiscard]] double maxWheelSpeedFromCosts(uint8_t cellCost) const {
        const auto velocityScale = mObstacleSpeedScalingBase +
                                   mObstacleSpeedScalingFraction *
                                       ((mObstacleSpeedScalingLimit -
                                         std::min<double>(cellCost, mObstacleSpeedScalingLimit)) /
                                        mObstacleSpeedScalingLimit);

        return velocityScale * mMaxWheelSpeed;
    }

    [[nodiscard]] bool isValidCellCost(uint8_t cellCost) const {
        return cellCost != costmap_2d::LETHAL_OBSTACLE &&
               cellCost != costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
               cellCost != costmap_2d::NO_INFORMATION;
    }

    double maxWheelSpeed() const {
        return mMaxWheelSpeed;
    }

private:
    static constexpr double mMaxWheelSpeed = 0.4;

    static constexpr double mObstacleSpeedScalingLimit = 255.0;
    // this fraction of total speed is affected by obstacles
    static constexpr double mObstacleSpeedScalingFraction = 0.75;
    static constexpr double mObstacleSpeedScalingBase = 1.0 - mObstacleSpeedScalingFraction;
};