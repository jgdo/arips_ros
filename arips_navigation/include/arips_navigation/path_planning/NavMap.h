#pragma once

#include <optional>

#include "CostFunction.h"
#include "PlanningMath.h"
#include "PotentialMap.h"

class NavMap {
public:
    [[nodiscard]] virtual std::string frameId() const = 0;

    [[nodiscard]] virtual std::optional<double> gradient(const Vector2d& point) const = 0;

    [[nodiscard]] virtual std::optional<double>
    interpolateGoalDistance(const Vector2d& point) const = 0;

    [[nodiscard]] virtual std::optional<uint8_t> cost(const Vector2d& point) const = 0;

    [[nodiscard]] virtual const CostFunction& costFunction() const = 0;

    [[nodiscard]] virtual std::vector<FloorStep> floorSteps() const { return {}; }

    virtual double lowestResolution() const = 0;
};

class ComposedNavMap : public NavMap {
public:
    ComposedNavMap(const PotentialMap& potmap, const Costmap& costmap)
        : mPotentialMap{potmap}, mCostmap{costmap} {
        if (mPotentialMap.frameId() != mCostmap.frameId()) {
            throw std::runtime_error(
                "Cannot create ComposedNavMap: potmap frame id '" + mPotentialMap.frameId() +
                "' is different from costmap frame id '" + mCostmap.frameId() + '\'');
        }
    }

    std::string frameId() const override;

    std::optional<double> interpolateGoalDistance(const Vector2d& point) const override;
    std::optional<uint8_t> cost(const Vector2d& point) const override;
    const CostFunction& costFunction() const override;
    double lowestResolution() const override;

    std::optional<double> gradient(const Vector2d& point) const override {
        if (const auto index = mPotentialMap.toMap(point)) {
            return mPotentialMap.getGradient(*index);
        }
        return {};
    }

    [[nodiscard]] std::vector<FloorStep> floorSteps() const override { return mCostmap.floorSteps(); }

private:
    const PotentialMap& mPotentialMap;
    const Costmap& mCostmap;
};
