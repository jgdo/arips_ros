#pragma once

#include <optional>

#include "CostFunction.h"
#include "PlanningMath.h"
#include "PotentialMap.h"

class NavMap {
public:
    [[nodiscard]] virtual const std::string& frameId() const = 0;

    [[nodiscard]] std::optional<double> gradient(const Vector2d& point) const {
        return {}; // TODO
    }

    [[nodiscard]] virtual std::optional<double> goalDistance(const Vector2d& point) const = 0;

    [[nodiscard]] virtual std::optional<uint8_t> cost(const Vector2d& index) const = 0;

    [[nodiscard]] virtual const CostFunction& costFunction() const = 0;

    virtual double lowestResolution() const = 0;
};

class ComposedNavMap : public NavMap {
public:
    ComposedNavMap(const PotentialMap& potmap, const Costmap& costmap)
        : mPotentialMap{mPotentialMap}, mCostmap{costmap} {
        if (mPotentialMap.frameId() != mCostmap.frameId()) {
            throw std::runtime_error(
                "Cannot create ComposedNavMap: potmap frame id '" + mPotentialMap.frameId() +
                "' is different from costmap frame id '" + mCostmap.frameId() + '\'');
        }
    }

    const std::string& frameId() const override;

    std::optional<double> goalDistance(const Vector2d& point) const override;
    std::optional<uint8_t> cost(const Vector2d& point) const override;
    const CostFunction& costFunction() const override;
    double lowestResolution() const override;

private:
    const PotentialMap& mPotentialMap;
    const Costmap& mCostmap;
};
