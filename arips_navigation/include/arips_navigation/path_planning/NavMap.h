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

class ComposedNavMap : NavMap {
public:
    ComposedNavMap(std::unique_ptr<PotentialMap> potmap, std::unique_ptr<Costmap> costmap)
        : mPotentialMap{std::move(mPotentialMap)}, mCostmap{std::move(costmap)} {}

    const std::string& frameId() const override;

    std::optional<double> goalDistance(const Vector2d& point) const override;
    std::optional<uint8_t> cost(const Vector2d& point) const override;
    const CostFunction& costFunction() const override;
    double lowestResolution() const override;

private:
    std::unique_ptr<PotentialMap> mPotentialMap;
    std::unique_ptr<Costmap> mCostmap;
};
