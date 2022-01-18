#pragma once

#include "PlanningMath.h"

#include <optional>

using CellIndex = Eigen::Vector2i;

template<class T>
class GridMap {
public:
    [[nodiscard]] virtual const std::string& frameId() const = 0;

    [[nodiscard]] virtual T at(CellIndex index) const = 0;

    [[nodiscard]] virtual Vector2d toWorld(CellIndex index) const = 0;

    [[nodiscard]] virtual std::optional<CellIndex> toMap(const Vector2d& point) const = 0;

    [[nodiscard]] virtual double resolution() const = 0;

    /*
    template <std::enable_if_t<std::is_floating_point<T>::value, bool> = true>
    T interpolate(const Vector2d& point) const {
        return T{}; // TODO
    }*/

};

class NavMapView {
public:
    [[nodiscard]] virtual const std::string& frameId() const = 0;

    [[nodiscard]] std::optional<double> gradient(const Vector2d& point) const {
        return {}; // TODO
    }

    [[nodiscard]] virtual std::optional<double> goalDistance(const Vector2d& point) const = 0;

    [[nodiscard]] virtual std::optional<uint8_t> cost(const Vector2d& index) const = 0;

    [[nodiscard]] virtual const CostFunction& costFunction() const = 0;
};
