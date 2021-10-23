#pragma once

#include "PlanningMath.h"

#include <optional>

using CellIndex = Eigen::Vector2i;

struct GridCostmap {
    const std::string& frameId() const;

    uint8_t operator[](CellIndex index) const;

    Vector2d toWorld(CellIndex index) const;

    std::optional<CellIndex> toMap(Vector2d point) const;

    double resolution() const;
};