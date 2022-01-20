#pragma once

#include "PlanningMath.h"

#include <optional>
#include <utility>

using CellIndex = Eigen::Vector2i;

struct GridMapGeometry {
    std::string frameId;
    Vector2d offset;
    double resolution;
};

template <class T> class GridMap {
public:
    [[nodiscard]] virtual int width() const = 0;

    [[nodiscard]] virtual int height() const = 0;

    [[nodiscard]] virtual const std::string& frameId() const = 0;
    [[nodiscard]] virtual double resolution() const = 0;

    [[nodiscard]] virtual Vector2d toWorld(const CellIndex& index) const = 0;
    [[nodiscard]] virtual std::optional<CellIndex> toMap(const Vector2d& point) const = 0;

    virtual GridMapGeometry geometry() const = 0;

    [[nodiscard]] virtual std::optional<T> at(CellIndex index) const = 0;

    std::optional<T> atPos(const Vector2d& pos) const {
        const auto index = toMap(pos);
        if(index) {
            return at(*index);
        }
        return {};
    }

    /*
    template <std::enable_if_t<std::is_floating_point<T>::value, bool> = true>
    T interpolate(const Vector2d& point) const {
        return T{}; // TODO
    }*/
};

using Costmap = GridMap<uint8_t>;

template <class T> class GridMapWithGeometry : public GridMap<T> {
public:
    explicit GridMapWithGeometry(GridMapGeometry  geo) : mGeometry{std::move(geo)} {}

    const std::string& frameId() const override { return mGeometry.frameId; };
    double resolution() const override { return mGeometry.resolution; }

    [[nodiscard]] Vector2d toWorld(const CellIndex& index) const override {
        return Vector2d{index.x(), index.y()} * mGeometry.resolution + mGeometry.offset;
    }

    std::optional<CellIndex> toMap(const Vector2d& point) const override {
        const CellIndex index{std::lround(point.x() / mGeometry.resolution),
                              std::lround(point.y() / mGeometry.resolution)};
        if (index.x() >= 0 && index.x() < GridMap<T>::width() && index.y() >= 0 &&
            index.y() < GridMap<T>::height()) {
            return index;
        }
        return {};
    }

    GridMapGeometry geometry() const override {
        return mGeometry;
    }

protected:
    GridMapGeometry mGeometry;
};
