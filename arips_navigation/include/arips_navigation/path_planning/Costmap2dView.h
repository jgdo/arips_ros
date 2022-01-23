#pragma once

#include <costmap_2d/costmap_2d_ros.h>

#include "Costmap.h"

struct Costmap2dView : public Costmap {
    const costmap_2d::Costmap2DROS& mCostmap;
    boost::unique_lock<boost::recursive_mutex> lock;

    [[nodiscard]] static bool isValidCellCost(uint8_t cellCost) {
        return cellCost != costmap_2d::LETHAL_OBSTACLE &&
               cellCost != costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
               cellCost != costmap_2d::NO_INFORMATION;
    }

    explicit Costmap2dView(costmap_2d::Costmap2DROS const& costmap, std::optional<Pose2D> robotPose = {})
        : mCostmap{costmap}, lock{*mCostmap.getCostmap()->getMutex()} {
        if(robotPose) {
            unsigned int x, y;
            if (mCostmap.getCostmap()->worldToMap(robotPose->x(), robotPose->y(), x, y)) {
                if(!isValidCellCost(mCostmap.getCostmap()->getCost(x, y))) {
                    mCostmap.getCostmap()->setCost(x, y, 252);
                }
            }
        }
    }

    int width() const override { return (int)mCostmap.getCostmap()->getSizeInCellsX(); }
    int height() const override { return (int)mCostmap.getCostmap()->getSizeInCellsY(); }
    std::string frameId() const override { return mCostmap.getGlobalFrameID(); }
    double resolution() const override { return mCostmap.getCostmap()->getResolution(); }
    Vector2d toWorld(const CellIndex& index) const override {
        double x, y;
        mCostmap.getCostmap()->mapToWorld(index.x(), index.y(), x, y);
        return {x, y};
    }
    std::optional<CellIndex> toMap(const Vector2d& point) const override {
        unsigned int x, y;
        if (mCostmap.getCostmap()->worldToMap(point.x(), point.y(), x, y)) {
            return CellIndex{x, y};
        }
        return {};
    }
    GridMapGeometry geometry() const override {
        return GridMapGeometry{
            frameId(),
            {mCostmap.getCostmap()->getOriginX(), mCostmap.getCostmap()->getOriginY()},
            resolution()};
    }

    std::optional<uint8_t> at(CellIndex index) const override {
        if (index.x() < 0 || index.x() >= width() || index.y() < 0 || index.y() >= height()) {
            return {};
        }

        const auto val = mCostmap.getCostmap()->getCost(index.x(), index.y());
        if (!isValidCellCost(val)) {
            return {};
        }

        return val;
    }
};
