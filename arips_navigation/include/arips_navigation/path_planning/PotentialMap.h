#pragma once

#include "Costmap.h"
#include "PlanningMath.h"
#include "CostFunction.h"

using CellIndex = Eigen::Vector2i;

class PotentialMap : public GridMapWithGeometry<double> {
    friend class DijkstraPotentialComputation;

public:
    PotentialMap(int width, int height, GridMapGeometry geo, CellIndex goal,
                 CostFunction costFunction)
        : GridMapWithGeometry{std::move(geo)}, mMatrix{CellMatrix::Constant(width, height, Cell{})},
          mGoal{goal}, mCostFunction{std::move(costFunction)} {}

    int width() const override { return mMatrix.rows(); };
    int height() const override { return mMatrix.cols(); }

    std::optional<double> at(CellIndex index) const override;

    [[nodiscard]] std::optional<CellIndex> findNeighborLowerCost(const CellIndex& index) const;
    [[nodiscard]] std::optional<double> getGradient(const CellIndex& index) const;
    std::vector<Pose2D> traceCurrentPath(const Pose2D& robotPose) const;

    CellIndex goal() const { return mGoal; }

    const CostFunction& costFunction() const {
        return mCostFunction;
    }

private:
    struct Cell {
        double goalDist = -1;
        bool visited = false; // whether already visited, e.g. costs cannot change
    };

    // index is (x, y), not (y, x) !!! Corresponds to costmap(x, y)
    typedef Eigen::Matrix<Cell, Eigen::Dynamic, Eigen::Dynamic> CellMatrix;
    CellMatrix mMatrix;

    CellIndex mGoal;

    CostFunction mCostFunction;

    Cell& cellAt(CellIndex index) { return mMatrix(index.x(), index.y()); }

    template <class M>
    std::optional<double> calcGradientWithMatrix(const CellIndex& index, const M& conv) const;
};