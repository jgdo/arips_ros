#pragma once

#include <queue>
#include <vector>

#include <Eigen/Core>

#include "CostFunction.h"
#include "Costmap.h"
#include "PlanningMath.h"
#include "PotentialMap.h"

class DijkstraPotentialComputation {
public:
    explicit DijkstraPotentialComputation(const CostFunction& costFunction);

    struct CellEntry {
        PotentialMap::Cell* cell;
        CellIndex index;

        inline bool operator<(const CellEntry& other) const {
            return this->cell->goalDist > other.cell->goalDist;
        }
    };

    PotentialMap computeDijkstra(const Costmap& costmap, const Pose2D& goal);

    const CostFunction& costFunction() const { return mCostFunction; }

    /*
    [[nodiscard]] std::optional<double> getGoalDistance(int x, int y) const {
        const auto dist = goalDist(x, y);
        if(dist < 0 || std::isnan(dist) || std::isinf(dist)) {
            return {};
        }

        return dist;
    } */

private:
    const CostFunction& mCostFunction;

    // using DistQueue = std::multimap<double, CellEntry>;
    using DistQueue = std::priority_queue<CellEntry>;
    DistQueue mDistQueue; /// cells to visit in future
    // std::unordered_map<Cell*, DistQueue::iterator>

    //    mLocationMap; /// location of a cell in the queue if any. TODO: consider using dense
    //    matrix
    /// if too slow

    /*
    inline const Cell& getCell(const CellIndex& index) const {
        return mMatrix(index.x(), index.y());
    }
    inline Cell& getCell(const CellIndex& index) { return mMatrix(index.x(), index.y()); }

    inline CellEntry getCellEntry(const CellIndex& index) {
        return {&mMatrix(index.x(), index.y()), index};
    }

     */

    static inline double obstacleCosts() {
        return -1.0F; // TODO
    }

    void insertCellIntoQueue(const CellIndex& index, PotentialMap& potmap);

    void computeTargetDistance(const Costmap& costmap, PotentialMap& potmap);

    CellEntry takeFirstFromQueue();

    void updatePathCell(const CellEntry& current_cell, const CellIndex& check_index,
                        const Costmap& costmap, PotentialMap& potmap, double dist);
};
