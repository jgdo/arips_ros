#pragma once

#include <queue>
#include <vector>

#include <Eigen/Core>

#include <costmap_2d/costmap_2d_ros.h>

using CellIndex = Eigen::Vector2i;

class PotentialMap {
public:
    explicit PotentialMap(costmap_2d::Costmap2DROS& costmap2D);

    struct Cell {
        double goalDist = -1;
        bool visited = false; // whether already visited, e.g. costs cannot change
    };

    struct CellEntry {
        Cell* cell;
        CellIndex index;

        inline bool operator<(const CellEntry& other) const {
            return this->cell->goalDist > other.cell->goalDist;
        }
    };

    void computeDijkstra(const CellIndex& goal);

    [[nodiscard]] inline costmap_2d::Costmap2DROS& getCostmapRos() { return mCostmapRos; }

    [[nodiscard]] inline const costmap_2d::Costmap2DROS& getCostmapRos() const {
        return mCostmapRos;
    }

    [[nodiscard]] double getGoalDistance(int x, int y) const { return mMatrix(x, y).goalDist; }

    [[nodiscard]] bool findNeighborLowerCost(CellIndex& index) const;

    std::optional<double> getGradient(const CellIndex& index) const;

private:
    costmap_2d::Costmap2DROS& mCostmapRos;

    // index is (x, y), not (y, x) !!! Corresponds to costmap(x, y)
    typedef Eigen::Matrix<Cell, Eigen::Dynamic, Eigen::Dynamic> CellMatrix;
    CellMatrix mMatrix;

    // using DistQueue = std::multimap<double, CellEntry>;
    using DistQueue = std::priority_queue<CellEntry>;
    DistQueue mDistQueue; /// cells to visit in future
    // std::unordered_map<Cell*, DistQueue::iterator>

    //    mLocationMap; /// location of a cell in the queue if any. TODO: consider using dense
    //    matrix
    /// if too slow

    inline const Cell& getCell(const CellIndex& index) const { return mMatrix(index.x(), index.y()); }
    inline Cell& getCell(const CellIndex& index) { return mMatrix(index.x(), index.y()); }

    inline CellEntry getCellEntry(const CellIndex& index) {
        return {&mMatrix(index.x(), index.y()), index};
    }

    static inline double obstacleCosts() {
        return -1.0F; // TODO
    }

    void insertCellIntoQueue(const CellIndex& index);

    void computeTargetDistance(const costmap_2d::Costmap2D& costmap);

    CellEntry takeFirstFromQueue();

    void updatePathCell(const CellEntry& current_cell, const CellIndex& check_index,
                        const costmap_2d::Costmap2D& costmap, float dist);

    template<class M>
    std::optional<double> calcGradientWithMatrix(const CellIndex& index, const M& conv) const;
};
