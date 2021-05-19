#include <arips_navigation/path_planning/PotentialMap.h>

#include <chrono>

PotentialMap::PotentialMap(costmap_2d::Costmap2DROS& costmap) : mCostmapRos(costmap) {}

void PotentialMap::computeDijkstra(const CellIndex& goal) {
    const auto begin = std::chrono::steady_clock::now();

    auto* costmap = mCostmapRos.getCostmap();

    // TODO check if gaol index in cell
    // TODO check if goal index not in collision

    mMatrix = CellMatrix::Constant(costmap->getSizeInCellsX(), costmap->getSizeInCellsY(), Cell{});
    // mDistQueue.swap({});
    // mLocationMap.clear();
    mDistQueue = DistQueue{};

    const auto endInit = std::chrono::steady_clock::now();
    const auto elapsedInitMs =
        std::chrono::duration_cast<std::chrono::milliseconds>(endInit - begin).count();

    ROS_INFO_STREAM("Global map dijkstra init took " << elapsedInitMs << " ms");

    auto& startCell = getCell(goal);
    startCell = {0.0, false};
    insertCellIntoQueue(goal);

    computeTargetDistance(*costmap);

    const auto end = std::chrono::steady_clock::now();
    const auto elapsedMs =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - endInit).count();

    ROS_INFO_STREAM("Global map dijkstra potential took " << elapsedMs << " ms");
}
void PotentialMap::insertCellIntoQueue(const CellIndex& index) {
    auto& cell = getCell(index);
    mDistQueue.push({&cell, index});
    // mLocationMap.insert({&cell, iter});
}

void PotentialMap::computeTargetDistance(const costmap_2d::Costmap2D& costmap) {
    const auto last_col = mMatrix.cols() - 1;
    const auto last_row = mMatrix.rows() - 1;

    /*
    using no = std::pair<CellIndex, double>; // neighbor offset
    const std::array<std::pair<CellIndex, double>, 8> neighborOffsets = {
        no {{-1, 0}, 1.0},
        no {{+1, 0}, 1.0},
        no {{0, -1}, 1.0},
        no {{0, +1}, 1.0},
        no {{-1, -1}, std::sqrt(2.0)},
        no {{-1, 1}, std::sqrt(2.0)},
        no {{1, 1}, std::sqrt(2.0)},
        no {{1, -1}, std::sqrt(2.0)},
    }; */

    while (!mDistQueue.empty()) {
        auto current_cell = takeFirstFromQueue();

        if (current_cell.cell->visited) {
            continue;
        }

        current_cell.cell->visited = true;

        if (current_cell.index.x() > 0) {
            updatePathCell(current_cell, current_cell.index + CellIndex{-1, 0}, costmap, 1);
        }

        if (current_cell.index.x() < last_col) {
            updatePathCell(current_cell, current_cell.index + CellIndex{+1, 0}, costmap, 1);
        }

        if (current_cell.index.y() > 0) {
            updatePathCell(current_cell, current_cell.index + CellIndex{0, -1}, costmap, 1);
        }

        if (current_cell.index.y() < last_row) {
            updatePathCell(current_cell, current_cell.index + CellIndex{0, +1}, costmap, 1);
        }

        if (current_cell.index.x() > 0 && current_cell.index.y() > 0) {
            updatePathCell(current_cell, current_cell.index + CellIndex{-1, -1}, costmap,
                           std::sqrt(2.0));
        }

        if (current_cell.index.x() > 0 && current_cell.index.y() < last_row) {
            updatePathCell(current_cell, current_cell.index + CellIndex{-1, +1}, costmap,
                           std::sqrt(2.0));
        }

        if (current_cell.index.x() < last_col && current_cell.index.y() > 0) {
            updatePathCell(current_cell, current_cell.index + CellIndex{+1, -1}, costmap,
                           std::sqrt(2.0));
        }

        if (current_cell.index.x() < last_col && current_cell.index.y() < last_row) {
            updatePathCell(current_cell, current_cell.index + CellIndex{+1, +1}, costmap,
                           std::sqrt(2.0));
        }
    }
}
PotentialMap::CellEntry PotentialMap::takeFirstFromQueue() {
    auto cellEntry = mDistQueue.top();
    mDistQueue.pop();
    return cellEntry;
}

void PotentialMap::updatePathCell(const PotentialMap::CellEntry& current_cell,
                                  const CellIndex& check_index,
                                  const costmap_2d::Costmap2D& costmap, float dist) {

    static constexpr double costmapFactor = 1 / 50.0; // TODO values

    auto& check_cell = getCell(check_index);
    if (check_cell.visited) {
        return;
    }

    // if the cell is an obstacle set the max path distance
    unsigned char check_cost = costmap.getCost(check_index.x(), check_index.y());
    if ((check_cost == costmap_2d::LETHAL_OBSTACLE ||
         check_cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
         check_cost == costmap_2d::NO_INFORMATION)) {
        check_cell.goalDist = obstacleCosts();
        check_cell.visited = true;
        return;
    }

    const double newGoalDist = current_cell.cell->goalDist + dist + check_cost * costmapFactor;
    if (check_cell.goalDist < 0 || newGoalDist < check_cell.goalDist) {
        check_cell.goalDist = newGoalDist;

        // erase cell from queue
        /*
        auto loc_iter = mLocationMap.find(&check_cell);
        if (loc_iter != mLocationMap.end()) {
            mDistQueue.erase(loc_iter->second);
            mLocationMap.erase(loc_iter);
        } */

        // and re-queue it with updated dist
        insertCellIntoQueue(check_index);
    }
}

bool PotentialMap::findNeighborLowerCost(CellIndex& index) {
    auto lowestGoalDist = getCell(index).goalDist;
    auto lowestIndex = index;

    for (int x = std::max(0, index.x() - 1); x < std::min(index.x() + 2, (int)mMatrix.cols());
         x++) {
        for (int y = std::max(0, index.y() - 1); y < std::min(index.y() + 2, (int)mMatrix.rows());
             y++) {
            if (x == index.x() && y == index.y()) {
                continue;
            }

            const auto dist = getGoalDistance(x, y);
            if (dist >= 0 && dist < lowestGoalDist) {
                lowestGoalDist = dist;
                lowestIndex = {x, y};
            }
        }
    }

    const bool hasNext = (lowestIndex != index);
    index = lowestIndex;
    return hasNext;
}
