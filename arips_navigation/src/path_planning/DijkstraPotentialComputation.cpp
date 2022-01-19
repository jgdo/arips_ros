#include <arips_navigation/path_planning/DijkstraPotentialComputation.h>

#include <chrono>

DijkstraPotentialComputation::DijkstraPotentialComputation(const CostFunction& costFunction)
    : mCostFunction{costFunction} {}

PotentialMap DijkstraPotentialComputation::computeDijkstra(const Costmap& costmap,
                                                           const CellIndex& goal) {
    const auto begin = std::chrono::steady_clock::now();

    // TODO check if gaol index in cell
    // TODO check if goal index not in collision

    PotentialMap potmap{costmap.width(), costmap.height(), costmap.geometry(), goal,
                        costFunction()};

    mDistQueue = DistQueue{};

    const auto endInit = std::chrono::steady_clock::now();
    const auto elapsedInitMs =
        std::chrono::duration_cast<std::chrono::milliseconds>(endInit - begin).count();

    ROS_INFO_STREAM("Global map dijkstra init took " << elapsedInitMs << " ms");

    auto& startCell = potmap.cellAt(goal);
    startCell = {0.0, false};
    insertCellIntoQueue(goal, potmap);

    computeTargetDistance(costmap, potmap);

    const auto end = std::chrono::steady_clock::now();
    const auto elapsedMs =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - endInit).count();

    ROS_INFO_STREAM("Global map dijkstra potential took " << elapsedMs << " ms");
}

void DijkstraPotentialComputation::insertCellIntoQueue(const CellIndex& index,
                                                       PotentialMap& potmap) {
    auto& cell = potmap.cellAt(index);
    mDistQueue.push({&cell, index});
    // mLocationMap.insert({&cell, iter});
}

void DijkstraPotentialComputation::computeTargetDistance(const Costmap& costmap,
                                                         PotentialMap& potmap) {
    const auto lastX = potmap.width() - 1;
    const auto lastY = potmap.height() - 1;

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
            updatePathCell(current_cell, current_cell.index + CellIndex{-1, 0}, costmap, potmap, 1);
        }

        if (current_cell.index.x() < lastX) {
            updatePathCell(current_cell, current_cell.index + CellIndex{+1, 0}, costmap, potmap, 1);
        }

        if (current_cell.index.y() > 0) {
            updatePathCell(current_cell, current_cell.index + CellIndex{0, -1}, costmap, potmap, 1);
        }

        if (current_cell.index.y() < lastY) {
            updatePathCell(current_cell, current_cell.index + CellIndex{0, +1}, costmap, potmap, 1);
        }

        if (current_cell.index.x() > 0 && current_cell.index.y() > 0) {
            updatePathCell(current_cell, current_cell.index + CellIndex{-1, -1}, costmap, potmap,
                           std::sqrt(2.0));
        }

        if (current_cell.index.x() > 0 && current_cell.index.y() < lastY) {
            updatePathCell(current_cell, current_cell.index + CellIndex{-1, +1}, costmap, potmap,
                           std::sqrt(2.0));
        }

        if (current_cell.index.x() < lastX && current_cell.index.y() > 0) {
            updatePathCell(current_cell, current_cell.index + CellIndex{+1, -1}, costmap, potmap,
                           std::sqrt(2.0));
        }

        if (current_cell.index.x() < lastX && current_cell.index.y() < lastY) {
            updatePathCell(current_cell, current_cell.index + CellIndex{+1, +1}, costmap, potmap,
                           std::sqrt(2.0));
        }
    }
}
DijkstraPotentialComputation::CellEntry DijkstraPotentialComputation::takeFirstFromQueue() {
    auto cellEntry = mDistQueue.top();
    mDistQueue.pop();
    return cellEntry;
}

void DijkstraPotentialComputation::updatePathCell(
    const DijkstraPotentialComputation::CellEntry& current_cell, const CellIndex& check_index,
    const Costmap& costmap, PotentialMap& potmap, double cellDist) {

    auto& check_cell = potmap.cellAt(check_index);
    if (check_cell.visited) {
        return;
    }

    // if the cell is an obstacle set the max path distance
    const auto cellCost = costmap.at(check_index);
    if (!cellCost) {
        check_cell.goalDist = obstacleCosts();
        check_cell.visited = true;
        return;
    }

    const auto meterDist = cellDist * costmap.resolution();
    const double newGoalDist =
        current_cell.cell->goalDist + meterDist / mCostFunction.maxWheelSpeedFromCosts(*cellCost);
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
        insertCellIntoQueue(check_index, potmap);
    }
}
