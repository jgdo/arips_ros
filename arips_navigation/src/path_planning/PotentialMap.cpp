#include <arips_navigation/path_planning/PotentialMap.h>

#include <chrono>

PotentialMap::PotentialMap(costmap_2d::Costmap2DROS& costmap, const CostFunction& costFunction)
    : mCostmapRos(costmap), mCostFunction{costFunction} {}

void PotentialMap::computeDijkstra(const CellIndex& goal) {
    mLastGoal = goal;

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
    const auto lastX = mMatrix.rows() - 1;
    const auto lastY = mMatrix.cols() - 1;

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

        if (current_cell.index.x() < lastX) {
            updatePathCell(current_cell, current_cell.index + CellIndex{+1, 0}, costmap, 1);
        }

        if (current_cell.index.y() > 0) {
            updatePathCell(current_cell, current_cell.index + CellIndex{0, -1}, costmap, 1);
        }

        if (current_cell.index.y() < lastY) {
            updatePathCell(current_cell, current_cell.index + CellIndex{0, +1}, costmap, 1);
        }

        if (current_cell.index.x() > 0 && current_cell.index.y() > 0) {
            updatePathCell(current_cell, current_cell.index + CellIndex{-1, -1}, costmap,
                           std::sqrt(2.0));
        }

        if (current_cell.index.x() > 0 && current_cell.index.y() < lastY) {
            updatePathCell(current_cell, current_cell.index + CellIndex{-1, +1}, costmap,
                           std::sqrt(2.0));
        }

        if (current_cell.index.x() < lastX && current_cell.index.y() > 0) {
            updatePathCell(current_cell, current_cell.index + CellIndex{+1, -1}, costmap,
                           std::sqrt(2.0));
        }

        if (current_cell.index.x() < lastX && current_cell.index.y() < lastY) {
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
                                  const costmap_2d::Costmap2D& costmap, double cellDist) {

    auto& check_cell = getCell(check_index);
    if (check_cell.visited) {
        return;
    }

    // if the cell is an obstacle set the max path distance
    const auto cellCost = costmap.getCost(check_index.x(), check_index.y());
    if (!mCostFunction.isValidCellCost(cellCost)) {
        check_cell.goalDist = obstacleCosts();
        check_cell.visited = true;
        return;
    }

    const auto meterDist = cellDist * costmap.getResolution();
    const double newGoalDist =
        current_cell.cell->goalDist + meterDist / mCostFunction.maxWheelSpeedFromCosts(cellCost);
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

bool PotentialMap::findNeighborLowerCost(CellIndex& index) const {
    const auto ownGoalDist = getCell(index).goalDist;
    auto lowestGoalDist = ownGoalDist;
    auto lowestIndex = index;

    for (int x = std::max(0, index.x() - 1); x < std::min(index.x() + 2, (int)mMatrix.rows());
         x++) {
        for (int y = std::max(0, index.y() - 1); y < std::min(index.y() + 2, (int)mMatrix.cols());
             y++) {
            if (x == index.x() && y == index.y()) {
                continue;
            }

            const auto dist = goalDist(x, y);
            if (dist < 0) {
                continue;
            }

            // Ideal way would be to sample around current pose in a circle, but since the cells
            // are quadratic, the distance difference in diagonal direction is higher than in
            // perpendicular. Therefore for diagonal cells, instead of taking the actual goal cost
            // of the neighboring cell we linearly interpolate the cost in the neighbor direction,
            // but with length of 1 instead of the actual 1.41.
            // Without this trick, the robot will prefer perpendicular movement along, sometimes
            // causing the robot to drive too close to a wall in narrow spaces instead of sticking
            // to the corridor middle.
            auto distDiff = ownGoalDist - dist;
            if (std::abs((int)x - (int)index.x()) + std::abs((int)y - (int)index.y()) == 2) {
                distDiff /= std::sqrt(2); // "interpolate" cost difference from 1.41 => 1.0 length
            }
            const auto scaledDist = ownGoalDist - distDiff;

            if (scaledDist < lowestGoalDist) {
                lowestGoalDist = scaledDist;
                lowestIndex = {x, y};
            }
        }
    }

    const bool hasNext = (lowestIndex != index);
    index = lowestIndex;
    return hasNext;
}

std::optional<double> PotentialMap::getGradient(const CellIndex& index) const {
    {
        Eigen::Matrix<float, 5, 5> conv5;
        conv5 << 5, 4, 0, -4, -5, 8, 10, 0, -10, -8, 10, 20, 0, -20, -10, 8, 10, 0, -10, -8, 5, 4,
            0, -4, -5;

        const auto grad = calcGradientWithMatrix(index, conv5);
        if (grad) {
            return grad;
        }
    }

    {
        Eigen::Matrix3f conv3;
        conv3 << 47, 0, -47, 162, 0, -162, 47, 0, -47;

        const auto grad = calcGradientWithMatrix(index, conv3);
        if (grad) {
            return grad;
        }
    }

    return {};
}

template <class M>
std::optional<double> PotentialMap::calcGradientWithMatrix(const CellIndex& index,
                                                           const M& conv) const {
    assert(conv.cols() == conv.rows());
    const int halfConv = (conv.cols() / 2);
    if (index.x() < halfConv || index.x() >= mMatrix.cols() - halfConv || index.y() < halfConv ||
        index.y() >= mMatrix.rows() - halfConv) {
        return {};
    }

    const int xoff = index.x() - (conv.rows() / 2);
    const int yoff = index.y() - (conv.cols() / 2);

    double dx, dy = 0;
    for (int x = index.x() - halfConv; x <= index.x() + halfConv; x++) {
        for (int y = index.y() - halfConv; y <= index.y() + halfConv; y++) {
            const auto val = mMatrix(x, y).goalDist;
            if (!mMatrix(x, y).visited || val < 0 || std::isnan(val) || std::isinf(val)) {
                return {};
            }

            const auto cx = x - xoff;
            const auto cy = y - yoff;

            dx += val * conv(cy, cx);
            dy += val * conv(cx, cy);
        }
    }

    const auto grad = atan2(dy, dx);
    return grad;
}

std::vector<Pose2D> PotentialMap::traceCurrentPath(const Pose2D& robotPose) const {
    unsigned int robotCx, robotCy;
    if (!costmap().getCostmap()->worldToMap(robotPose.x(), robotPose.y(), robotCx, robotCy)) {

        ROS_WARN_STREAM("Could not find robot or goal pose on costmap.");
        return {};
    }

    auto pathPose = robotPose;
    std::vector<Pose2D> path = {pathPose};
    pathPose.theta = 0; // TODO for now orientation is always 0

    CellIndex index{robotCx, robotCy};
    while (findNeighborLowerCost(index)) {
        costmap().getCostmap()->mapToWorld(index.x(), index.y(), pathPose.point.x(),
                                           pathPose.point.y());

        path.push_back(pathPose);
    }

    return path;
}
