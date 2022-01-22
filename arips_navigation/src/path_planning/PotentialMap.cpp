#include <arips_navigation/path_planning/PotentialMap.h>

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

std::optional<double> PotentialMap::at(CellIndex index) const {
    if (index.x() >= 0 && index.x() < mMatrix.rows() && index.y() >= 0 &&
        index.y() < mMatrix.cols()) {
        const auto& cell = mMatrix(index.x(), index.y());
        if (cell.goalDist >= 0.0) {
            return {cell.goalDist};
        }
    }
    return {};
}

std::optional<CellIndex> PotentialMap::findNeighborLowerCost(const CellIndex& index) const {
    const auto ownGoalDist = at(index);
    if (!ownGoalDist) {
        return {};
    }
    auto lowestGoalDist = *ownGoalDist;
    std::optional<CellIndex> lowestIndex = {};

    for (int x = std::max(0, index.x() - 1); x < std::min(index.x() + 2, (int)mMatrix.rows());
         x++) {
        for (int y = std::max(0, index.y() - 1); y < std::min(index.y() + 2, (int)mMatrix.cols());
             y++) {
            if (x == index.x() && y == index.y()) {
                continue;
            }

            const auto dist = mMatrix(x, y).goalDist; // range checked by loop
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
            auto distDiff = *ownGoalDist - dist;
            if (std::abs((int)x - (int)index.x()) + std::abs((int)y - (int)index.y()) == 2) {
                distDiff /= std::sqrt(2); // "interpolate" cost difference from 1.41 => 1.0 length
            }
            const auto scaledDist = *ownGoalDist - distDiff;

            if (scaledDist < lowestGoalDist) {
                lowestGoalDist = scaledDist;
                lowestIndex = {x, y};
            }
        }
    }

    return lowestIndex;
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

std::vector<Pose2D> PotentialMap::traceCurrentPath(const Pose2D& robotPose) const {
    auto index = toMap(robotPose.point);
    if (!index) {

        ROS_WARN_STREAM("Could not find robot or goal pose on costmap.");
        return {};
    }

    auto pathPose = robotPose;
    std::vector<Pose2D> path = {pathPose};
    pathPose.theta = 0; // TODO for now orientation is always 0

    while ((index = findNeighborLowerCost(*index))) {
        pathPose.point = toWorld(*index);
        path.push_back(pathPose);
    }

    return path;
}

std::optional<double> PotentialMap::interpolateAt(const Vector2d& pos) const {
    const auto gridPos = (pos - mGeometry.offset) / mGeometry.resolution;
    const Vector2d p00{std::floor(gridPos.x()), std::floor(gridPos.y())};

    const auto v00 = at({p00.x(), p00.y()});
    const auto v10 = at({p00.x() + 1, p00.y()});
    const auto v01 = at({p00.x(), p00.y() + 1});
    const auto v11 = at({p00.x() + 1, p00.y() + 1});

    if (!v00 || !v10 || !v01 || !v11) {
        return atPos(pos); // TODO extrapolate from other direction
    }

    const Vector2d pIn = gridPos - p00;
    const Vector2d pOut = Vector2d{1.0, 1.0} - pIn;

    if(pIn.x() < 0 || pIn.y() < 0 || pOut.x() < 0 || pOut.y() < 0) {
        throw std::runtime_error("interpolation error! " + std::to_string(p00.x()) + " " + std::to_string(p00.y())
                                 + " " + std::to_string(gridPos.x()) + " " + std::to_string(gridPos.y())
                                 + " " + std::to_string(pIn.x()) + " " + std::to_string(pIn.y())
                                 + " " + std::to_string(pOut.x()) + " " + std::to_string(pOut.y()));
    }

    const auto val = *v00 * pOut.x() * pOut.y() + *v01 * pOut.x() * pIn.y() +
                     *v10 * pIn.x() * pOut.y() + *v11 * pIn.x() * pIn.y();
    return val;
}
