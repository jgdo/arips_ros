#include <arips_navigation/path_planning/MotionController.h>

#include <memory>

// See
// https://stackoverflow.com/questions/16605967/set-precision-of-stdto-string-when-converting-floating-point-values
template <typename T> std::string to_string_with_precision(const T a_value, const int n = 6) {
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
}

using Eigen::Vector2d;

struct Pose2D {
    Vector2d point;
    double theta = 0;

    [[nodiscard]] Pose2D moved(const Pose2D& vel, double dt) const {
        const Vector2d dir = {vel.point.x() * cos(theta) + vel.point.y() * cos(M_PI_2 + theta),
                              vel.point.x() * sin(theta) + vel.point.y() * cos(M_PI_2 + theta)};
        return {point + dir * dt, angles::normalize_angle(theta + vel.theta * dt)};
    }

    // TODO conversion functions from geometry_msgs::PoseStamped and similar
};

struct TrajectoryPoint {
    Pose2D pose;
    Pose2D velocity;
    double timeFromStart = 0;
};

using Trajectory = std::vector<TrajectoryPoint>;
using Trajectories = std::vector<Trajectory>;

Trajectory generateTrajectory(Pose2D currentPose, const Pose2D& vel, double duration, double dt) {
    const int numSteps = std::ceil(duration / dt);
    dt = duration / numSteps;

    Trajectory traj;

    double t = 0;
    for (int i = 0; i < numSteps; i++) {
        traj.push_back(TrajectoryPoint{currentPose, vel, t});
        currentPose = currentPose.moved(vel, dt);
        t += dt;
    }

    traj.emplace_back(TrajectoryPoint{currentPose, vel, t});

    return traj;
}

double computeDuration(double speed, double goalDist, double maxDuration) {
    return std::min(goalDist / speed, maxDuration);
}

double computeSpeed(double goalDist, double duration, double maxSpeed) {
    return std::min(goalDist / duration, maxSpeed);
}

Trajectories sampleTrajectories(const Pose2D& currentPose, const double goalDistance,
                                const CostFunction& constFunction) {
    const auto duration = 0.8;
    const auto transSpeed = computeSpeed(goalDistance, duration, constFunction.maxWheelSpeed());
    const auto rotSpeed = 1.0;
    const auto dt = duration / 20;

    Trajectories traj;
    traj.emplace_back(generateTrajectory(currentPose, {{0, 0}, rotSpeed}, duration, dt));
    traj.emplace_back(generateTrajectory(currentPose, {{0, 0}, -rotSpeed}, duration, dt));

    const auto curvatureStep = 0.5;
    for (double curvature = -20; curvature < 20 + curvatureStep / 2; curvature += curvatureStep) {
        const auto speedFactor = transSpeed / (1.0 + std::abs(curvature) * 0.2);
        const Pose2D vel{{1 * speedFactor, 0}, curvature * speedFactor};

        traj.emplace_back(generateTrajectory(currentPose, vel, duration, dt));
    }

    return traj;
}

std::optional<std::pair<double, uint8_t>>
scoreTrajectory(const PotentialMap& map, const Trajectory& traj, const CostFunction& costFunction) {
    const auto& costmap = map.costmap();
    const auto resolution = map.getCostmapRos().getCostmap()->getResolution();

    if (traj.empty()) {
        return {};
    }

    const auto frontPos = traj.back().pose.point;
    unsigned int cx, cy;
    if (!costmap.getCostmap()->worldToMap(frontPos.x(), frontPos.y(), cx, cy)) {
        return {};
    }

    const auto costToGoal = map.getGoalDistance(cx, cy);
    if (costToGoal < 0.0 || std::isnan(costToGoal) || std::isinf(costToGoal)) {
        return {};
    }

    double trajectoryCost = 0;
    uint8_t worstCellCost = 0;
    for (size_t i = 1; i < traj.size(); i++) {
        const auto& a = traj.at(i - 1);
        const auto& b = traj.at(i);

        if (!costmap.getCostmap()->worldToMap(a.pose.point.x(), a.pose.point.y(), cx, cy)) {
            return {};
        }

        const auto cellCost = costmap.getCostmap()->getCost(cx, cy);
        if (!costFunction.isValidCellCost(cellCost)) {
            return {};
        }

        const auto meterDist = (b.pose.point - a.pose.point).norm();
        const auto dCost = meterDist / costFunction.maxWheelSpeedFromCosts(cellCost);
        const auto rCost =
            0; // std::abs(angles::shortest_angular_distance(a.pose.theta, b.pose.theta));
        trajectoryCost += dCost + rCost;

        worstCellCost = std::max(worstCellCost, cellCost);
    }

    const auto totalCost = costToGoal * 1.1 + trajectoryCost;
    return std::pair<double, uint8_t>{totalCost, worstCellCost};
}

struct MotionController::Pimpl {
    static constexpr auto GoalToleranceXY = 0.1;
    static constexpr auto GoalToleranceYaw = 0.1;

    PotentialMap& mPotentialMap;
    ros::Publisher mVizPub;

    explicit Pimpl(PotentialMap& potentialMap) : mPotentialMap{potentialMap} {
        ros::NodeHandle nh;
        mVizPub = nh.advertise<visualization_msgs::MarkerArray>("sampled_trajectories", 10);
    }

    static double posePositionDistance(const geometry_msgs::PoseStamped& robotPose,
                                       const geometry_msgs::PoseStamped& goalPose) {
        return Vector2d{goalPose.pose.position.x - robotPose.pose.position.x,
                        goalPose.pose.position.y - robotPose.pose.position.y}
            .norm();
    }

    [[nodiscard]] bool goalReached(const geometry_msgs::PoseStamped& goalPose) const {
        geometry_msgs::PoseStamped robotPose;
        if (!mPotentialMap.getCostmapRos().getRobotPose(robotPose)) {
            return false;
        }

        const auto goalDistance = posePositionDistance(robotPose, goalPose);
        const auto angularDistance =
            angles::shortest_angular_distance(getYawFromQuaternion(robotPose.pose.orientation),
                                              getYawFromQuaternion(goalPose.pose.orientation));

        return goalDistance <= GoalToleranceXY && std::abs(angularDistance) < GoalToleranceYaw;
    }

    void visualizeTrajectories(const Trajectories& trajectories,
                               const std::vector<double>& allCosts, const Trajectory* bestTraj) {
        visualization_msgs::MarkerArray markerArray;
        visualization_msgs::Marker marker;

        marker.header.frame_id = mPotentialMap.getCostmapRos().getGlobalFrameID();
        marker.header.stamp = ros::Time::now();
        marker.ns = "trajectory";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.001;

        for (const auto& traj : trajectories) {
            const auto index = &traj - trajectories.data();
            const auto trajCosts = allCosts.at(index);
            std_msgs::ColorRGBA color;
            if (&traj == bestTraj) {
                color.r = 0.0;
                color.g = 1.0;
                color.b = 0.0;
                color.a = 1.0;
            } else if (trajCosts < 0) {
                color.r = 0.8;
                color.g = 0.8;
                color.b = 0.8;
                color.a = 0.5;
            } else {
                color.r = 1.0;
                color.g = 0.0;
                color.b = 0.0;
                color.a = 1.0;
            }

            for (size_t i = 1; i < traj.size(); i++) {
                const auto& start = traj.at(i - 1).pose.point;
                const auto& end = traj.at(i).pose.point;
                geometry_msgs::Point point;
                point.x = start.x();
                point.y = start.y();
                point.z = 0.001; // place slightly above gradients for better visibility
                marker.points.push_back(point);
                marker.colors.push_back(color);

                point.x = end.x();
                point.y = end.y();
                marker.points.push_back(point);
                marker.colors.push_back(color);
            }

            visualization_msgs::Marker textMarker;
            textMarker.text = to_string_with_precision(trajCosts, 3) + "=";

            const auto& frontPos = traj.back().pose.point;
            unsigned int cx, cy;
            if (mPotentialMap.getCostmapRos().getCostmap()->worldToMap(frontPos.x(), frontPos.y(),
                                                                       cx, cy)) {

                const auto goalDist = mPotentialMap.getGoalDistance(cx, cy);

                const auto diff = trajCosts - goalDist;
                textMarker.text +=
                    to_string_with_precision(diff, 5) + "+" + to_string_with_precision(goalDist, 3);
                //  to_string_with_precision(mPotentialMap.getGoalDistance(cx, cy), 3);
            } else {
                textMarker.text += "NaN";
            }

            textMarker.header.frame_id = mPotentialMap.getCostmapRos().getGlobalFrameID();
            textMarker.header.stamp = ros::Time::now();
            textMarker.ns = "trajectory_score";
            textMarker.id = index;
            textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            textMarker.action = visualization_msgs::Marker::ADD;
            textMarker.pose.position.x = frontPos.x();
            textMarker.pose.position.y = frontPos.y();
            textMarker.pose.position.z =
                0.001; // place slightly above gradients for better visibility
            textMarker.scale.z = 0.006;
            color.r = 0.8;
            color.g = 0.8;
            color.b = 0.8;
            color.a = 1.0;
            textMarker.color = color;
            markerArray.markers.push_back(textMarker);
        }

        markerArray.markers.emplace_back(std::move(marker));
        mVizPub.publish(markerArray);
    }

    std::optional<geometry_msgs::Twist>
    computeVelocity(const geometry_msgs::PoseStamped& goalPose) {
        const auto angleThresRotateInPlace = 1.5;

        auto& costmap = mPotentialMap.costmap();

        geometry_msgs::PoseStamped robotPose;
        if (!costmap.getRobotPose(robotPose)) {
            return {};
        }

        if (goalReached(goalPose)) {
            return geometry_msgs::Twist{};
        }

        const double robotYaw = getYawFromQuaternion(robotPose.pose.orientation);

        const auto goalDistXY = posePositionDistance(robotPose, goalPose);
        if (goalDistXY < GoalToleranceXY*0.8) {
            const double goalYaw = getYawFromQuaternion(goalPose.pose.orientation);
            const auto rotationDiff = angles::shortest_angular_distance(robotYaw, goalYaw);

            if (std::abs(rotationDiff) > GoalToleranceYaw) {
                const auto sign = rotationDiff < 0 ? -1.0 : 1.0;
                geometry_msgs::Twist cmdVel;
                cmdVel.linear.x = 0;
                cmdVel.linear.y = 0;
                cmdVel.angular.z = sign * (0.1 + 0.8 * std::min(1.0, std::abs(rotationDiff) * 3.0));
                return cmdVel;
            }
        }

        unsigned int robotCx, robotCy;
        if (costmap.getCostmap()->worldToMap(robotPose.pose.position.x, robotPose.pose.position.y,
                                             robotCx, robotCy)) {
            const auto optGrad = mPotentialMap.getGradient({robotCx, robotCy});
            if (optGrad) {
                const auto rotationDiff = angles::shortest_angular_distance(robotYaw, *optGrad);

                if (std::abs(rotationDiff) > angleThresRotateInPlace) {
                    const auto sign = rotationDiff < 0 ? -1.0 : 1.0;
                    geometry_msgs::Twist cmdVel;
                    cmdVel.linear.x = 0;
                    cmdVel.linear.y = 0;
                    cmdVel.angular.z = 0.5 * sign;
                    return cmdVel;
                }
            }
        }

        const auto trajectories =
            sampleTrajectories({{robotPose.pose.position.x, robotPose.pose.position.y}, robotYaw},
                               goalDistXY, mPotentialMap.costFunction());

        std::vector<double> trajectoryCosts;
        trajectoryCosts.reserve(trajectories.size());
        const Trajectory* bestTraj = nullptr;
        std::pair<double, uint8_t> bestCost{-1, 0};

        for (const auto& traj : trajectories) {
            const auto cost = scoreTrajectory(mPotentialMap, traj, mPotentialMap.costFunction());
            trajectoryCosts.push_back(cost ? cost->first : -1);

            if (!cost) {
                continue;
            }

            if (bestTraj == nullptr || cost->first < bestCost.first) {
                bestTraj = &traj;
                bestCost = *cost;
            }
        }

        visualizeTrajectories(trajectories, trajectoryCosts, bestTraj);

        if (bestTraj == nullptr) {
            ROS_WARN_STREAM("Could not find any legal trajectory.");
            return {};
        }

        const auto velocityScale =
            0.5 + 0.5 * ((150.0 - std::min<double>(bestCost.second, 150)) / 150.0);
        ROS_INFO_STREAM("best cell cost = " << int(bestCost.second)
                                            << ", velocityScale = " << velocityScale);
        const auto bestVel = bestTraj->at(0).velocity;
        geometry_msgs::Twist cmdVel;
        cmdVel.linear.x = bestVel.point.x() * velocityScale;
        cmdVel.linear.y = bestVel.point.y() * velocityScale;
        cmdVel.angular.z = bestVel.theta * velocityScale;
        return cmdVel;

        /*
        unsigned int cx, cy;
        if (!costmap.getCostmap()->worldToMap(robotPose.pose.position.x, robotPose.pose.position.y,
                                              cx, cy)) {
            return {};
        }

        const auto initialRobotCost = costmap.getCostmap()->getCost(cx, cy);
        const auto goal = mPotentialMap.lastGoal();
        const auto closeToGoal =
            std::abs<int>(goal.x() - cx) <= 4 && std::abs<int>(goal.y() - cx) < 4;
        double gradient = 0;

        if (closeToGoal) {
            gradient = atan2(goal.y() - cy, goal.x() - cx);
        } else {
            auto optGrad = mPotentialMap.getGradient({cx, cy});

            // get gradient from path if too close to obstacle
            if (!optGrad) {
                CellIndex index{cx, cy};
                if (mPotentialMap.findNeighborLowerCost(index)) {
                    if (mPotentialMap.findNeighborLowerCost(index)) {
                        if (mPotentialMap.findNeighborLowerCost(index)) {
                            const auto dx = (int)index.x() - (int)cx;
                            const auto dy = (int)index.y() - (int)cy;
                            optGrad = atan2(dy, dx);
                        }
                    }
                }
            }

            if (!optGrad) {
                return {};
            }

            gradient = *optGrad;

            ROS_INFO_STREAM("Gradient: " << gradient);

            const auto stepSize = 0.1;
            const auto nextX = robotPose.pose.position.x + cos(*optGrad) * stepSize;
            const auto nextY = robotPose.pose.position.y + sin(*optGrad) * stepSize;

            unsigned int nextCx, nextCy;
            if (costmap.getCostmap()->worldToMap(nextX, nextY, nextCx, nextCy)) {

                const auto nextGrad = mPotentialMap.getGradient({nextCx, nextCy});
                if (nextGrad) {
                    ROS_INFO_STREAM("nextGrad: " << *nextGrad);
                    const auto gradDiff = angles::shortest_angular_distance(*optGrad, *nextGrad);
                    ROS_INFO_STREAM("gradDiff: " << gradDiff);
                    gradient += gradDiff / 2;
                }
            }
        }

        const double maxCost = 50;
        const double velocityScale =
            (maxCost - std::min<double>(initialRobotCost, maxCost)) / maxCost * 0.1 + 0.05;

        const auto vThFactor = 10.0;

        ROS_INFO_STREAM("initialRobotCost: " << (int)initialRobotCost);

        const double yaw = getYawFromQuaternion(robotPose.pose.orientation);
        const auto relativeAngle = angles::normalize_angle(gradient - yaw);

        geometry_msgs::Twist cmdVel;
        cmdVel.linear.x = cos(relativeAngle) * velocityScale;
        cmdVel.angular.z = sin(relativeAngle) * velocityScale * vThFactor;

        return cmdVel;
         */
    }
};

MotionController::MotionController(PotentialMap& potentialMap)
    : mPimpl{std::make_unique<Pimpl>(potentialMap)} {}

MotionController::~MotionController() = default;

bool MotionController::goalReached(const geometry_msgs::PoseStamped& gaolPose) {
    return mPimpl->goalReached(gaolPose);
}

std::optional<geometry_msgs::Twist>
MotionController::computeVelocity(const geometry_msgs::PoseStamped& goalPose) {
    return mPimpl->computeVelocity(goalPose);
}
