#include <memory>

#include <dynamic_reconfigure/server.h>

#include <arips_navigation/path_planning/MotionController.h>
#include <arips_navigation/path_planning/PlanningMath.h>

#include <visualization_msgs/MarkerArray.h>

// See
// https://stackoverflow.com/questions/16605967/set-precision-of-stdto-string-when-converting-floating-point-values
template <typename T> std::string to_string_with_precision(const T a_value, const int n = 6) {
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
}

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
                                const CostFunction& costFunction,
                                const arips_navigation::MotionControllerConfig& config) {
    const auto duration = config.traj_sampling_duration_s;
    const auto transSpeed = computeSpeed(goalDistance, duration, costFunction.maxWheelSpeed());
    const auto rotSpeed = 1.0;
    const auto dt = duration / config.traj_sampling_steps;

    Trajectories traj;
    traj.emplace_back(generateTrajectory(currentPose, {{0, 0}, rotSpeed}, duration, dt));
    traj.emplace_back(generateTrajectory(currentPose, {{0, 0}, -rotSpeed}, duration, dt));

    const auto curvatureStep = config.traj_curvature_stepsize;
    for (double curvature = -config.traj_curvature_range;
         curvature < config.traj_curvature_range + curvatureStep / 2; curvature += curvatureStep) {
        const auto speedFactor =
            transSpeed / (1.0 + std::abs(curvature) * config.robot_wheel_base_m / 2.0);
        const Pose2D vel{{speedFactor, 0}, curvature * speedFactor};

        traj.emplace_back(generateTrajectory(currentPose, vel, duration, dt));
    }

    return traj;
}

std::optional<std::pair<double, uint8_t>> scoreTrajectory(const NavMap& navmap,
                                                          const Trajectory& traj) {
    const auto& costFunction = navmap.costFunction();

    if (traj.empty()) {
        return {};
    }

    const auto costToGoal = navmap.interpolateGoalDistance(traj.back().pose.point);
    if (!costToGoal) {
        return {};
    }

    double trajectoryCost = 0;
    uint8_t worstCellCost = 0;
    for (size_t i = 1; i < traj.size(); i++) {
        const auto& a = traj.at(i - 1);
        const auto& b = traj.at(i);

        const auto cellCost = navmap.cost(a.pose.point);
        if (!cellCost) {
            return {};
        }

        const auto meterDist = (b.pose.point - a.pose.point).norm();
        const auto dCost = meterDist / costFunction.maxWheelSpeedFromCosts(*cellCost);
        const auto rCost =
            0; // std::abs(angles::shortest_angular_distance(a.pose.theta, b.pose.theta));
        trajectoryCost += dCost + rCost;

        worstCellCost = std::max(worstCellCost, *cellCost);
    }

    const auto totalCost = *costToGoal * 1.1 + trajectoryCost;
    return std::pair<double, uint8_t>{totalCost, worstCellCost};
}

struct MotionController::Pimpl {
    arips_navigation::MotionControllerConfig mConfig;

    ros::Publisher mVizPub;
    dynamic_reconfigure::Server<arips_navigation::MotionControllerConfig> mConfigServer{
        ros::NodeHandle{"~/MotionController"}};

    explicit Pimpl() {
        ros::NodeHandle nh;
        mVizPub = nh.advertise<visualization_msgs::MarkerArray>("sampled_trajectories", 10);

        dynamic_reconfigure::Server<arips_navigation::MotionControllerConfig>::CallbackType cb =
            [this](auto&& PH1, auto&& PH2) {
                onDynamicReconfigure(std::forward<decltype(PH1)>(PH1),
                                     std::forward<decltype(PH2)>(PH2));
            };
        mConfigServer.setCallback(cb);
    }

    [[nodiscard]] bool goalReached(const Pose2D& robotPose, const Pose2D& goalPose) const {
        const auto goalDistance = robotPose.distance(goalPose);
        const auto angularDistance =
            angles::shortest_angular_distance(robotPose.theta, goalPose.theta);

        return goalDistance <= mConfig.goal_tolerance_xy_m &&
               angles::to_degrees(std::abs(angularDistance)) < mConfig.goal_tolerance_yaw_deg;
    }

    void visualizeTrajectories(const NavMap& navmap, const Trajectories& trajectories,
                               const std::vector<double>& allCosts, const Trajectory* bestTraj) {
        visualization_msgs::MarkerArray markerArray;
        visualization_msgs::Marker marker;

        marker.header.frame_id = navmap.frameId();
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
            if (const auto goalDist = navmap.interpolateGoalDistance(frontPos)) {
                const auto diff = trajCosts - *goalDist;
                textMarker.text += to_string_with_precision(diff, 5) + "+" +
                                   to_string_with_precision(*goalDist, 3);
                //  to_string_with_precision(mPotentialMap.getGoalDistance(cx, cy), 3);
            } else {
                textMarker.text += "NaN";
            }

            textMarker.header.frame_id = navmap.frameId();
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

    std::pair<double, double> calcWheelSpeed(const Twist2D& vel) const {
        const auto velRot = vel.theta * mConfig.robot_wheel_base_m / 2.0;

        return {vel.x() - velRot, vel.x() + velRot};
    }

    Twist2D scaleAcceleration(const Twist2D& desiredVel, const Odom2D& current,
                              const NavMap& navmap, double dt) const {
        const auto wheelDes = calcWheelSpeed(desiredVel);
        const auto wheelCurrent = calcWheelSpeed(current.vel);

        const auto maxDiff = std::max(std::abs(wheelDes.first - wheelCurrent.first),
                                      std::abs(wheelDes.second - wheelCurrent.second));

        ROS_INFO_STREAM("maxDiff for scaling is " << maxDiff);

        const auto scalingFactor = mConfig.acc_limit_m_s2 * dt / maxDiff;
        if (scalingFactor >= 1.0) {
            return desiredVel; // acceleration already within limits, original trajectory assumed to
                               // be safe
        }

        ROS_INFO_STREAM("Scaling trajectory by " << scalingFactor);

        // interpolate between current and desired vel by scalingFactor
        const Twist2D newVel{
            current.vel.point * (1.0 - scalingFactor) + desiredVel.point * scalingFactor,
            current.vel.theta * (1.0 - scalingFactor) + desiredVel.theta * scalingFactor};

        // we have now a new velocity, check if it's safe to drive
        const auto traj =
            generateTrajectory(current.pose, newVel, mConfig.collision_lookahead_s,
                               mConfig.collision_lookahead_s / mConfig.traj_sampling_steps);

        const auto optScore = scoreTrajectory(navmap, traj);
        if (optScore) {
            ROS_INFO("Scaling trajectory is safe");
            return newVel;
        }

        ROS_INFO_STREAM(
            "Scaling trajectory is not safe, using old velocity, scalingFactor: " << scalingFactor);

        // new trajectory would collide, ignore acceleration limits
        return desiredVel;
    }

    std::optional<Twist2D> computeVelocity(const NavMap& navmap, const Odom2D& odom,
                                           const Pose2D& goalPose, double dt) {
        const auto& robotPose = odom.pose;

        if (goalReached(robotPose, goalPose)) {
            visualizeTrajectories(navmap, {}, {}, {});
            return Pose2D{};
        }

        Twist2D result;

        const auto goalDistXY = robotPose.distance(goalPose);
        if (goalDistXY < mConfig.goal_tolerance_xy_m * 0.8) {
            const auto rotationDiff =
                angles::shortest_angular_distance(robotPose.theta, goalPose.theta);

            const auto rotDiffDeg = angles::to_degrees(rotationDiff);

            if (std::abs(rotDiffDeg) > mConfig.goal_tolerance_yaw_deg) {

                const auto sign = rotationDiff < 0 ? -1.0 : 1.0;
                result.theta = sign * (0.1 + 0.8 * std::min(1.0, std::abs(rotationDiff) * 3.0));
                visualizeTrajectories(navmap, {}, {}, {});
                return scaleAcceleration(result, odom, navmap, dt);
            }
        }

        if (const auto optGrad = navmap.gradient(robotPose.point)) {
            const auto rotationDiff = angles::shortest_angular_distance(robotPose.theta, *optGrad);

            /*
            ROS_INFO_STREAM("Rotational diff at("
                            << robotCx << ", " << robotCy << ") to path is "
                            << angles::to_degrees(rotationDiff) << ", robot grad: "
                            << robotPose.theta << ", map grad: " << *optGrad << " -- " <<
            *mPotentialMap.getGradient({robotCx, robotCy}) << " " << *optGrad2 << " # "<<
            *optGrad << " * " << *optGrad3);

            ROS_WARN_STREAM("From motion controller " << __LINE__ << " : gradient at 266,293
            from viz is " << *mPotentialMap.getGradient({266, 293}));
             */

            if (std::abs(rotationDiff) >
                angles::from_degrees(mConfig.angle_diff_rotate_in_place_deg)) {
                const auto sign = rotationDiff < 0 ? -1.0 : 1.0;
                result.theta = 1.0 * sign;
                visualizeTrajectories(navmap, {}, {}, {});
                return scaleAcceleration(result, odom, navmap, dt);
            }
        }

        const auto trajectories =
            sampleTrajectories(robotPose, goalDistXY, navmap.costFunction(), mConfig);

        std::vector<double> trajectoryCosts;
        trajectoryCosts.reserve(trajectories.size());
        const Trajectory* bestTraj = nullptr;
        std::pair<double, uint8_t> bestCost{-1, 0};

        for (const auto& traj : trajectories) {
            const auto cost = scoreTrajectory(navmap, traj);
            trajectoryCosts.push_back(cost ? cost->first : -1);

            if (!cost) {
                continue;
            }

            if (bestTraj == nullptr || cost->first < bestCost.first) {
                bestTraj = &traj;
                bestCost = *cost;
            }
        }

        visualizeTrajectories(navmap, trajectories, trajectoryCosts, bestTraj);

        if (bestTraj == nullptr) {
            ROS_WARN_STREAM("Could not find any legal trajectory.");
            return {};
        }

        const auto velocityScale =
            0.5 + 0.5 * ((150.0 - std::min<double>(bestCost.second, 150)) / 150.0);
        ROS_INFO_STREAM("best cell cost = " << int(bestCost.second)
                                            << ", velocityScale = " << velocityScale);
        const auto bestVel = bestTraj->at(0).velocity;
        return scaleAcceleration(bestVel * velocityScale, odom, navmap, dt);

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

    void onDynamicReconfigure(arips_navigation::MotionControllerConfig& config, uint32_t level) {
        mConfig = config;
    }
};

MotionController::MotionController() : mPimpl{std::make_unique<Pimpl>()} {}

MotionController::~MotionController() = default;

bool MotionController::goalReached(const Pose2D& robotPose, const Pose2D& gaolPose) {
    return mPimpl->goalReached(robotPose, gaolPose);
}

std::optional<Twist2D> MotionController::computeVelocity(const NavMap& costmap,
                                                         const Odom2D& robotPose,
                                                         const Pose2D& goalPose, double dt) {
    return mPimpl->computeVelocity(costmap, robotPose, goalPose, dt);
}
