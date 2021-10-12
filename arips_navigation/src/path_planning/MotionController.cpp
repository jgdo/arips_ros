#include <arips_navigation/path_planning/MotionController.h>

#include <memory>

struct MotionController::Pimpl {
    PotentialMap& mPotentialMap;

    explicit Pimpl(PotentialMap& potentialMap) : mPotentialMap{potentialMap} {}

    bool goalReached() const {
        auto& costmap = mPotentialMap.costmap();

        geometry_msgs::PoseStamped robotPose;
        if (!costmap.getRobotPose(robotPose)) {
            return false;
        }

        unsigned int cx, cy;
        if (!costmap.getCostmap()->worldToMap(robotPose.pose.position.x, robotPose.pose.position.y,
                                              cx, cy)) {
            return false;
        }

        const auto goal = mPotentialMap.lastGoal();
        return std::abs<int>(goal.x() - cx) <= 2 && std::abs<int>(goal.y() - cy) < 2;
    }

    std::optional<geometry_msgs::Twist> computeVelocity() {
        if (goalReached()) {
            return geometry_msgs::Twist{};
        }

        auto& costmap = mPotentialMap.costmap();

        geometry_msgs::PoseStamped robotPose;
        if (!costmap.getRobotPose(robotPose)) {
            return {};
        }

        unsigned int cx, cy;
        if (!costmap.getCostmap()->worldToMap(robotPose.pose.position.x, robotPose.pose.position.y,
                                              cx, cy)) {
            return {};
        }

        const auto initialRobotCost = costmap.getCostmap()->getCost(cx, cy);
        const auto goal = mPotentialMap.lastGoal();
        const auto closeToGoal = std::abs<int>(goal.x() - cx) <= 4 && std::abs<int>(goal.y() - cx) < 4;
        double gradient = 0;

        if(closeToGoal) {
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
    }
};

MotionController::MotionController(PotentialMap& potentialMap)
    : mPimpl{std::make_unique<Pimpl>(potentialMap)} {}

MotionController::~MotionController() = default;

bool MotionController::goalReached() { return mPimpl->goalReached(); }

std::optional<geometry_msgs::Twist> MotionController::computeVelocity() {
    return mPimpl->computeVelocity();
}
