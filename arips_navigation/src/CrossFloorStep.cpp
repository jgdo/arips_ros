#include <arips_navigation/CrossFloorStep.h>

#include <arips_navigation/utils/FloorStepTracker.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <arips_navigation/utils/transforms.h>

struct FloorStepRect {
    Point2d origin;
    Point2d dirX, dirY;
    double angleY;

    //static constexpr double stepWidth = 0.45f;
    //static constexpr double maxValidAngle = 20.0 * M_PI / 180.0;

    explicit FloorStepRect(const FloorStep& step, double robotTheta) {
        dirX = (step[1] - step[0]);
        dirX /= dirX.norm() * dirX.norm();
        dirY = Point2d{-dirX.y(), dirX.x()}.normalized();

        const Point2d robotDir {cos(robotTheta), sin(robotTheta)};
        if(robotDir.dot(dirY) < 0) {
            dirY *= -1;
        }

        origin = step[0];
        angleY = atan2(dirY.y(), dirY.x());
    }

    /*
    bool contains(const Point2d& p) const {
        const Point2d pStep{(p - origin).dot(dirX), (p - origin).dot(dirY)};
        return pStep.x() >= 0 && pStep.x() <= 1 && pStep.y() >= -stepWidth / 2.0f &&
               pStep.y() <= stepWidth / 2.0f;
    }

    std::optional<double> costFactor(double theta) const {
        auto angle = std::abs(angles::shortest_angular_distance(theta, angleY));
        if (angle > M_PI_2) {
            angle = M_PI - angle;
        }

        if (angle > maxValidAngle) {
            return {};
        }

        return 0.2 + angle * 2.0;
    }

     */

    struct DistAngle {
        double dist;
        double angle;
    };

    // dist > 0 means robot has crossed the step, < 0 means not crossed yet
    // angle is from robot to step cross dir
    DistAngle crossingProgressDistAndAngle(const Pose2D& pose) const {
        const auto angle = angles::shortest_angular_distance(pose.theta, angleY);
        const Point2d locationOnStep{(pose.point - origin).dot(dirX), (pose.point - origin).dot(dirY)};
        return {locationOnStep.y(), angle };
    }
};

struct CrossFloorStep::Pimpl : public DrivingStateProto {
    using DrivingStateProto::DrivingStateProto;

    static constexpr double crossSpeed = 0.1;
    static constexpr double distancePastEdgeToComplete = 0.3;
    static constexpr double rotationPFactor = 0.5;

    void activate(const FloorStep& step) {
        mStepTracker.track(step);
    }

    bool isActive() override { return mStepTracker.trackedStepPosition().operator bool(); }

    void runCycle() override {
        geometry_msgs::Twist cmd_vel;
        if (const auto optStep = mStepTracker.trackedStepPosition()) {
            geometry_msgs::PoseStamped localPoseMsg;
            localCostmap().getRobotPose(localPoseMsg);
            const auto robotPose = Pose2D::fromMsg(localPoseMsg.pose);

            const FloorStepRect stepRect {*optStep, robotPose.theta};
            const auto distAngle = stepRect.crossingProgressDistAndAngle(robotPose);
            if(distAngle.dist < distancePastEdgeToComplete) {
                cmd_vel.linear.x = crossSpeed;
                cmd_vel.angular.z = distAngle.angle * rotationPFactor;
            } else {
                // we crossed the step!
                mStepTracker.track(std::nullopt);
            }
        }

        publishCmdVel(cmd_vel);
    }

    SingleFloorStepTracker mStepTracker { tf(), localCostmap().getGlobalFrameID() };
};

CrossFloorStep::CrossFloorStep(NavigationContext& context)
    : pimpl(std::make_unique<Pimpl>(context)) {}

CrossFloorStep::~CrossFloorStep() = default;

bool CrossFloorStep::isActive() { return pimpl->isActive(); }

void CrossFloorStep::runCycle() { pimpl->runCycle(); }

void CrossFloorStep::activate(const FloorStep& step) {
    pimpl->activate(step);
}
