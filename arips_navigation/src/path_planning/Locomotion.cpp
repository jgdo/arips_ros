#include <arips_navigation/LocomotionConfig.h>
#include <arips_navigation/path_planning/Locomotion.h>
#include <arips_navigation/path_planning/MotionController.h>
#include <arips_navigation/path_planning/PotentialMap.h>

struct Locomotion::Pimpl {
    CostFunction mCostFunction;
    PotentialMap mPotentialMap;
    MotionController mMotionController;

    std::optional<Pose2D> mCurrentGoal;

    ros::Time mLastControllerSuccessfulTime{0};

    arips_navigation::LocomotionConfig mConfig;
    dynamic_reconfigure::Server<arips_navigation::LocomotionConfig> mConfigServer{
        ros::NodeHandle{"~/locomotion"}};

    explicit Pimpl(costmap_2d::Costmap2DROS& costmap)
        : mPotentialMap{costmap, mCostFunction}, mMotionController{mPotentialMap} {

        dynamic_reconfigure::Server<arips_navigation::LocomotionConfig>::CallbackType cb =
            [this](auto&& PH1, auto&& PH2) {
                onDynamicReconfigure(std::forward<decltype(PH1)>(PH1),
                                     std::forward<decltype(PH2)>(PH2));
            };
        mConfigServer.setCallback(cb);
    }

    [[nodiscard]] const costmap_2d::Costmap2DROS& costmap() const {
        return mPotentialMap.getCostmapRos();
    }

    std::optional<double> makePlan(const Pose2D& robotPose, const Pose2D& goal) {
        const boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(
            *(costmap().getCostmap()->getMutex()));

        // path.poses.push_back(robotPose);
        unsigned int robotCx, robotCy, goalCx, goalCy;
        if (!costmap().getCostmap()->worldToMap(robotPose.x(), robotPose.y(), robotCx, robotCy) ||
            !costmap().getCostmap()->worldToMap(goal.x(), goal.y(), goalCx, goalCy)) {

            ROS_WARN_STREAM("Could not find robot or goal pose on costmap.");
            return {};
        }

        costmap().getCostmap()->setCost(robotCx, robotCy, costmap_2d::FREE_SPACE);

        const auto begin = ros::WallTime::now();
        mPotentialMap.computeDijkstra({goalCx, goalCy});
        const auto end = ros::WallTime::now();
        ROS_INFO_STREAM("My potential map potential took ms " << (end - begin).toSec() * 1000);

        // costmap.getCostmap()->setCost(robotCx, robotCy, initialRobotCost);

        return mPotentialMap.getGoalDistance(robotCx, robotCy);
    }

    bool setGoal(const Pose2D& robotPose, const Pose2D& goal) {
        if (makePlan(robotPose, goal)) {
            mCurrentGoal = goal;
            return true;
        }

        mCurrentGoal = {};
        return false;
    }

    bool goalReached(const Pose2D& robotPose) {
        if (mCurrentGoal) {
            return mMotionController.goalReached(robotPose, *mCurrentGoal);
        }

        return true;
    }

    std::optional<double> getGoalDistance(const Pose2D& robotPose) const {
        unsigned int robotCx, robotCy;
        if (!costmap().getCostmap()->worldToMap(robotPose.x(), robotPose.y(), robotCx, robotCy)) {
            return {};
        }

        return mPotentialMap.getGoalDistance(robotCx, robotCy);
    }

    std::optional<Pose2D> computeVelNoRecovery(const Odom2D& robotPose) {
        const boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(
            *(costmap().getCostmap()->getMutex()));

        if (mMotionController.goalReached(robotPose.pose, *mCurrentGoal)) {
            mCurrentGoal = {};
            return Pose2D{};
        }

        bool failed = false;
        if (mConfig.replan_every_step || !getGoalDistance(robotPose.pose)) {
            if (!makePlan(robotPose.pose, *mCurrentGoal)) {
                return {};
            }
        }

        return mMotionController.computeVelocity(robotPose, *mCurrentGoal);
    }


    std::optional<Pose2D> computeVelocityCommands(const Odom2D& robotPose) {
        const boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(
            *(costmap().getCostmap()->getMutex()));

        if (!mCurrentGoal) {
            return {};
        }

        auto vel = computeVelNoRecovery(robotPose);
        if (vel) {
            mLastControllerSuccessfulTime = ros::Time::now();
            return vel;
        }

        ROS_WARN("Could not compute velocity command, choosing alternatives");
        if (ros::Time::now() >
            mLastControllerSuccessfulTime + ros::Duration(mConfig.controller_patience)) {
            doRecovery();
        }

        if (ros::Time::now() >
            mLastControllerSuccessfulTime + ros::Duration(mConfig.fault_patience)) {
            return {};
        }

        return Pose2D{};
    }

    void cancel() { mCurrentGoal = {}; }

    void onDynamicReconfigure(arips_navigation::LocomotionConfig& config, uint32_t level) {
        mConfig = config;
    }

    void doRecovery() { mPotentialMap.costmap().resetLayers(); }
};

Locomotion::Locomotion(costmap_2d::Costmap2DROS& costmap)
    : mPimpl{std::make_unique<Pimpl>(costmap)} {}
Locomotion::~Locomotion() = default;

bool Locomotion::setGoal(const Pose2D& robotPose, const Pose2D& goal) {
    return mPimpl->setGoal(robotPose, goal);
}
bool Locomotion::goalReached(const Pose2D& robotPose) { return mPimpl->goalReached(robotPose); }
std::optional<Pose2D> Locomotion::computeVelocityCommands(const Odom2D& robotPose) {
    return mPimpl->computeVelocityCommands(robotPose);
}
void Locomotion::cancel() { mPimpl->cancel(); }
std::optional<Pose2D> Locomotion::currentGoal() const { return mPimpl->mCurrentGoal; }
const PotentialMap& Locomotion::potentialMap() const { return mPimpl->mPotentialMap; }

std::optional<double> Locomotion::makePlan(const Pose2D& robotPose, const Pose2D& goal) {
    return mPimpl->makePlan(robotPose, goal);
}
