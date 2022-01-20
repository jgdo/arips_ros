#include <arips_navigation/LocomotionConfig.h>
#include <arips_navigation/path_planning/DijkstraPotentialComputation.h>
#include <arips_navigation/path_planning/Locomotion.h>
#include <arips_navigation/path_planning/MotionController.h>

#include <arips_navigation/path_planning/Costmap.h>

struct Locomotion::Pimpl {
    CostFunction mCostFunction;
    DijkstraPotentialComputation mPathPlanning;
    MotionController mMotionController;

    std::optional<PotentialMap> mPotmap;

    ros::Time mLastControllerSuccessfulTime{0};

    arips_navigation::LocomotionConfig mConfig;
    dynamic_reconfigure::Server<arips_navigation::LocomotionConfig> mConfigServer{
        ros::NodeHandle{"~/locomotion"}};

    explicit Pimpl() : mPathPlanning{mCostFunction}, mMotionController{} {

        dynamic_reconfigure::Server<arips_navigation::LocomotionConfig>::CallbackType cb =
            [this](auto&& PH1, auto&& PH2) {
                onDynamicReconfigure(std::forward<decltype(PH1)>(PH1),
                                     std::forward<decltype(PH2)>(PH2));
            };
        mConfigServer.setCallback(cb);
    }

    std::optional<PotentialMap> makePlan(const Costmap& costmap, const Pose2D& robotPose,
                                         const Pose2D& goal) {
        // path.poses.push_back(robotPose);

        const auto robotIndex = costmap.toMap(robotPose.point);
        if (!robotIndex) {
            ROS_WARN_STREAM("Could not find robot pose on costmap.");
            return {};
        }

        // TODO costmap().getCostmap()->setCost(robotCx, robotCy, costmap_2d::FREE_SPACE);

        const auto begin = ros::WallTime::now();
        auto potmap = mPathPlanning.computeDijkstra(costmap, goal);
        const auto end = ros::WallTime::now();
        ROS_INFO_STREAM("My potential map potential took ms " << (end - begin).toSec() * 1000);

        // TODO costmap.getCostmap()->setCost(robotCx, robotCy, initialRobotCost);
        if (!potmap.at(*robotIndex)) {
            return {};
        }

        return potmap;
    }

    bool setGoal(const Costmap& costmap, const Pose2D& robotPose, const Pose2D& goal) {
        mPotmap = makePlan(costmap, robotPose, goal);
        return mPotmap.operator bool();
    }

    bool goalReached(const Pose2D& robotPose) {
        if (mPotmap) {
            return mMotionController.goalReached(robotPose, mPotmap->goal());
        }

        return true;
    }

    std::optional<double> getGoalDistance(const Pose2D& robotPose) const {
        if (!mPotmap) {
            return {};
        }

        return mPotmap->atPos(robotPose.point);
    }

    std::optional<Twist2D> computeVelNoRecovery(const Costmap& costmap, const Odom2D& robotPose) {
        const auto goal = mPotmap->goal();
        if (mMotionController.goalReached(robotPose.pose, goal)) {
            mPotmap.reset();
            return Twist2D{}; // 0 twist, not confuse with nullopt!
        }

        if (mConfig.replan_every_step || !getGoalDistance(robotPose.pose)) {
            if (!(mPotmap = makePlan(costmap, robotPose.pose, goal))) {
                return {};
            }
        }

        return mMotionController.computeVelocity(ComposedNavMap{*mPotmap, costmap}, robotPose,
                                                 goal);
    }

    std::optional<Pose2D> computeVelocityCommands(const Costmap& costmap, const Odom2D& robotPose) {
        if (!mPotmap) {
            return {};
        }

        auto vel = computeVelNoRecovery(costmap, robotPose);
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

    void cancel() { mPotmap.reset(); }

    void onDynamicReconfigure(arips_navigation::LocomotionConfig& config, uint32_t level) {
        mConfig = config;
    }

    void doRecovery() {
        // TODO mPathPlanning.costmap().resetLayers();
    }
};

Locomotion::Locomotion() : mPimpl{std::make_unique<Pimpl>()} {}
Locomotion::~Locomotion() = default;

bool Locomotion::setGoal(const Costmap& costmap, const Pose2D& robotPose, const Pose2D& goal) {
    return mPimpl->setGoal(costmap, robotPose, goal);
}
bool Locomotion::goalReached(const Pose2D& robotPose) { return mPimpl->goalReached(robotPose); }
std::optional<Pose2D> Locomotion::computeVelocityCommands(const Costmap& costmap,
                                                          const Odom2D& robotPose) {
    return mPimpl->computeVelocityCommands(costmap, robotPose);
}
void Locomotion::cancel() { mPimpl->cancel(); }
std::optional<Pose2D> Locomotion::currentGoal() const {
    if (mPimpl->mPotmap) {
        return mPimpl->mPotmap->goal();
    }
    return std::nullopt;
}

const PotentialMap* Locomotion::potentialMap() const {
    return mPimpl->mPotmap? &*mPimpl->mPotmap : nullptr;
}

/*
std::optional<double> Locomotion::makePlan(const Costmap& costmap, const Pose2D& robotPose, const
Pose2D& goal) { return mPimpl->makePlan(costmap, robotPose, goal);
}
*/