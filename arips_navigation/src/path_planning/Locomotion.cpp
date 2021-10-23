#include <arips_navigation/path_planning/Locomotion.h>
#include <arips_navigation/path_planning/MotionController.h>
#include <arips_navigation/path_planning/PotentialMap.h>

struct Locomotion::Pimpl {
    CostFunction mCostFunction;
    PotentialMap mPotentialMap;
    MotionController mMotionController;

    std::optional<Pose2D> mCurrentGoal;

    explicit Pimpl(costmap_2d::Costmap2DROS& costmap)
        : mPotentialMap{costmap, mCostFunction}, mMotionController{mPotentialMap} {}

    [[nodiscard]] const costmap_2d::Costmap2DROS& costmap() const {
        return mPotentialMap.getCostmapRos();
    }

    bool setGoal(const Pose2D& robotPose, const Pose2D& goal) {
        // path.poses.push_back(robotPose);
        unsigned int robotCx, robotCy, goalCx, goalCy;
        if (!costmap().getCostmap()->worldToMap(robotPose.x(), robotPose.y(), robotCx, robotCy) ||
            !costmap().getCostmap()->worldToMap(goal.x(), goal.y(), goalCx, goalCy)) {

            ROS_WARN_STREAM("Could not find robot or goal pose on costmap.");
            return false;
        }

        const auto initialRobotCost = costmap().getCostmap()->getCost(robotCx, robotCy);
        costmap().getCostmap()->setCost(robotCx, robotCy, costmap_2d::FREE_SPACE);

        const auto begin = ros::WallTime::now();
        mPotentialMap.computeDijkstra({goalCx, goalCy});
        const auto end = ros::WallTime::now();
        ROS_INFO_STREAM("My potential map potential took ms " << (end - begin).toSec() * 1000);

        // costmap.getCostmap()->setCost(robotCx, robotCy, initialRobotCost);

        /*
        CellIndex index{cx, goalCy};
        while (potentialMap.findNeighborLowerCost(index)) {
            costmap.getCostmap()->mapToWorld(index.x(), index.y(), robotPose.pose.position.x,
                                             robotPose.pose.position.y);

            path.poses.push_back(robotPose);
        }

        pathPub.publish(path); */

        mCurrentGoal = goal;

        return true;
    }

    bool goalReached(const Pose2D& robotPose) {
        if (mCurrentGoal) {
            return mMotionController.goalReached(robotPose, *mCurrentGoal);
        }

        return true;
    }

    std::optional<Pose2D> computeVelocityCommands(const Pose2D& robotPose) {
        return mMotionController.computeVelocity(robotPose, *mCurrentGoal);
    }

    void cancel() { mCurrentGoal = {}; }

    bool hasGoal() const { return static_cast<bool>(mCurrentGoal); }
};

Locomotion::Locomotion(costmap_2d::Costmap2DROS& costmap)
    : mPimpl{std::make_unique<Pimpl>(costmap)} {}
Locomotion::~Locomotion() = default;

bool Locomotion::setGoal(const Pose2D& robotPose, const Pose2D& goal) {
    return mPimpl->setGoal(robotPose, goal);
}
bool Locomotion::goalReached(const Pose2D& robotPose) { return mPimpl->goalReached(robotPose); }
std::optional<Pose2D> Locomotion::computeVelocityCommands(const Pose2D& robotPose) {
    return mPimpl->computeVelocityCommands(robotPose);
}
void Locomotion::cancel() { mPimpl->cancel(); }
bool Locomotion::hasGoal() const { return mPimpl->hasGoal(); }
const PotentialMap& Locomotion::potentialMap() const { return mPimpl->mPotentialMap; }
