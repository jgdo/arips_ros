#include <arips_navigation/DriveUntilCollision.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

struct DriveUntilCollision::Pimpl : public DrivingStateProto {
    using DrivingStateProto::DrivingStateProto;

    void activate(const geometry_msgs::Twist& cmd_vel, double collisionDistance, ros::Duration timeout) {
        mCmdVel = cmd_vel;
        mCollisionDistance = collisionDistance;
        mLastTime = ros::Time::now();
        mTimeout = timeout;
    }

    bool isActive() override { return mCollisionDistance >= 0; }

    void runCycle() override {
        geometry_msgs::Twist cmd_vel;
        if (isActive()) {
            geometry_msgs::PoseStamped localPoseMsg;
            localCostmap().getRobotPose(localPoseMsg);
            tf2::Stamped<tf2::Transform> localPose;
            tf2::fromMsg(localPoseMsg, localPose);

            const auto robotFront = localPose * tf2::Vector3{mCollisionDistance, 0, 0};
            const auto* costmap = localCostmap().getCostmap();
            unsigned int mx, my;

            int costs = costmap->worldToMap(robotFront.x(), robotFront.y(), mx, my)
                            ? costmap->getCost(mx, my)
                            : costmap_2d::LETHAL_OBSTACLE;

            const auto duration = ros::Time::now() - mLastTime;
            if (costs < costmap_2d::INSCRIBED_INFLATED_OBSTACLE && duration < mTimeout) {
                cmd_vel = mCmdVel;
            } else {
                // finished
                mCollisionDistance = -1;
            }
        }

        publishCmdVel(cmd_vel);
    }

    geometry_msgs::Twist mCmdVel;
    double mCollisionDistance = -1;
    ros::Time mLastTime;
    ros::Duration mTimeout;
};

DriveUntilCollision::DriveUntilCollision(NavigationContext& context)
    : pimpl(std::make_unique<Pimpl>(context)) {}

DriveUntilCollision::~DriveUntilCollision() = default;

bool DriveUntilCollision::isActive() { return pimpl->isActive(); }

void DriveUntilCollision::runCycle() { pimpl->runCycle(); }

void DriveUntilCollision::activate(const geometry_msgs::Twist& cmd_vel, double collisionDistance, ros::Duration timeout) {
    pimpl->activate(cmd_vel, collisionDistance, timeout);
}
