#include <arips_navigation/AutoDocker.h>

#include <geometry_msgs/Twist.h>

template<class T>
const T& clamp( const T& v, const T& lo, const T& hi )
{
    assert( !(hi < lo) );
    return (v < lo) ? lo : (hi < v) ? hi : v;
}

AutoDocker::AutoDocker(costmap_2d::Costmap2DROS& costmap, ros::Publisher& cmdVelPub):
    mCostmap(costmap), mCmdVelPub(cmdVelPub) {
}

bool AutoDocker::isActive() {
    return mIsDocking;
} 

static double raycastDist(const costmap_2d::Costmap2D* costmap, double angle) {
    const double dx = cos(angle), dy = sin(angle);

    int x0, y0;
    costmap->worldToMapNoBounds(0, 0, x0, y0);

    // const double r = 0.2;

    for(unsigned i = 0; i < std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()); i++) {
        int x = std::round(x0 + dx*i), y = std::round(y0 + dy*i);

        // std::cout << x << ", " << y << "| ";

        if(x < 0 || x >= costmap->getSizeInCellsX() || y < 0 || y >= costmap->getSizeInCellsY()) {
            break;
        }

        if(costmap->getCost(x, y) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
            const double minDist = std::sqrt(std::pow(x-x0, 2) + std::pow(y-y0, 2));
            return minDist;
        }
    }

    return -1;
}

static double raycastSet(const costmap_2d::Costmap2D* costmap, double angleStart, double angleEnd, int num) {
    const double angleInc = num > 1? (angleEnd - angleStart) / (num-1) : 0;

    double angle = angleStart;
    double sum = 0;
    int numValid = 0;
    for(int i = 0; i < num; i++) {
        const double dist = raycastDist(costmap, angle);
        if(dist >= 0) {
            sum += dist;
            numValid++;
        }
    }

    if(numValid == 0) {
        return -1;
    }

    return sum / numValid;
}

void AutoDocker::runCycle() {
    const costmap_2d::Costmap2D* costmap = mCostmap.getCostmap();

    const double dist = raycastSet(costmap, M_PI-0.2, M_PI+0.2, 5) * costmap->getResolution();

    const double distL = raycastSet(costmap, M_PI-1, M_PI-0.1, 10)*costmap->getResolution(); // about 10..60deg
    const double distR = raycastSet(costmap, -(M_PI-1), -(M_PI-0.1), 10)*costmap->getResolution(); // about 10..60deg

    // ROS_INFO_STREAM("dist " << dist);

    geometry_msgs::Twist twist;
    if(dist > 0.23 || dist < 0) {
        twist.linear.x = -0.05;
    //}
    //
    // if(dist > 0) {
        twist.angular.z += (distR - distL) * 2;
        twist.angular.z = clamp<double>(twist.angular.z, -0.2, 0.2);
        mIsDocking = true;
    } else {
        mIsDocking = false;
    } 

    mCmdVelPub.publish(twist);

    ROS_INFO_STREAM("dist " << dist << " L " << distL << " R " << distR << " X "<< twist.linear.x << " Z " << twist.angular.z);
} 