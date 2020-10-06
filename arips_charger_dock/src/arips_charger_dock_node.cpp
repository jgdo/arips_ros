//
// Created by jgdo on 9/27/20.
//

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <algorithm>

template<class T>
const T& clamp( const T& v, const T& lo, const T& hi )
{
    assert( !(hi < lo) );
    return (v < lo) ? lo : (hi < v) ? hi : v;
}

class AripsChargerDock {
public:
    AripsChargerDock(tf2_ros::Buffer& tf) :
    mCostmap("docking_costmap", tf) {

        ros::NodeHandle nh;
        mCostmap.start();

        mCmdVelPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 2);

        mControlTimer = nh.createTimer(ros::Duration(0.1), &AripsChargerDock::timerCallback, this);
    }

    ~AripsChargerDock() {
        geometry_msgs::Twist twist;
        mCmdVelPub.publish(twist);
    }

private:
    ros::Publisher mCmdVelPub;

    costmap_2d::Costmap2DROS mCostmap;

    ros::Timer mControlTimer;



    void timerCallback(const ros::TimerEvent&) {
    // mCostmap.updateMap();
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
        }

        mCmdVelPub.publish(twist);

        ROS_INFO_STREAM("dist " << dist << " L " << distL << " R " << distR << " X "<< twist.linear.x << " Z " << twist.angular.z);
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
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "arips_charger_dock");

    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);

    AripsChargerDock dock(buffer);

    while(ros::ok()) {
        ros::spin();
    }


}