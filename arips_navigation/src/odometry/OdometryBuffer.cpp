#include <arips_navigation/odometry/OdometryBuffer.h>

#include <fstream>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

OdometryBuffer::OdometryBuffer(bool alwaysPublish) {
    ros::NodeHandle nh;

    mForecastCmdVelPub = nh.advertise<geometry_msgs::Twist>("cmd_vel_forecast", 10);
    mMarkerPub = nh.advertise<visualization_msgs::Marker>("driven_path", 10);
    mCmdVelSub = nh.subscribe("cmd_vel", 10, &OdometryBuffer::onCmdVel, this);
    mOdomSub = nh.subscribe("odom", 10, &OdometryBuffer::onOdom, this);

    if(alwaysPublish) {
        mPublishTimer = nh.createTimer(ros::Rate{100.0}, &OdometryBuffer::onTimer, this);
    }
}

void OdometryBuffer::onOdom(const nav_msgs::Odometry& msg) {
    mBuffer.push_back({msg, mLastCmdVel});
    while (mBuffer.size() > 1000) {
        mBuffer.pop_front();
    }

    const auto& pos = mBuffer.back().odom.pose.pose.position;
    publishPoint(pos.x, pos.y, 0, 1, 0);
}

void OdometryBuffer::onCmdVel(const geometry_msgs::Twist& msg) { mLastCmdVel = msg; }

void OdometryBuffer::saveBuffer(const std::string& filename) {
    std::ofstream fout{filename};
    if (!fout) {
        return;
    }

    fout << "odom_vel_x, odom_twist_z, cmd_vel_x, cmd_twist_z" << std::endl;
    for (const auto& entry : mBuffer) {
        fout << entry.odom.twist.twist.linear.x << ", " << entry.odom.twist.twist.angular.z << ", "
             << entry.cmdVel.linear.x << ", " << entry.cmdVel.angular.z << std::endl;
    }
}

static double getYawFromQuaternion(const geometry_msgs::Quaternion& msg) {
    tf2::Quaternion q(msg.x, msg.y, msg.z, msg.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}


static auto createQuaternionMsgFromYaw(double yaw)
{
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
}

std::optional<geometry_msgs::PoseStamped> OdometryBuffer::forecastRobotPose() {
    if (mBuffer.size() < 2) {
        return {};
    }

    const auto now = ros::Time::now();
    const auto forecastOffset = ros::Duration{0.15};
    const auto time = now - forecastOffset;

    auto iter = mBuffer.begin();
    auto lastiter = iter++;

    Entry* entry = nullptr;

    for (; iter != mBuffer.end(); lastiter = iter++) {
        if (lastiter->odom.header.stamp < time && time <= iter->odom.header.stamp) {
            entry = &*iter;
            break;
        }
    }

    if (!entry) {
        return {};
    }

    const double alpha = (time - lastiter->odom.header.stamp).toSec() /
                         (iter->odom.header.stamp - lastiter->odom.header.stamp).toSec();

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = iter->cmdVel.linear.x * alpha + lastiter->cmdVel.linear.x * (1.0 - alpha);
    cmd_vel.angular.z = iter->cmdVel.angular.z * alpha + lastiter->cmdVel.angular.z * (1.0 - alpha);
    mForecastCmdVelPub.publish(cmd_vel);

    // extrapolate last odom position into future given velocities between [time, now]
    // Take the previously interpolated velocity between time and *iter, then the velocities between
    // [lastiter, iter], and at the end assume constant velocity as given by last received cmd_vel
    const auto& lastOdom = mBuffer.back().odom;
    double x = lastOdom.pose.pose.position.x;
    double y = lastOdom.pose.pose.position.y;
    double th = getYawFromQuaternion(lastOdom.pose.pose.orientation);

    {
        const double dt = (iter->odom.header.stamp - time).toSec();
        const double vx = cmd_vel.linear.x;
        const double vy = cmd_vel.linear.y; // usually 0
        const double vth = cmd_vel.angular.z;

        // see also http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
        const double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        const double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        const double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;
    }
    lastiter = iter++;
    for(;iter != mBuffer.end(); lastiter = iter++) {
        const double dt = (iter->odom.header.stamp - lastiter->odom.header.stamp).toSec();
        const double vx = lastiter->cmdVel.linear.x;
        const double vy = lastiter->cmdVel.linear.y; // usually 0
        const double vth = lastiter->cmdVel.angular.z;

        // see also http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
        const double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        const double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        const double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;
    }

    {
        const double dt = (now - lastiter->odom.header.stamp).toSec();
        const double vx = lastiter->cmdVel.linear.x;
        const double vy = lastiter->cmdVel.linear.y; // usually 0
        const double vth = lastiter->cmdVel.angular.z;

        // see also http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
        const double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        const double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        const double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;
    }

    geometry_msgs::PoseStamped result;
    result.header.stamp = now;
    result.header.frame_id = lastOdom.header.frame_id;
    result.pose.position.x = x;
    result.pose.position.y = y;
    result.pose.orientation = createQuaternionMsgFromYaw(th);

    return result;
}

void OdometryBuffer::onTimer(const ros::TimerEvent& ev) {
    const auto pose = forecastRobotPose();
    if(pose) {
        publishPoint(pose->pose.position.x, pose->pose.position.y, 1, 0, 0);
    }

    /*
    nav_msgs::Odometry odom;
    odom.header = pose->header;
    odom.header.frame_id = lastOdom.header.frame_id;
    odom.pose = *pose;

    odom.child_frame_id = lastOdom.child_frame_id;
    odom.twist.twist.linear.x = lastiter->cmdVel.linear.x;
    odom.twist.twist.linear.y = lastiter->cmdVel.linear.y;
    odom.twist.twist.angular.z = lastiter->cmdVel.angular.z;


    //publish the message
    odom_pub.publish(odom);
     */
}

void OdometryBuffer::publishPoint(double x, double y, float r, float g, float b) {
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = mBuffer.back().odom.header.frame_id;
    marker.ns = "driven_path";
    marker.id = mMarkerIdCount++;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.lifetime = ros::Duration(10);
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    marker.points.push_back(p);
    std_msgs::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = 1;
    marker.colors.push_back(c);
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    mMarkerPub.publish(marker);
}
