#include <arips_navigation/NavigationContext.h>

NavigationContext::NavigationContext() {
    ros::NodeHandle nh;
    mCmdVelPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, false);
}

void NavigationContext::publishCmdVel(const geometry_msgs::Twist& cmd_vel) {
    mCmdVelPub.publish(cmd_vel);
}
