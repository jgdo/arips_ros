#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, PointStamped

navigation_active = False

def callback(data):
    global navigation_active
    navigation_active = data.data

def talker():
    global navigation_active

    rospy.init_node('open_door', anonymous=True)

    goal_pub = rospy.Publisher('/topo_planner/nav_goal', PoseStamped, queue_size=1)
    clicked_point = rospy.Publisher('/clicked_point', PointStamped, queue_size=1)

    rospy.Subscriber("/arips_navigation_active", Bool, callback)

    rospy.sleep(1)

    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = 0.3874
    pose.pose.position.y = -1.119
    pose.pose.orientation.z = 0.8254347619565904
    pose.pose.orientation.w = 0.5644975232484789

    goal_pub.publish(pose)

    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        rate.sleep()
        if not navigation_active:
            print("Arrived at door, opening it!")
            clicked_point.publish(PointStamped())
            rospy.sleep(1)
            break

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

