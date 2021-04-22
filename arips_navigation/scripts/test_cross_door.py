#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, PointStamped
from arips_navigation.msg import CrossDoorInformation

navigation_active = False

def callback(data):
    global navigation_active
    navigation_active = data.data

def talker():
    global navigation_active

    rospy.init_node('open_door', anonymous=True)

    door_info_pub = rospy.Publisher("/cross_door_info", CrossDoorInformation, queue_size=1)

    #goal_pub = rospy.Publisher('/topo_planner/nav_goal', PoseStamped, queue_size=1)
    #clicked_point = rospy.Publisher('/clicked_point', PointStamped, queue_size=1)

    # rospy.Subscriber("/arips_navigation_active", Bool, callback)

    rospy.sleep(1)

    doorInfo = CrossDoorInformation()
    doorInfo.approachPose.header.frame_id = "map"
    doorInfo.approachPose.header.stamp = rospy.Time.now()
    doorInfo.approachPose.pose.position.x = 0.30492
    doorInfo.approachPose.pose.position.y = -1.308
    doorInfo.approachPose.pose.orientation.z = -0.698194951660605
    doorInfo.approachPose.pose.orientation.w = 0.7159076822298008

    doorInfo.pivotPose.header.frame_id = "map"
    doorInfo.pivotPose.header.stamp = rospy.Time.now()
    doorInfo.pivotPose.pose.position.x = -0.30547
    doorInfo.pivotPose.pose.position.y = -1.5728
    doorInfo.pivotPose.pose.orientation.w = 1

    door_info_pub.publish(doorInfo)

    rospy.sleep(1)

    #rate = rospy.Rate(1) # 1hz
    #while not rospy.is_shutdown():
    #    rate.sleep()
    #    if not navigation_active:
    #        print("Arrived at door, opening it!")
    #        clicked_point.publish(PointStamped())
    #        rospy.sleep(1)
    #        break

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

