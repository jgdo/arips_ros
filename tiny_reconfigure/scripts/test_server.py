#!/usr/bin/env python

#from tiny_reconfigure.srv import *
from tiny_reconfigure.msg import *
import rospy

groupPub = None

def handle_get_def(req):
    print "handle_get_def"
    global groupPub

    groupPub.publish(GroupDef(1, 2, "group 1"))
    groupPub.publish(GroupDef(0, 2, "group 0"))

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Subscriber('get_def', GetDef, handle_get_def)
    global groupPub
    groupPub = rospy.Publisher('group_def', GroupDef, queue_size=10)
    print "Ready to add two ints."
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
