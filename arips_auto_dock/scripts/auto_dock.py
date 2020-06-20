#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math


# based on http://wiki.ros.org/turtlesim/Tutorials/Go%20to%20Goal
class AutoDocker:
    def __init__(self):
        rospy.init_node('arips_autodocker')
        
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        
        # Publisher which will publish to the topic '/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/cmd_vel',
                                                  Twist, queue_size=10)
        
        self.rate = rospy.Rate(10)
        self.Timer = None
        
    def update_vel(self, event):
        vel_msg = Twist()
        
        try:
            trans = self.tfBuffer.lookup_transform('arips_base', 'docking_station', rospy.Time())
            
            dist = math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
            if dist > 0.1:
                vel_msg.angular.z = math.copysign(0.5, math.atan2(trans.transform.translation.y, trans.transform.translation.x))
                vel_msg.linear.x = 0.5 
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Could not find frame docking_station")
            pass
        
        self.velocity_publisher.publish(vel_msg)
    
    def start(self):
        self.timer = rospy.Timer(rospy.Duration(0.1), self.update_vel)
    
        
if __name__ == '__main__':
    try:
        d = AutoDocker()
        d.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
