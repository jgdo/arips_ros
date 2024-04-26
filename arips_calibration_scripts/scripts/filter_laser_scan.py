#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import LaserScan
from scipy.spatial.transform import Rotation as R
import numpy as np
import math

class LaserFilter:
    def __init__(self) -> None:
        self.listener = tf.TransformListener()
        self.pub = rospy.Publisher('scan_filtered', LaserScan, queue_size=1)
        self.sub = rospy.Subscriber("scan", LaserScan, self.laser_callback)

        self.door_pivot_map = np.array([0.69, -1.75])
    
    def laser_callback(self, msg: LaserScan):
        try:
            (trans,rot) = self.listener.lookupTransform('map', msg.header.frame_id, rospy.Time(0))

            msg.ranges = list(msg.ranges)
            # rospy.loginfo(f"Reveived a scan with {len(msg.ranges)} points.")

            mat = R.from_quat(rot).as_matrix()
            for i, range in enumerate(msg.ranges):
                if range < msg.range_min or range > msg.range_max:
                    continue

                angle = msg.angle_min + i*msg.angle_increment
                pos_laser = [range * math.cos(angle), range*math.sin(angle), 0]
                pos_map = mat @ pos_laser + trans

                if np.linalg.norm(pos_map[:2] - self.door_pivot_map) < 0.8:
                    msg.ranges[i] = 0

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
            rospy.logwarn(f"Could not transform {msg.header.frame_id} to map: {ex}")
        
        self.pub.publish(msg)

        

if __name__ == '__main__':
    rospy.init_node('laser_filter')
    filter = LaserFilter()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass