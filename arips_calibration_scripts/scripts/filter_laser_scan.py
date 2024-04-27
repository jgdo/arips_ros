#!/usr/bin/env python3

from typing import Optional
import rospy
import tf
from sensor_msgs.msg import LaserScan
from scipy.spatial.transform import Rotation as R
from arips_semantic_map_msgs.msg import SemanticMap
import numpy as np
import math


class LaserFilter:
    def __init__(self) -> None:
        self.listener = tf.TransformListener()
        self.pub = rospy.Publisher("scan_filtered", LaserScan, queue_size=5)
        self.laser_sub = rospy.Subscriber("scan", LaserScan, self.laser_callback, queue_size=5)
        self.sem_map_sub = rospy.Subscriber("semantic_map", SemanticMap, self.semantic_map_callback, queue_size=1)

        self.semantic_map: Optional[SemanticMap] = None

    def laser_callback(self, msg: LaserScan):
        if self.semantic_map:
            rospy.loginfo_once("Received scan but no semantic scan yet, will forward scan as-is.")
            try:
                (trans, rot) = self.listener.lookupTransform(self.semantic_map.header.frame_id, msg.header.frame_id, rospy.Time(0))

                msg.ranges = list(msg.ranges)
                # rospy.loginfo(f"Reveived a scan with {len(msg.ranges)} points.")

                mat = R.from_quat(rot).as_matrix()
                for i, range in enumerate(msg.ranges):
                    if range < msg.range_min or range > msg.range_max:
                        continue

                    angle = msg.angle_min + i * msg.angle_increment
                    pos_laser = [range * math.cos(angle), range * math.sin(angle), 0]
                    pos_map = mat @ pos_laser + trans

                    # TODO: more efficient, at least do not recompute door point on every scan
                    for door in self.semantic_map.doors:
                        pivot = np.array([door.pivot.x, door.pivot.y])
                        extent = np.array([door.extent.x, door.extent.y])
                        radius = np.linalg.norm(extent - pivot)
                        radious_margin = 1.1

                        if np.linalg.norm(pos_map[:2] - pivot) < radius * radious_margin:
                            msg.ranges[i] = 0

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
                rospy.logwarn(f"Could not transform {msg.header.frame_id} to {self.semantic_map.header.frame_id}: {ex}. Will forward scan as-is.")

        self.pub.publish(msg)

    def semantic_map_callback(self, map: SemanticMap):
        self.semantic_map = map


if __name__ == "__main__":
    rospy.init_node("laser_filter")
    filter = LaserFilter()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
