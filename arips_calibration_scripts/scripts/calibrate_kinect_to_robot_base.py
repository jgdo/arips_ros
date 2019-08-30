#!/usr/bin/env python

import numpy as np


# Ros libraries
import roslib
import rospy
import tf
import tf.transformations
import sys

# We do not use cv_bridge it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError

VERBOSE = False


class kinect_calibrator:
    def __init__(self):
        self.tf_listener = tf.TransformListener()
        rospy.Timer(rospy.Duration(2), self.check_transform)

    def getTransform(self, dst, src):
        pos, quat = self.tf_listener.lookupTransform(dst, src, rospy.Time(0))
        return tf.TransformerROS().fromTranslationRotation(pos, quat)

    def check_transform(self, event):
        try:
            t_markerfloor_markerrobot = self.getTransform("/marker_floor", "/marker_robot")

            print "t_markerfloor_markerrobot", t_markerfloor_markerrobot

            t_ideal_base_markerfloor = tf.TransformerROS().fromTranslationRotation([0.30, 0.0, 0.0], [0, 0, 0, 1])

            t_base_markerrobot = np.matmul(t_ideal_base_markerfloor, t_markerfloor_markerrobot)

            print "t_base_markerrobot", t_base_markerrobot

            trans = tf.transformations.translation_from_matrix(t_base_markerrobot)
            quat = tf.transformations.quaternion_from_matrix(t_base_markerrobot)

            print "transform"
            print trans, tf.transformations.euler_from_quaternion(quat)

            print
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass


def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('image_feature', anonymous=True)
    kc = kinect_calibrator()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"

if __name__ == '__main__':
    main(sys.argv)
