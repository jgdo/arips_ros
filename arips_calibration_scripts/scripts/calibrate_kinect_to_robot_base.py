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
        t_marker_kinect = self.getTransform("/kinect_base", "/ar_marker_1")

        print "t_marker_kinect", t_marker_kinect

        t_ideal_marker_base = tf.TransformerROS().fromTranslationRotation([0.41, 0.0, 0.0], [0, 0, 0, 1])

        t_kinect_base_corrected = np.matmul(t_ideal_marker_base, np.linalg.inv(t_marker_kinect))

        print "t_kinect_base_corrected", t_kinect_base_corrected

        trans = tf.transformations.translation_from_matrix(t_kinect_base_corrected)
        quat = tf.transformations.quaternion_from_matrix(t_kinect_base_corrected)

        print "transform"
        print trans, tf.transformations.euler_from_quaternion(quat)

        print


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
