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


class arm_calibrator:
    def __init__(self):
        self.tf_listener = tf.TransformListener()
        rospy.Timer(rospy.Duration(2), self.check_transform)

    def getTransform(self, dst, src):
        pos, quat = self.tf_listener.lookupTransform(dst, src, rospy.Time(0))
        return tf.TransformerROS().fromTranslationRotation(pos, quat)

    def check_transform(self, event):
        t_base_marker = self.getTransform("/arips_base", "/ar_marker_0")

        #print("t_base_marker", t_base_marker

        t_arm_tool0 = self.getTransform("/base_link", "/tool0")
        t_tool0_marker = tf.TransformerROS().fromTranslationRotation([-0.0475, 0.001, 0.03], [0, 0, 0, 1])
        t_arm_marker = np.matmul(t_arm_tool0, t_tool0_marker)
        t_marker_arm = np.linalg.inv(t_arm_marker)

        t_base_arm_corrected = np.matmul(t_base_marker, t_marker_arm);

        np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

        #print("t_arm_base_corrected"
        #print(t_arm_base_corrected

        trans = tf.transformations.translation_from_matrix(t_base_arm_corrected)
        quat = tf.transformations.quaternion_from_matrix(t_base_arm_corrected)

        print("t_base_arm_corrected")
        print(t_base_arm_corrected)

        print("transform")
        print(trans, quat)
        print()


def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('image_feature', anonymous=True)
    ac = arm_calibrator()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")

if __name__ == '__main__':
    main(sys.argv)