#!/usr/bin/env python3

import numpy as np


# Ros libraries
import roslib
import rospy
import tf
import tf.transformations
import sys
import numpy as np

# We do not use cv_bridge it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError

VERBOSE = False


class kinect_calibrator:
    def __init__(self):
        self.tf_listener = tf.TransformListener()
        rospy.Timer(rospy.Duration(0.3), self.check_transform)
        self.tf_br = tf.TransformBroadcaster()

    def getTransform(self, dst, src):
        pos, quat = self.tf_listener.lookupTransform(dst, src, rospy.Time(0))
        return tf.TransformerROS().fromTranslationRotation(pos, quat)

    def check_transform(self, event):   
        try: 
            self.check_transform_robot()
        
            (trans1,rot1) = self.tf_listener.lookupTransform('/kinect_link', '/ar_marker_10', rospy.Time(0))
            (trans2,rot2) = self.tf_listener.lookupTransform('/kinect_link', '/ar_marker_11', rospy.Time(0))
            (trans3,rot3) = self.tf_listener.lookupTransform('/kinect_link', '/ar_marker_12', rospy.Time(0))
            (trans4,rot4) = self.tf_listener.lookupTransform('/kinect_link', '/ar_marker_13', rospy.Time(0))
            
            v0 = np.zeros((3, 4))
            v0[:, 0] = trans1
            v0[:, 1] = trans2
            v0[:, 2] = trans3
            v0[:, 3] = trans4
                        
            v1 = np.zeros((3, 4))
            v1[:, 0] = [0.28, -0.2, 0.0]
            v1[:, 1] = [0.28, 0.2, 0.0]
            v1[:, 2] = [0.0, 0.2, 0.0]
            v1[:, 3] = [0.0, -0.2, 0.0]
            
            M = tf.transformations.superimposition_matrix(v1, v0)
            
            #print(M
            rospy.logdebug("publishing marker_floor")
            
            self.tf_br.sendTransform(tf.transformations.translation_from_matrix(M),
                         tf.transformations.quaternion_from_matrix(M),
                         rospy.Time.now(),
                         "marker_floor",
                         "kinect_link")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
                     
    def check_transform_robot(self):
        (trans1,rot1) = self.tf_listener.lookupTransform('/kinect_link', '/fiducial_14', rospy.Time(0))
        (trans2,rot2) = self.tf_listener.lookupTransform('/kinect_link', '/fiducial_11', rospy.Time(0))
        (trans3,rot3) = self.tf_listener.lookupTransform('/kinect_link', '/fiducial_13', rospy.Time(0))
        (trans4,rot4) = self.tf_listener.lookupTransform('/kinect_link', '/fiducial_12', rospy.Time(0))
        
        v0 = np.zeros((3, 4))
        v0[:, 0] = trans1
        v0[:, 1] = trans2
        v0[:, 2] = trans3
        v0[:, 3] = trans4
                    
        v1 = np.zeros((3, 4))
        v1[:, 0] = [0.08, -0.1, 0.0]
        v1[:, 1] = [0.08, 0.1, 0.0]
        v1[:, 2] = [0.0, 0.1, 0.0]
        v1[:, 3] = [0.0, -0.1, 0.0]
        
        M = tf.transformations.superimposition_matrix(v1, v0)
        
        #print(M
        rospy.logdebug("publishing marker_robot")
        
        self.tf_br.sendTransform(tf.transformations.translation_from_matrix(M),
                     tf.transformations.quaternion_from_matrix(M),
                     rospy.Time.now(),
                     "marker_robot",
                     "kinect_link")

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('image_feature', anonymous=True)
    kc = kinect_calibrator()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")

if __name__ == '__main__':
    main(sys.argv)
