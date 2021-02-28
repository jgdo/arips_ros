#!/usr/bin/env python3

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import skimage.feature
import numpy as np

class image_converter:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/kinect/depth_registered/image_raw", Image, self.callback)

        self.clamp_min = 0
        self.clamp_max = 1500

        cv2.namedWindow("image")
        cv2.createTrackbar("min", "image", 0, 1500, self.on_trackbar_min)
        cv2.createTrackbar("max", "image", 0, 1500, self.on_trackbar_max)

    def callback(self, data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        except CvBridgeError as e:
            print(e)
            return

        img = cv2.GaussianBlur(img, (3, 3), 0)
        edges = cv2.Laplacian(img, ddepth=cv2.CV_32F)
        edges = (edges - edges.mean()) / (0.1*edges.std())
        edges = np.abs(edges)
        cv2.imshow("edges", edges)
        cv2.imshow("image", ((img - self.clamp_min) / float(self.clamp_max - self.clamp_min)).clip(0, 1))
        cv2.waitKey(3)

    def on_trackbar_min(self, val):
        self.clamp_min = val

    def on_trackbar_max(self, val):
        self.clamp_max = val

def main(args):
    rospy.init_node('image_converter', argv=args, anonymous=True)
    ic = image_converter()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

