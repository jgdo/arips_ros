#!/bin/python3

import rosbag
import rospy
import numpy as np
import cv2

in_bag = "/home/jgdo/catkin_ws/src/arips_data/bags/floor/2022-02-12-10-35-56.bag"
out_folder = "/home/jgdo/catkin_ws/src/arips_data/datasets/floor/all/"

#image_topic = '/kinect/rgb/image_rect_color'
image_topic = '/kinect/rgb/image_rect_color/compressed'

# export every nth frame
every = 3

def write_data(data, path, compression):
    if compression:
        with open(path, "wb") as file:
            file.write(data)
    else:
        img = np.frombuffer(data, dtype=np.uint8).reshape((480, 640, 3))
        cv2.imwrite(path, img)

count = 0
bag = rosbag.Bag(in_bag)
for index, (topic, msg, t) in enumerate(bag.read_messages(topics=[image_topic])):
    name = "frame_{}_{}.jpg".format(msg.header.stamp.secs, msg.header.stamp.nsecs)

    if index % every == 0:
        print(name)
        count += 1
        write_data(msg.data, out_folder + name, image_topic.endswith('compressed'))
bag.close()

print("Exported {} frames.".format(count))
