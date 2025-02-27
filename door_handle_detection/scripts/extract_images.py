#!/bin/python3

import rosbag
import rospy

out_folder = "/home/jgdo/catkin_ws/src/arips_ros/door_handle_detection/data/door_handle_with_gipper/"

# export every nth frame
every = 3

count = 0
bag = rosbag.Bag('/home/jgdo/catkin_ws/src/arips_ros/door_handle_detection/bags/2021-04-29-19-27-32.bag')
for index, (topic, msg, t) in enumerate(bag.read_messages(topics=['/kinect/rgb/image_color/compressed'])):
    name = "frame_{}_{}.jpg".format(msg.header.stamp.secs, msg.header.stamp.nsecs)

    if index % every == 0:
        print(name)
        count += 1
        with open(out_folder + name, "wb") as file:
            file.write(msg.data)
bag.close()

print("Exported {} frames.".format(count))
