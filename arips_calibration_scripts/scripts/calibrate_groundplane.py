#!/usr/bin/env python
import rospy
from pcl_msgs.msg import ModelCoefficients
import tf
import tf.transformations
import numpy as np
from std_msgs.msg import Float32
import sys

global tf_listener
global angle_counter
global times_counter
global last_time

global pub

global angle_input_table
global angle_table

def qv_mult(q1, v1):
    v1 = tf.transformations.unit_vector(v1)
    q2 = list(v1)
    q2.append(0.0)
    return tf.transformations.quaternion_multiply(
        tf.transformations.quaternion_multiply(q1, q2),
        tf.transformations.quaternion_conjugate(q1)
    )[:3]

def callback(msg):
    global angle_counter
    global times_counter
    global last_time
    global angle_table
    global angle_input_table

    set_angle = angle_input_table[angle_counter]

    if times_counter < 0:
        f32 = Float32()
        f32.data = set_angle
        pub.publish(f32)
        last_time = rospy.Time.now()
        times_counter = 0
        return
    elif times_counter == 0:
        if (rospy.Time.now() - last_time).to_sec() < 3.0:
            return

    normal_orig = msg.values[0:3]
    global tf_listener
    pos, quat = tf_listener.lookupTransform("/arips_base", msg.header.frame_id, rospy.Time(0))
    # print("quat = ", quat
    normal_rotated = qv_mult(quat, normal_orig)
    z = [0, 0, 1]
    angle = -np.arctan2(normal_rotated[0], normal_rotated[2])
    if angle < -np.pi/2:
        angle += np.pi

    angle_deg = np.rad2deg(angle)
    print("angle = ", angle_deg, " value = " , set_angle)

    if set_angle not in angle_table:
        angle_table[set_angle] = [set_angle, 0, 0]

    angle_table[set_angle][1] += angle_deg
    angle_table[set_angle][2] += 1

    times_counter += 1

    if times_counter >= 10:
        angle_counter += 1
        times_counter = -1

        if angle_counter >= len(angle_input_table):
            sys.stdout.write("angles: [")
            for elem in sorted(angle_table.values(), key=lambda e: -e[1] / e[2]):
                sys.stdout.write(str(-elem[1] / elem[2]) + ", ")
            sys.stdout.write("]\nraws: [")
            for elem in sorted(angle_table.values(), key=lambda e: -e[1] / e[2]):
                sys.stdout.write(str(-elem[0]) + ", ")
            sys.stdout.write("]\n")
            rospy.signal_shutdown('done')


def listener():
    global angle_counter
    global times_counter
    global last_time
    global angle_table
    global angle_input_table

    angle_counter = 0
    times_counter = -1
    angle_table = {}

    angle_input_table = range(-310, -451, -20) + range(-450, -309, 20)

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('groundplane_calibration', anonymous=True)

    global tf_listener
    global pub

    pub = rospy.Publisher("/kinect_tilt_deg", Float32, queue_size=10)

    tf_listener = tf.TransformListener()


    rospy.Subscriber("coefficients", ModelCoefficients, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
