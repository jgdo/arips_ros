#!/usr/bin/env python

import serial
import rospy
import atexit

from std_msgs.msg import Float32

def talker():
    rospy.init_node('battery_node')
    pub = rospy.Publisher('battery_level', Float32, queue_size=10)
    ser = serial.Serial('/dev/arips_battery_controller', 115200)
    ser.timeout = 0

    atexit.register(lambda: ser.write(b'0'))

    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        ser.read_all()
        ser.write(b'1')

        hello_str = "hello world %s" % rospy.get_time()
        pub.publish(hello_str)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass