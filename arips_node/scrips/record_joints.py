#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState


f = None

def callback(data):
    global f
    # f.write(str(data.header.stamp.secs + data.header.stamp.nsecs * 1e-9) + ';'+ str(data.position[0]) + ';' + str(data.velocity[0]) + ';' + str(data.effort[0]) + '\n')
    f.write('%.9f;%.9f;%.9f;%.9f;%.9f;%.9f\n' % (data.position[0], data.position[1], data.velocity[0], data.velocity[1], data.effort[0], data.effort[1]))
    
def listener():
    global f
    f = open('traj_record.csv', 'w')

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaenously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/joint_states", JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
    f.close()
        
if __name__ == '__main__':
    listener()


