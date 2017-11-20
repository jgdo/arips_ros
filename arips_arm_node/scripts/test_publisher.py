#!/usr/bin/env python
# license removed for brevity
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def talker():
    pub = rospy.Publisher('trajectory', JointTrajectory, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rospy.Rate(1).sleep();
    traj = JointTrajectory()
    traj.points = [JointTrajectoryPoint() for i in range(2)]
    traj.points[0].positions = [0]
    traj.points[0].velocities = [0]
    traj.points[0].accelerations = [0]
    traj.points[0].time_from_start = rospy.Time.from_sec(0.0)
    traj.points[1].positions = [1]
    traj.points[1].velocities = [0]
    traj.points[1].accelerations = [0]
    traj.points[1].time_from_start = rospy.Time.from_sec(0.2)
    pub.publish(traj)
    rospy.spin()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
        
