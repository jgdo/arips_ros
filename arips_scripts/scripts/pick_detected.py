#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf_conversions

robot = None
scene = None
arm = None
gripper = None
    
def callback(point):
    global arm
    global gripper
    rospy.loginfo("picking at " + str(point))
    
    gripper.clear_pose_targets()
    gripper.set_joint_value_target([0.3])
    gripper.plan()
    gripper.go()

    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.0, 1.57, 0))
    pose_target.position.x = point.x
    pose_target.position.y = point.y
    pose_target.position.z = point.z + 0.07
    arm.set_pose_target(pose_target)
    plan1 = arm.plan()
    arm.go()
    
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.0, 1.57, 0))
    pose_target.position.x = point.x
    pose_target.position.y = point.y
    pose_target.position.z = point.z
    arm.set_pose_target(pose_target)
    plan1 = arm.plan()
    arm.go()
    
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.0, -1, 0))
    pose_target.position.x = 0.05
    pose_target.position.y = 0
    pose_target.position.z = 0.4
    arm.set_pose_target(pose_target)
    plan1 = arm.plan()
    arm.go()
        
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('listener', anonymous=True)
    
    global robot
    global scene
    global arm
    global gripper
    
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    arm = moveit_commander.MoveGroupCommander("arm")
    gripper = moveit_commander.MoveGroupCommander("gripper")

    rospy.Subscriber("/pick_pose", Point, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
