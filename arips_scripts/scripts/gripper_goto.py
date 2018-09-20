#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf_conversions

print "============ Starting tutorial setup"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm = moveit_commander.MoveGroupCommander("arm")
gripper = moveit_commander.MoveGroupCommander("gripper")


rospy.sleep(1)

gripper.set_goal_tolerance(0.001)


#print "current pose: ", gripper.get_current_pose()
#print "current rpy: ", gripper.get_current_rpy()
#print "joints: ", gripper.get_current_joint_values()

print "============ Generating plan 1"
gripper.clear_pose_targets()
gripper.set_joint_value_target([float(sys.argv[1]) if len(sys.argv) >= 2 else 0.03])
gripper.plan()
gripper.go()

rospy.sleep(1)

