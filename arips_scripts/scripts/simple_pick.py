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

arm.set_goal_tolerance(0.003)
gripper.set_goal_tolerance(0.001)

#print "============ Waiting for RVIZ..."
#rospy.sleep(1)
#print "============ Starting tutorial "
#print "============ Reference frame: %s" % arm.get_planning_frame()
#print "============ Reference frame: %s" % arm.get_end_effector_link()
#print "============ Robot Groups:"
#print robot.get_group_names()
#print "============ Printing robot state"
#print robot.get_current_state()
print "============"



gripper.clear_pose_targets()
gripper.set_joint_value_target([0.04])
gripper.plan()
gripper.go()

rospy.sleep(1)

print "============ Generating plan 1"
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.0, 1.57, 0))
pose_target.position.x = 0.2
pose_target.position.y = 0.0
pose_target.position.z = 0.07
arm.set_pose_target(pose_target)
plan1 = arm.plan()
arm.go()

rospy.sleep(1)

pose_target = geometry_msgs.msg.Pose()
pose_target.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.0, 1.57, 0))
pose_target.position.x = 0.2
pose_target.position.y = 0.0
pose_target.position.z = 0.045
arm.set_pose_target(pose_target)
plan1 = arm.plan()
arm.go()

rospy.sleep(1)

gripper.clear_pose_targets()
gripper.set_joint_value_target([0.028])
gripper.plan()
gripper.go()

rospy.sleep(1)

pose_target = geometry_msgs.msg.Pose()
pose_target.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.0, 1.57, 0))
pose_target.position.x = 0.12
pose_target.position.y = 0.12
pose_target.position.z = 0.11
arm.set_pose_target(pose_target)
plan1 = arm.plan()
arm.go()

rospy.sleep(1)

gripper.clear_pose_targets()
gripper.set_joint_value_target([0.04])
gripper.plan()
gripper.go()

rospy.sleep(1)

#arm.clear_pose_targets()
#arm_variable_values = arm.get_current_joint_values()
#arm_variable_values[2] = 0.0
#arm.set_joint_value_target(arm_variable_values)
#arm.plan()
#arm.go()

