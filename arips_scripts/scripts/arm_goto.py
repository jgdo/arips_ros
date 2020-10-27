#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf_conversions
import geometry_msgs.msg
from tf import TransformListener

print("============ Starting tutorial setup")
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)


tf_listener = TransformListener()

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm = moveit_commander.MoveGroupCommander("arm")
gripper = moveit_commander.MoveGroupCommander("gripper")


rospy.sleep(1)

arm.set_goal_tolerance(0.003)


print("current pose: ", arm.get_current_pose())
print("current rpy: ", arm.get_current_rpy())
print("joints: ", arm.get_current_joint_values())

print("============ Generating plan 1")
pose_target = geometry_msgs.msg.PoseStamped()
pose_target.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 1.57, 0))

pose_target.pose.position.x = float(sys.argv[1]) if len(sys.argv) >= 2 else 0.30
pose_target.pose.position.y = float(sys.argv[2]) if len(sys.argv) >= 3 else 0.0
pose_target.pose.position.z = float(sys.argv[3]) if len(sys.argv) >= 4 else 0.06

pose_target.header.frame_id = "/arips_base"

tp_base = tf_listener.transformPose("/base_link", pose_target)

arm.set_pose_target(tp_base.pose)
plan1 = arm.plan()
arm.go()

rospy.sleep(1)

