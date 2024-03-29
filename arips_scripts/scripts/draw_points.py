#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf_conversions

print("============ Starting tutorial setup")
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm = moveit_commander.MoveGroupCommander("arm")
gripper = moveit_commander.MoveGroupCommander("gripper")


#print("============ Waiting for RVIZ..."
#rospy.sleep(1)
#print("============ Starting tutorial "
#print("============ Reference frame: %s" % arm.get_planning_frame()
#print("============ Reference frame: %s" % arm.get_end_effector_link()
#print("============ Robot Groups:"
#print(robot.get_group_names()
#print("============ Printing robot state"
#print(robot.get_current_state()
print("============")


print("============ Generating plan 1")

def drawAt(x, y, z):
    global arm
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.0, 1.57, 0))
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = 0
    arm.set_pose_target(pose_target)
    arm.go()
    
    rospy.sleep(0.5)
    
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.0, 1.57, 0))
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z
    arm.set_pose_target(pose_target)
    arm.go()
    
    rospy.sleep(0.5)
    
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.0, 1.57, 0))
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = 0
    arm.set_pose_target(pose_target)
    arm.go()
    
    rospy.sleep(0.5)


drawAt(0.25, 0.15, -0.09)
drawAt(0.25, 0.0, -0.08)
drawAt(0.2, 0.0, -0.08)
drawAt(0.25, -0.15, -0.08)
drawAt(0.2, -0.15, -0.08)
drawAt(0.2, 0.15, -0.09)
drawAt(0.15, 0.15, -0.95)
drawAt(0.15, -0.15, -0.95)
drawAt(0.15, -0.0, -0.95)

arm.clear_pose_targets()
arm.set_joint_value_target([0.0, -0.6, 0.1, 0, 0])
arm.go()

rospy.sleep(0.5)


#arm.clear_pose_targets()
#arm_variable_values = arm.get_current_joint_values()
#arm_variable_values[2] = 0.0
#arm.set_joint_value_target(arm_variable_values)
#arm.plan()
#arm.go()

