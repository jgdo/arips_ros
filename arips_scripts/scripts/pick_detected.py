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

def go(group):
    for i in range(0, 3):
        if group.go():
            return True
    return False
    

def setGripper(width):
    global gripper
    gripper.clear_pose_targets()
    gripper.set_joint_value_target([width])
    gripper.plan()
    gripper.go()
    

def openGripper():
    setGripper(0.06)
    
def closeGripper():
    setGripper(0.027)
    
def callback(point):
    # point = point.point
    global arm
    global gripper
    rospy.loginfo("picking at " + str(point))
    
    
    point.x += 0.03
    point.y += -0.000
    print "picking at " + str(point)
    print "# open gripper"
    
    openGripper()
    
    rospy.sleep(0.5)
    
    print "# approach"

    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.0, 1.57, 0))
    pose_target.position.x = point.x
    pose_target.position.y = point.y
    pose_target.position.z = point.z + 0.05
    arm.clear_pose_targets()
    arm.set_pose_target(pose_target)
    if(not go(arm)):
        return
    
    rospy.sleep(3)
    
    print "pick"
    
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.0, 1.57, 0))
    pose_target.position.x = point.x
    pose_target.position.y = point.y
    pose_target.position.z = point.z
    arm.clear_pose_targets()
    arm.set_pose_target(pose_target)
    if(not go(arm)):
        return
    
    rospy.sleep(0.5)
    
    print "close gripper"
    
    closeGripper()
    
    rospy.sleep(1)
    
    print "go drop"
    
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 1.0
    pose_target.position.x = 0.3
    pose_target.position.y = 0.0
    pose_target.position.z = 0.2
    arm.clear_pose_targets()
    arm.set_pose_target(pose_target)
    if(not go(arm)):
        return
    
    rospy.sleep(0.5)
    
    print "idle"
    
    openGripper()
    
    rospy.sleep(0.5)
    
    print "go home"
    
    arm.clear_pose_targets()
    arm.set_joint_value_target([0.0, -0.6, 0.1, 0, 0])
    if(not go(arm)):
        return
        
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
    # sub = rospy.Subscriber("/clicked_point", geometry_msgs.msg.PoseStamped, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
