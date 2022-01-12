import copy
import math

import tf.transformations
from arips_user_interaction.intent_handler import IntentHandler
import moveit_commander
import sys
import rospy
import geometry_msgs.msg
import tf_conversions
from typing import Tuple

from moveit_msgs.msg import MoveItErrorCodes


class SceneIntentHandler(IntentHandler):
    def __init__(self, ui):
        super(IntentHandler, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        self.ui = ui
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.gripper = moveit_commander.MoveGroupCommander("gripper")
        self.arm.set_max_velocity_scaling_factor(1.0)
        self.gripper.set_max_velocity_scaling_factor(1.0)

        self.pick_pose_pub = rospy.Publisher('pick_target', geometry_msgs.msg.PoseStamped, queue_size=1)

    def handle_intent(self, msg):
        if msg.intent == 'describe_scene':
            self.describe_scene()
        elif msg.intent == 'pick_object':
            self.handle_pickup(msg)
        elif msg.intent == 'drop_object':
            self.handle_drop(msg)
        elif msg.intent == 'place_on_object':
            self.handle_place_on_object(msg)
        else:
            self.ui.say("Cannot handle {}".format(msg.intent))

        return None

    def stop(self):
        return None

    def get_required_capabilities(self):
        return []

    def describe_scene(self):
        objects = self.scene.get_objects([])
        colors = self.scene.get_object_colors()

        print(colors)

        if objects:
            self.ui.say("I see {} object{}".format(len(objects), "s" if len(objects) > 1 else ""))
        else:
            self.ui.say("I don't see any objects")
    
    def select_object_pose(self, msg) -> Tuple[geometry_msgs.msg.Pose, str]:
        slots = dict(zip(msg.slots, msg.values))
        objects = list(self.scene.get_objects([]).values())

        color_filter = slots['color'] if 'color' in slots else ''

        if color_filter:
            objects = list(filter(lambda obj: obj.type.key == color_filter, objects))

        if not objects:
            self.ui.say(f"I don't see any {color_filter} objects")
            return None, color_filter

        if len(objects) > 1:
            if 'position' not in slots:
                self.ui.say("I see too many objects, please select the left or the right one")
                return None, color_filter
            if slots['position'] == 'any':
                pose = objects[0].primitive_poses[0]
            elif len(objects) == 2 and slots['position'] != "the":
                # sort objects
                poses = [p.primitive_poses[0] for p in objects]
                poses = sorted(poses, key=lambda x: x.position.y)
                if slots['position'] == 'left':
                    pose = poses[0]
                elif slots['position'] == 'right':
                    pose = poses[1]
                else:
                    self.ui.say(f"I don't know what the {slots['position']} object is, please choose left or right.")
                    return None, color_filter
            else:
                self.ui.say("I see too many objects, please pick one")
                return None, color_filter
        else:
            pose = objects[0].primitive_poses[0]
        
        pose.position.x += 0.01
        pose.position.y += 0.02
        pose.position.z += 0.005

        return pose, color_filter

    def handle_pickup(self, msg):
        pose, color = self.select_object_pose(msg)

        if pose is None:
            return

        # point = point.point
        rospy.loginfo("picking at " + str(pose))

        self.ui.say(f"I will now pick up the {color} object")
        print("picking at " + str(pose))
        
        try:
            self.pickup_object_at(pose)
        except Exception as ex:
            rospy.logerr("I could not pick up the object: " + str(ex))
            self.ui.say("I could not pick up the object: " + str(ex))

    def handle_place_on_object(self, msg):
        pose, color = self.select_object_pose(msg)

        if pose is None:
            return
        
        pose.position.z += 0.035

        self.ui.say(f"I will now place the object onto {color} object")
        
        try:
            self.place_object_at(pose.position)
        except Exception as ex:
            rospy.logerr("I could not place on the object: " + str(ex))
            self.ui.say("I could not place on the object: " + str(ex))

    def handle_drop(self, msg):
        self.ui.say("I will now drop the object")
        try:
            self.drop_object()
        except Exception as ex:
            self.ui.say("I could not drop the object: " + str(ex))

    def pickup_object_at(self, pose: geometry_msgs.msg.Pose):
        print("# open gripper")
        self.open_gripper()

        rospy.sleep(0.3)

        print(f"# approach {pose}")

        tcp_pose = copy.deepcopy(pose)
        # we want to grab the object from above

        quat = [tcp_pose.orientation.x, tcp_pose.orientation.y, tcp_pose.orientation.z, tcp_pose.orientation.w]
        object_rpy = tf.transformations.euler_from_quaternion(quat)

        print(f"object has rpy = {object_rpy}")

        tcp_pose.orientation = geometry_msgs.msg.Quaternion(
            *tf.transformations.quaternion_multiply(
                #quat,
                tf.transformations.quaternion_from_euler(0.0,
                                                         0.0,
                                                         object_rpy[2]),
                tf.transformations.quaternion_from_euler(0.0,
                                                         math.pi/2,
                                                         0.0)))

        #tcp_pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0.0, math.pi/2, 0.0))

        pose_target = copy.deepcopy(tcp_pose)
        pose_target.position.z += 0.03

        stamped_msg = geometry_msgs.msg.PoseStamped()
        stamped_msg.header.frame_id = "arm_base_link"
        stamped_msg.header.stamp = rospy.Time.now()
        stamped_msg.pose = pose_target
        self.pick_pose_pub.publish(stamped_msg)

        self.arm.clear_pose_targets()
        self.arm.set_pose_target(pose_target)
        ok, _, _, _ = self.arm.plan(pose_target)
        if not ok:
            tcp_pose.orientation = geometry_msgs.msg.Quaternion(
                *tf.transformations.quaternion_multiply(
                    # quat,
                    tf.transformations.quaternion_from_euler(0.0,
                                                             0.0,
                                                             object_rpy[2] + math.pi),
                    tf.transformations.quaternion_from_euler(0.0,
                                                             math.pi / 2,
                                                             0.0)))
            pose_target = copy.deepcopy(tcp_pose)
            pose_target.position.z += 0.03

            stamped_msg = geometry_msgs.msg.PoseStamped()
            stamped_msg.header.frame_id = "arm_base_link"
            stamped_msg.header.stamp = rospy.Time.now()
            stamped_msg.pose = pose_target
            self.pick_pose_pub.publish(stamped_msg)

            self.arm.clear_pose_targets()
            self.arm.set_pose_target(pose_target)

        self.go(self.arm)

        rospy.sleep(0.3)


        print("pick")

        self.arm.clear_pose_targets()
        self.arm.set_pose_target(tcp_pose)
        self.go(self.arm)

        rospy.sleep(0.3)

        print("close gripper")

        self.close_gripper()

        rospy.sleep(6)


        """
        print("go drop")

        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.w = 1.0
        pose_target.position.x = 0.3
        pose_target.position.y = 0.0
        pose_target.position.z = 0.2
        self.arm.clear_pose_targets()
        self.arm.set_pose_target(pose_target)
        self.go(self.arm)

        rospy.sleep(0.3)

        print("idle")

        self.open_gripper()

        rospy.sleep(0.3)

        print("go home")
        """

        self.goto_idle_pose()

    
    def place_object_at(self, point):
        print("# approach")

        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation = geometry_msgs.msg.Quaternion(
            *tf_conversions.transformations.quaternion_from_euler(0.0, 1.57, 0))
        pose_target.position.x = point.x
        pose_target.position.y = point.y
        pose_target.position.z = point.z + 0.03
        self.arm.clear_pose_targets()
        self.arm.set_pose_target(pose_target)
        self.go(self.arm)

        rospy.sleep(0.3)

        print("place")

        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation = geometry_msgs.msg.Quaternion(
            *tf_conversions.transformations.quaternion_from_euler(0.0, 1.57, 0))
        pose_target.position.x = point.x
        pose_target.position.y = point.y
        pose_target.position.z = point.z
        self.arm.clear_pose_targets()
        self.arm.set_pose_target(pose_target)
        self.go(self.arm)

        rospy.sleep(0.3)

        print("open gripper")

        self.open_gripper()

        rospy.sleep(0.3)


        self.goto_idle_pose()


    def goto_idle_pose(self):
        self.arm.clear_pose_targets()
        self.arm.set_joint_value_target([0.0, 0.0, -1.5, 0.0, 0])
        self.go(self.arm)


    def drop_object(self):
        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.w = 1.0
        pose_target.position.x = 0.3
        pose_target.position.y = 0.0
        pose_target.position.z = 0.0
        self.arm.clear_pose_targets()
        self.arm.set_pose_target(pose_target)
        self.go(self.arm)
        rospy.sleep(0.3)
        self.open_gripper()

        self.goto_idle_pose()

    def go(self, group):
        for i in range(0, 3):
            if group.go():
                return
        raise Exception("Could not move arm, aborting")

    def set_gripper(self, width):
        self.gripper.clear_pose_targets()
        self.gripper.set_joint_value_target([width])
        self.gripper.plan()
        self.gripper.go(wait=True)

    def open_gripper(self):
        self.set_gripper(0.06)

    def close_gripper(self):
        self.set_gripper(0.02)