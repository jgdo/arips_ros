from intent_handler import IntentHandler
import moveit_commander
import sys
import rospy
import geometry_msgs.msg
import tf_conversions

class SceneIntentHandler(IntentHandler):
    def __init__(self, ui):
        super(IntentHandler, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        self.ui = ui
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.gripper = moveit_commander.MoveGroupCommander("gripper")

    def handle_intent(self, msg):
        if msg.intent == 'describe_scene':
            self.describe_scene()
        elif msg.intent == 'pick_up':
            self.pickup(msg)
        else:
            self.ui.say("Cannot handle {}".format(msg.intent))

        return None

    def stop(self):
        return None

    def get_required_capabilities(self):
        return []

    def describe_scene(self):
        objects = self.scene.get_objects([])
        if objects:
            self.ui.say("I see {} object{}".format(len(objects), "s" if len(objects) > 1 else ""))
        else:
            self.ui.say("I don't see any objects")

    def pickup(self, msg):
        slots = dict(zip(msg.slots, msg.values))
        objects = self.scene.get_objects([])
        if not objects:
            self.ui.say("I don't see any objects")
            return
        elif len(objects) > 1:
            if 'position' not in slots:
                self.ui.say("I see too many objects, please pick one")
                return
            if slots['position'] == 'any':
                point = next(iter(objects.values())).primitive_poses[0].position
            elif len(objects) == 2:
                # sort objects
                poses = [p.primitive_poses[0].position for p in objects.values()]
                poses = sorted(poses, cmp=lambda a, b: a.y < b.y)
                if slots['position'] == 'left':
                    point = poses[0]
                else:
                    point = poses[1]
            else:
                self.ui.say("I see too many objects, please pick one")
                return
        else:
            point = next(iter(objects.values())).primitive_poses[0].position

        # point = point.point
        rospy.loginfo("picking at " + str(point))

        point.x += 0.013
        point.y += 0.008
        point.z -= 0.010
        print "picking at " + str(point)
        print "# open gripper"

        self.open_gripper()

        rospy.sleep(0.5)

        print "# approach"

        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation = geometry_msgs.msg.Quaternion(
            *tf_conversions.transformations.quaternion_from_euler(0.0, 1.57, 0))
        pose_target.position.x = point.x
        pose_target.position.y = point.y
        pose_target.position.z = point.z + 0.03
        self.arm.clear_pose_targets()
        self.arm.set_pose_target(pose_target)
        if (not self.go(self.arm)):
            return

        rospy.sleep(3)

        print "pick"

        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation = geometry_msgs.msg.Quaternion(
            *tf_conversions.transformations.quaternion_from_euler(0.0, 1.57, 0))
        pose_target.position.x = point.x
        pose_target.position.y = point.y
        pose_target.position.z = point.z
        self.arm.clear_pose_targets()
        self.arm.set_pose_target(pose_target)
        if (not self.go(self.arm)):
            return

        rospy.sleep(0.5)

        print "close gripper"

        self.close_gripper()

        rospy.sleep(1)

        print "go drop"

        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.w = 1.0
        pose_target.position.x = 0.3
        pose_target.position.y = 0.0
        pose_target.position.z = 0.2
        self.arm.clear_pose_targets()
        self.arm.set_pose_target(pose_target)
        if (not self.go(self.arm)):
            return

        rospy.sleep(0.5)

        print "idle"

        self.open_gripper()

        rospy.sleep(0.5)

        print "go home"

        self.arm.clear_pose_targets()
        self.arm.set_joint_value_target([0.0, 0.0, -1.3, 0.0, 0])
        self.go(self.arm)

    def go(self, group):
        for i in range(0, 3):
            if group.go():
                return True
        return False

    def set_gripper(self, width):
        self.gripper.clear_pose_targets()
        self.gripper.set_joint_value_target([width])
        self.gripper.plan()
        self.gripper.go()

    def open_gripper(self):
        self.set_gripper(0.06)

    def close_gripper(self):
        self.set_gripper(0.02)