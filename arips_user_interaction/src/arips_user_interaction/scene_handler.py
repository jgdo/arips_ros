from arips_user_interaction.intent_handler import IntentHandler
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
        self.arm.set_max_velocity_scaling_factor(1.0)
        self.gripper.set_max_velocity_scaling_factor(1.0)

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
        if objects:
            self.ui.say("I see {} object{}".format(len(objects), "s" if len(objects) > 1 else ""))
        else:
            self.ui.say("I don't see any objects")
    
    def select_object_pose(self, msg):
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
                point = objects[0].primitive_poses[0].position
            elif len(objects) == 2 and slots['position'] != "the":
                # sort objects
                poses = [p.primitive_poses[0].position for p in objects]
                poses = sorted(poses, key=lambda x: x.y)
                if slots['position'] == 'left':
                    point = poses[0]
                elif slots['position'] == 'right':
                    point = poses[1]
            else:
                self.ui.say("I see too many objects, please pick one")
                return None, color_filter
        else:
            point = objects[0].primitive_poses[0].position
        
        point.x += 0.013
        point.y += 0.01
        point.z += 0.005

        return point, color_filter

    def handle_pickup(self, msg):
        point, color = self.select_object_pose(msg)

        if point is None:
            return

        # point = point.point
        rospy.loginfo("picking at " + str(point))

        self.ui.say(f"I will now pick up the {color} object")
        print("picking at " + str(point))
        
        try:
            self.pickup_object_at(point)
        except Exception as ex:
            self.ui.say("I could not pick up the object: " + str(ex))


    def handle_place_on_object(self, msg):
        point, color = self.select_object_pose(msg)

        if point is None:
            return
        
        point.z += 0.035

        self.ui.say(f"I will now place the object onto {color} object")
        
        try:
            self.place_object_at(point)
        except Exception as ex:
            self.ui.say("I could not pick up the object: " + str(ex))
        
    
    def handle_drop(self, msg):
        self.ui.say("I will now drop the object")
        try:
            self.drop_object()
        except Exception as ex:
            self.ui.say("I could not drop the object: " + str(ex))


    def pickup_object_at(self, point):
        print("# open gripper")
        self.open_gripper()

        rospy.sleep(0.3)

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

        print("pick")

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

        print("close gripper")

        self.close_gripper()

        rospy.sleep(0.7)

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