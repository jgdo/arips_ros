#!/usr/bin/env python

import rhasspy_ros_interface.msg
import rospy
from arips_user_interaction import intent_handler, teleop_handler, look_handler

class UserInteraction:
    def __init__(self):
        self.intent_table = {}
        self.active_handler = None

        rospy.Subscriber("speech_intent", rhasspy_ros_interface.msg.Intent, self.intent_callback)

    def intent_callback(self, msg):
        if msg.intent == 'stop':
            rospy.loginfo("Received stop intent")

            # special handing for stop intent
            if self.active_handler is not None:
                self.active_handler.stop()
                self.active_handler = None
            else:
                rospy.loginfo("Nothing to stop")

            return None
        elif self.active_handler is not None:
            rospy.logerr("Cannot start new intent since a different intent is already running")
            # cannot run intent since already occupied
            return self.user_respond_fail()
        elif msg.intent in self.intent_table:
            rospy.loginfo("Handling intent '{}'".format(msg.intent))

            # handle new valid intent
            handler = self.intent_table[msg.intent]

            # check if all required capabilities are active and activate if not
            active_capabilities = self.get_active_capabilities()
            required_capabilities = handler.get_required_capabilities()
            if any(cap not in active_capabilities for cap in required_capabilities):
                if not self.activate_required_capabilities(required_capabilities):
                    rospy.logerr("Could not activate required capabilities for intent '{}', ignoring".format(msg.intent))
                    return self.user_respond_fail()

            # handle intent. Intent can be handled either directly or over long time
            res = handler.handle_intent(msg)
            if res is not None:
                self.active_handler = res

            return None
        else:
            rospy.logerr("Cannot find handler for intent '{}', ignoring".format(msg.intent))
            return self.user_respond_fail()

    def handle_intent_finished(self, handler):
        '''
        Mark current intent as done, go to idle state. Called by an intent handler when it finished it's job.

        :param handler: intent handler, should be same as self.active_handler
        :return: None
        '''
        assert handler == self.active_handler
        self.active_handler = None

    def get_active_capabilities(self):
        return [] # TODO

    def activate_required_capabilities(self, capabilities):
        return True

    def user_respond_fail(self):
        '''
        Respond to the user that an action is not possible

        :return: None
        '''
        pass

    def add_intent(self, intent, handler):
        self.intent_table[intent] = handler

class TestIntentHandler(intent_handler.IntentHandler):
    def __init__(self):
        super(TestIntentHandler, self).__init__()

    def handle_intent(self, msg):
        print("Handling intent {}".format(str(msg)))
        return None

    def stop(self):
        pass

    def get_required_capabilities(self):
        return []

def main():
    rospy.init_node('user_interaction_node')
    ui = UserInteraction()
    ui.add_intent('test_intent', TestIntentHandler())
    ui.add_intent('teleop_joy', teleop_handler.JoyTeleopIntentHandler())
    ui.add_intent('kinect_tilt', look_handler.LookIntentHandler())

    rospy.spin()

if __name__ == '__main__':
    main()
