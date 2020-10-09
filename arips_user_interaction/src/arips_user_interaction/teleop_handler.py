from intent_handler import IntentHandler
import capabilities.srv
import rospy

class JoyTeleopIntentHandler(IntentHandler):
    CAPABILITY_NAME = 'arips_launch/Teleop'

    def __init__(self):
        super(IntentHandler, self).__init__()
        self.start_cap_service = rospy.ServiceProxy('/capability_server/start_capability',
                                                    capabilities.srv.StartCapability)
        self.stop_cap_service = rospy.ServiceProxy('/capability_server/stop_capability',
                                                    capabilities.srv.StopCapability)

    def handle_intent(self, msg):
        try:
            res = self.start_cap_service(self.CAPABILITY_NAME, '')

            if res.successful:
                rospy.logerr("Started teleop")
                return self
            else:
                rospy.logerr("Failed to start teleop")
                return None

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return None

    def stop(self):
        rospy.loginfo("Stopping teleop")
        res = self.stop_cap_service(self.CAPABILITY_NAME)
        return None

    def get_required_capabilities(self):
        return []
