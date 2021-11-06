from arips_user_interaction.intent_handler import IntentHandler
import capabilities.srv
import rospy

class ShutdownHandler(IntentHandler):
    CAPABILITY_NAME = 'arips_launch/shutdown'

    def __init__(self):
        super(IntentHandler, self).__init__()
        self.start_cap_service = rospy.ServiceProxy('/capability_server/start_capability',
                                                    capabilities.srv.StartCapability)

    def handle_intent(self, msg):
        try:
            res = self.start_cap_service(self.CAPABILITY_NAME, '')

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)


    def get_required_capabilities(self):
        return []
