from arips_user_interaction.intent_handler import IntentHandler
import rospy
from std_msgs.msg import Float32

class LookIntentHandler(IntentHandler):
    def __init__(self):
        super(IntentHandler, self).__init__()
        self.tilt_pub = rospy.Publisher('/kinect_tilt_deg', Float32, queue_size=10)

    def handle_intent(self, msg):
        slots = dict(zip(msg.slots, msg.values))
        self.tilt_pub.publish(Float32(float(slots['angle'])))
        return None

    def stop(self):
        return None

    def get_required_capabilities(self):
        return []
