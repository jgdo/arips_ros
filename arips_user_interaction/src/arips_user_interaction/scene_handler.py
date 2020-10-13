from intent_handler import IntentHandler
import moveit_commander
import sys

class SceneIntentHandler(IntentHandler):
    def __init__(self, ui):
        super(IntentHandler, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        self.ui = ui
        self.scene = moveit_commander.PlanningSceneInterface()

    def handle_intent(self, msg):
        objects = self.scene.get_objects([])
        if objects:
            self.ui.say("I see {} object{}".format(len(objects), "s" if len(objects) > 1 else ""))
        else:
            self.ui.say("I don't see any objects")
        return None

    def stop(self):
        return None

    def get_required_capabilities(self):
        return []
