

class IntentHandler(object):
    def __init__(self):
        pass

    def handle_intent(self, intent):
        raise NotImplementedError()

    def stop(self):
        raise NotImplementedError()

    def get_required_capabilities(self):
        return []
