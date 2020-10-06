

class UserInteraction:
    def __init__(self):
        self.intent_table = {}
        self.active_handler = None

    def intent_callback(self, msg):
        if msg.intent == 'stop':
            # special handing for stop intent
            if self.active_handler is not None:
                self.active_handler.stop()
                self.active_handler = None
            return None
        elif self.active_handler is not None:
            # cannot run intent since already occupied
            return self.user_respond_fail()
        elif msg.intent in self.intent_table:
            # handle new valid intent
            handler = self.intent_table[msg.intent]

            # check if all required capabilities are active and activate if not
            active_capabilities = self.get_active_capabilities()
            required_capabilities = handler.get_required_capabilities()
            if required_capabilities not in active_capabilities:
                if not self.activate_required_capabilities(required_capabilities):
                    return self.user_respond_fail()

            # handle intent. Intent can be handled either directly or over long time
            res = handler.handle_intent(msg.intent)
            if res is not None:
                self.active_handler = res

            return None
        else:
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
        pass

    def activate_required_capabilities(self, capabilities):
        pass

    def user_respond_fail(self):
        '''
        Respond to the user that an action is not possible

        :return: None
        '''
        pass