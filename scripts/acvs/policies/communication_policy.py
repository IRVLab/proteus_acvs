class CommunicationPolicy(object):
    def __init__(self, lang_tup):
        self.symbols = lang_tup[0]

    # Given a message and the current context, return a selected vector for the communication.
    def select_vector(self):
        raise NotImplementedError()

    # If necessary, update the policy's rules
    def update_policy(self):
        raise NotImplementedError()
