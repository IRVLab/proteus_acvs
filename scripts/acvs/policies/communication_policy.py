class CommunicationPolicy(object):
    def __init__(self, language):
        self.language = language
        self.symbols = self.language.out_symbols

    # Given a message and the current context, return a selected vector for the communication.
    def select_vector(self):
        raise NotImplementedError()

    # If necessary, update the policy's rules
    def update_policy(self):
        raise NotImplementedError()
