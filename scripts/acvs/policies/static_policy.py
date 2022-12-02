from acvs.policies.communication_policy import CommunicationPolicy

class StaticPolicy(CommunicationPolicy):
    def __init__(self, language, vector_id):
        super().init(self, language)
        self.static_vector = vector_id

    # Given a message and the current context, return a selected vector for the communication.
    def select_vector(self):
        return self.static_vector

    # If necessary, update the policy's rules
    def update_policy(self,):
        raise NotImplementedError()
