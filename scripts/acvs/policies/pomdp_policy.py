from acvs.policies.communication_policy import CommunicationPolicy

class POMDPPolicy(CommunicationPolicy):
    def __init__(self, language):
        super().__init__(self, language)

        
    # Given a message and the current context, return a selected vector for the communication.
    def select_vector(self, message, context):
        raise NotImplementedError()

    # If necessary, update the policy's rules
    def update_policy(self,):
        raise NotImplementedError()
