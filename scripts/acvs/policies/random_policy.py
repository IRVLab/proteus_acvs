from acvs.policies.communication_policy import CommunicationPolicy
import random

class RandomPolicy(CommunicationPolicy):
    def __init__(self, language, min_vectors=1, max_vectors=None):
        super().init(self, language)
        self.vectors = language.out_vectors()
        self.vector_range = [min_vectors, max_vectors]

        if min_vectors < 1: self.vector_range[0] = 1
        if max_vectors == None: self.vector_range[1] = self.vector_range[0] + 1
        if self.vector_range[0] > self.vector_range[1]: raise ValueError("min_vectors cannot be greater than max_vectors")
        if self.vector_range[1] == self.vector_range[0]: self.vector_range[1] = self.vector_range[0] + 1
        
        
    # Given a message and the current context, return a selected vector for the communication.
    def select_vector(self):
        n_vectors = random.randrange(*self.vector_range)
        random.sample(self.vector, n_vectors)

    # If necessary, update the policy's rules
    def update_policy(self, min_vectors=1, max_vectors=None):
        self.vector_range = [min_vectors, max_vectors]

        if min_vectors < 1: self.vector_range[0] = 1
        if max_vectors == None: self.vector_range[1] = self.vector_range[0] + 1
        if self.vector_range[0] > self.vector_range[1]: raise ValueError("min_vectors cannot be greater than max_vectors")
        if self.vector_range[1] == self.vector_range[0]: self.vector_range[1] = self.vector_range[0] + 1
