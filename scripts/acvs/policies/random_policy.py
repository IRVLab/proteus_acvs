from acvs.policies.communication_policy import CommunicationPolicy
import random

class RandomPolicy(CommunicationPolicy):
    def __init__(self, lang_tup, min_vectors=1, max_vectors=None):
        super().__init__(lang_tup)
        self.vectors = lang_tup[1]
        self.vector_range = [min_vectors, max_vectors]

        if min_vectors < 1: self.vector_range[0] = 1
        if max_vectors == None: self.vector_range[1] = self.vector_range[0] + 1
        if self.vector_range[0] > self.vector_range[1]: raise ValueError("min_vectors cannot be greater than max_vectors")
        if self.vector_range[1] == self.vector_range[0]: self.vector_range[1] = self.vector_range[0] + 1
        
    # Given a message and the current context, return a selected vector for the communication.
    def select_vector(self, goal):
        if goal.dynamic: 
            n_vectors = random.randrange(*self.vector_range)
            dyn_vectors = [vec for vec in self.vectors.values() if vec.has_dynamic]
            return random.sample(dyn_vectors, n_vectors)
        else:
            n_vectors = random.randrange(*self.vector_range)
            static_vectors = [vec for vec in self.vectors.values() if vec.has_static]
            return random.sample(static_vectors, n_vectors)

    # If necessary, update the policy's rules
    def update_policy(self, min_vectors=1, max_vectors=None):
        self.vector_range = [min_vectors, max_vectors]

        if min_vectors < 1: self.vector_range[0] = 1
        if max_vectors == None: self.vector_range[1] = self.vector_range[0] + 1
        if self.vector_range[0] > self.vector_range[1]: raise ValueError("min_vectors cannot be greater than max_vectors")
        if self.vector_range[1] == self.vector_range[0]: self.vector_range[1] = self.vector_range[0] + 1
