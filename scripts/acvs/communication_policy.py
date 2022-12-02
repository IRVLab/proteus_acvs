class CommunicationPolicy(object):
    def __init__(self, language):
        self.language = language


    def test():
        raise NotImplementedError()


class StaticPolicy(CommunicationPolicy):
    def __init__(self, language, vector):
        super().init(self, language)
        self.static_vector = vector

class RandomPolicy(CommunicationPolicy):
    def __init__(self, language):
        super().init(self, language)

class HeuristicPolicy(CommunicationPolicy):
    def __init__(self, language):
        super().init(self, language)

class POMDPPolicy(CommunicationPolicy):
    def __init__(self, language):
        super().init(self, language)

class PersonalPolicy(CommunicationPolicy):
    def __init__(self, language):
        super().init(self, language)