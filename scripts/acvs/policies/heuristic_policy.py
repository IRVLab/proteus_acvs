from acvs.policies.communication_policy import CommunicationPolicy
import yaml
import rospy

class HeuristicPolicy(CommunicationPolicy):
    def __init__(self, lang_tup, fp):
        super().__init__(lang_tup)
        self.vectors = lang_tup[1]

        with open(fp, 'r') as file:
            heuristic_config = yaml.safe_load(file)
            self.selection_config = heuristic_config['heuristic_config']['selection_config']
            self.context_config = heuristic_config['heuristic_config']['context_config']
            self.combination_config = heuristic_config['heuristic_config']['combination_config']


    def classify_distance(self, pd):
        ranges = self.context_config['distance']
        dist_class = ''
        for key, dr in ranges.items():
            if pd >= dr[0] and pd <= dr[1]:
                dist_class = key
                break
        return self.context_config['rules']['distance'][dist_class]

    def classify_angle(self, pa):
        ranges = self.context_config['angle']
        ang_class = ''
        for key, ar in ranges.items():
            if pa >= ar[0] and pa <= ar[1]:
                ang_class = key
                break
        return self.context_config['rules']['angle'][ang_class]

    def classify_priority(self, prio):
        ranges = self.context_config['priority']
        prio_class = ''
        for key, pr in ranges.items():
            if prio >= pr[0] and prio <= pr[1]:
                prio_class = key
                break
        return self.context_config['rules']['priority'][prio_class]

    def classify_content(self, tags):
        contents = self.context_config['content']
        wdicts = []
        for tag in tags:
            if tag in contents:
                wdicts.append(self.context_config['rules']['content'][tag])

        final_dict = {k:1.0 for k in wdicts[0].keys()}
        for wd in wdicts:
            for k, w in wd.items():
                final_dict[k] = final_dict[k] * w

        return final_dict


    # Given a message and the current context, return a selected vector for the communication.
    def select_vector(self, goal, interactant_context):
        if interactant_context is not None:
            pd, pa = interactant_context.get_pseudopose()
            prio = goal.priority
            content_tags = self.symbols[goal.symbol].content_tags if not goal.dynamic else ["dynamic"]
            rospy.loginfo(f"Selecting a vector using heuristics for pd={pd}, pa={pa}, prio={prio}, content={content_tags}")

            distance_weights = self.classify_distance(pd)
            angle_weights = self.classify_angle(pa)
            priority_weights = self.classify_priority(prio)
            content_weights = self.classify_content(content_tags)

            selection_weights = {}
            for id in self.vectors.keys():
                selection_weights[id] = distance_weights[id] * angle_weights[id] * priority_weights[id] * content_weights[id]

            rospy.loginfo(selection_weights)

            rospy.loginfo("Weights calculated, selecting now")
            selected_vectors = []
            # Now, we select based on our selection config (usually going to be max)
            max_vector = max(selection_weights, key=selection_weights.get)
            selected_vectors.append(self.vectors[max_vector])

            # Now, if combination is enabled, we add combination vectors.
            if self.combination_config['enabled']:
                pass

            return selected_vectors
        else:
            rospy.loginfo(f"We have no known interactant, so we're going with our configured default.")
            return [self.vectors[self.selection_config['default']]]


    # If necessary, update the policy's rules
    def update_policy(self,):
        raise NotImplementedError()
