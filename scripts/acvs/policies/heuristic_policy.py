from acvs.policies.communication_policy import CommunicationPolicy
import operator
import yaml
import rospy
import random

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

    def get_combination_rules(self, selected):
        final_dict = {}
        for vec in selected:
            k = vec.id
            combo_weights = self.combination_config['rules'][k]
            for vk, weight in combo_weights.items():
                if vk not in final_dict:
                    final_dict[vk] = weight
                else:
                    final_dict[vk] = final_dict[vk] * weight

        return final_dict

    # Given a message and the current context, return a selected vector for the communication.
    def select_vector(self, goal, interactant_context):
        if interactant_context is not None:
            pd, pa = interactant_context.get_pseudopose()
            prio = goal.priority
            content_tags = self.symbols[goal.symbol].content_tags if not goal.dynamic else ["dynamic"]
            rospy.loginfo(f"Selecting a vector using heuristics for {interactant_context.id} with pd={pd}, pa={pa}, prio={prio}, content={content_tags}")

            distance_weights = self.classify_distance(pd)
            angle_weights = self.classify_angle(pa)
            priority_weights = self.classify_priority(prio)
            content_weights = self.classify_content(content_tags)

            selection_weights = {}
            for id in self.vectors.keys():
                selection_weights[id] = distance_weights[id] * angle_weights[id] * priority_weights[id] * content_weights[id]

            rospy.loginfo(f"Using {selection_weights}")

            criteria = {'distance':distance_weights, 'angle':angle_weights, 'priority':priority_weights, 'content':content_weights, 'selection_weights':selection_weights}

            rospy.loginfo("Weights calculated, selecting now")
            selected_vectors = []
            # Now, we select based on our selection config (usually going to be max)
            max_vector = max(selection_weights, key=selection_weights.get)
            selected_vectors.append(self.vectors[max_vector])

            # Now, if combination is enabled, we add combination vectors.
            if self.combination_config['enabled']:
                if len(selected_vectors) <= self.combination_config['max_vectors']:
                    # We start by selecting calculating how close every other vector is to the selected vector. (The percentage of their weights)
                    combo_chances = {}
                    combo_rules = self.get_combination_rules(selected_vectors)
                    for k, weight in selection_weights.items():
                        if k != max_vector:
                            combo_chances[k] = selection_weights[k]/selection_weights[max_vector] # Get percentage of selected item.

                            # Next, we set any vectors with a nonzero weight that aren't at least 25% the weight of the selected to 0.25.
                            
                            combo_chances[k] += combo_rules[k] # Next, we add likelihood modifiers from the combination rules. 

                            # Any likelihood great than 8.0 gets set to 8.0, anything under 0.0 gets dropped. 
                            if combo_chances[k] >= self.combination_config['max_prob']:
                                combo_chances[k] = self.combination_config['max_prob']
                            elif combo_chances[k] < 0.0:
                                combo_chances.pop(k)

                    
                    rospy.loginfo(f"Running a random combination vector selection with the combination chances {combo_chances}")
                    
                    # Now we do a random selection based on these likelihoods. We sort the combo weights to let the higher chance vectors
                    # Get their chance to fill the spots first.
                    for k, chance in sorted(combo_chances.items(), key=operator.itemgetter(1),reverse=True):
                        if random.random() < chance and len(selected_vectors) < self.combination_config['max_vectors']:
                            selected_vectors.append(self.vectors[k])

                    criteria['combination_weights']=combo_chances

            return [selected_vectors, str(criteria)]

        else:
            rospy.loginfo(f"We have no known interactant, so we're going with our configured default.")
            return [[self.vectors[self.selection_config['default']]], 'default']


    # If necessary, update the policy's rules
    def update_policy(self,):
        raise NotImplementedError()
