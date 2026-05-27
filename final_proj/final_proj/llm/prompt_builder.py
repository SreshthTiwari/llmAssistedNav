import json

class PromptBuilder:
    def __init__(self):
        pass

    def build_region_prompt(self, region_context, goal, skill_context=None):
        payload = {
            "task": "classify_uncertain_region",
            "region_context": region_context,
            "goal": goal,
            "skill_context": skill_context or []
        }
        return json.dumps(payload)

    def build_skill_query_prompt(self, situation_context):
        payload = {
            "task": "retrieve_similar_skill",
            "situation_context": situation_context
        }
        return json.dumps(payload)