import json

class ResponseParser:
    def __init__(self):
        pass

    def parse_region_response(self, response_text):
        data = json.loads(response_text)
        label = data.get("label", "uncertain")
        confidence = float(data.get("confidence", 0.0))
        return {"label": label, "confidence": confidence}

    def parse_skill_response(self, response_text):
        data = json.loads(response_text)
        skill_id = data.get("skill_id", None)
        score = float(data.get("score", 0.0))
        return {"skill_id": skill_id, "score": score}