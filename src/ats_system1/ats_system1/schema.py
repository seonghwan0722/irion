# ats_system1/schema.py

# HighLevelPlan에 대한 JSON 스키마 정의
HIGH_LEVEL_PLAN_SCHEMA = {
    "type": "object",
    "properties": {
        "version": {"type": "string"},
        "mission_id": {"type": "string"},
        "intent": {"type": "string"},
        "constraints": {"type": "array", "items": {"type": "string"}},
        "steps": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "task": {
                        "type": "string",
                        "enum": ["move_to", "scan", "track", "report_and_wait", "return_to_home"]
                    },
                    "params": {"type": "object"},
                    "guard": {"type": "string"},
                    "retry": {"type": "integer", "minimum": 0}
                },
                "required": ["task"]
            }
        },
        "replan_rules": {
            "type": "object",
            "properties": {
                "lost_target_sec": {"type": "number"},
                "battery_rtb": {"type": "number"},
                "hard_stuck_timeout_sec": {"type": "number"}
            }
        }
    },
    "required": ["mission_id", "steps"]
}
