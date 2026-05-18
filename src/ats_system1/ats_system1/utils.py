# ats_system1/utils.py
import json
from typing import Dict, Any, Optional

def make_state(
    node,
    pose: Dict[str, Any],
    mission_id: str,
    state_str: str,
    queue_status: str,
    current_plan: Optional[Dict[str, Any]],
    current_index: int,
    roe_ok: bool,
    safe_backstop: bool,
    battery_soc: float,
    max_speed: float,
    vision_snapshot: Dict[str, Any]
) -> Any:
    """
    Executor의 현재 상태를 JSON 문자열로 변환하여 String 메시지로 반환합니다.
    """
    from std_msgs.msg import String
    state = {
        "mission_id": mission_id,
        "state_string": state_str,
        "queue_status": queue_status,
        "current_index": current_index,
        "pose": pose,
        "roe_ok": roe_ok,
        "safe_backstop": safe_backstop,
        "battery_soc": battery_soc,
        "max_speed": max_speed,
        "vision_snapshot": vision_snapshot
    }
    msg = String()
    msg.data = json.dumps(state, ensure_ascii=False)
    return msg

def emit_replan(node, reason: str, mission_id: str):
    """
    재플랜 요청을 발행합니다.
    """
    from std_msgs.msg import String
    msg = String()
    msg.data = json.dumps({"reason": reason, "mission_id": mission_id}, ensure_ascii=False)
    if hasattr(node, "pub_replan"):
        node.pub_replan.publish(msg)
    node.get_logger().info(f"[emit_replan] reason: {reason}")

def eval_guard(guard: str, symbols: Dict[str, Any]) -> bool:
    """
    Guard 조건을 평가합니다.
    """
    try:
        # 매우 단순화된 평가 (실제로는 safer eval이나 전용 파서 필요)
        return eval(guard, {"__builtins__": {}}, symbols)
    except Exception:
        return False
