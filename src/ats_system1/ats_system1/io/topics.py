# ats_system1/io/topics.py
from typing import Dict, Any, Callable
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from system_msgs.msg import PlanCommand

def create_publishers(node: Node) -> Dict[str, Any]:
    """
    Executor에 필요한 모든 Publisher를 생성하여 딕셔너리로 반환합니다.
    - ats: 시스템 전체 상태 발행 (/ats_state)
    - replan: 상위 시스템(System-2)에 재계획 요청 발행 (/system2/replan_request)
    - cmd_vel: 로봇 바디 이동 속도 명령 발행 (/cmd_vel)
    - gimbal: 짐벌 제어 명령 발행 (/ats_twist)
    """
    pubs = {}
    pubs["ats"] = node.create_publisher(String, "/ats_state", 10)
    pubs["replan"] = node.create_publisher(String, "/system2/replan_request", 10)
    pubs["cmd_vel"] = node.create_publisher(Twist, "/cmd_vel", 10)
    pubs["gimbal"] = node.create_publisher(Twist, "/ats_twist", 10)
    
    # 별도 I/O 관리 모듈로 분리하여 정의
    # 토픽 설정에 종속되지 않는 독립적 실행 엔진으로 만드는 설계 요소
    # 네트워크/토폴로지가 자주 바뀌는 로봇 환경에서 유리한 설계
    return pubs

def create_subscriptions(node: Node, plan_cb: Callable, vision_cb: Callable):
    """
    Executor에 필요한 주요 Subscription을 생성합니다.
    - plan_cmd: System-2로부터 고수준 플랜 수신
    - vision_context: 정규화된 비전 정보 수신
    """
    node.create_subscription(PlanCommand, "/system2/plan_cmd", plan_cb, 10)
    node.create_subscription(String, "/vision_context", vision_cb, 10)
    
    # 비정형 JSON 스트림도 선택적으로 수용할 수 있는 유연한 구조
