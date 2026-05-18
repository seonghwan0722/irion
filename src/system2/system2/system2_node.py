# system2/system2/system2_node.py
import rclpy
from rclpy.node import Node
import json
import threading
from typing import Dict, Any, Optional

from std_msgs.msg import String
from system_msgs.msg import PlanCommand

from .llm_planner import build_plan
from .models import SystemState

class System2Node(Node):
    """
    System-2 High-Level Planner Node
    운용자의 명령을 받아 LLM을 통해 고수준 계획을 생성하고 System-1에 하달합니다.
    """
    def __init__(self):
        super().__init__('system2_node') # 시스템 동기화를 위한 구독(Subscription) 설정
        self.get_logger().info("System-2 Node Initializing...")

        self.latest_state: Optional[SystemState] = None

        # --- 구독자 및 퍼블리셔 ---
        self.state_sub = self.create_subscription(
            String,
            '/ats_state',
            self.state_callback,
            10,
        ) # System-1(로봇)이 발행하는 상태 정보 수신 (위치, 배터리, 현재 태스크, 진행 중인 미션 정보 등)
            # 이후 self.latest_state 변수에 캐싱되어 context-aware(상황인지)판단에 사용

        self.plan_cmd_pub = self.create_publisher( 
            PlanCommand,
            '/system2/plan_cmd',
            10,
        ) # HighLevelPlan을 System-1 Executor에 전달하는 메인 채널

        # original note mentioned report_decision, adding it for completeness
        self.decision_pub = self.create_publisher(
            String,
            '/system1/report_decision',
            10,
        ) # report_and_wait 상태일 때, 운영자의 승인/지시(Decision) 를 즉시 전달하는 별도 제어 채널

        self.report_sub = self.create_subscription(
            String, '/system2/report_context', self.report_context_callback, 10
        )

        # --- 키보드 입력 스레드 ---
        self._keyboard_thread = threading.Thread(
            target=self._operator_input_loop,
            daemon=True,
        ) # 별도의 데몬 스레드로 _operator_input_loop 실행 하여 spin 루프의 블로킹 문제를 방지
                # 로봇의 자율 동작은 계속 유지, 운영자는 콘솔로 언제든지 비동기적(Asynchronously) 으로 명령 입력 가능
        self._keyboard_thread.start()

    def state_callback(self, msg: String):
        """System-1의 상태를 업데이트합니다."""
        try:
            data = json.loads(msg.data)
            self.latest_state = SystemState(**data)
        except Exception:
            pass

    def report_context_callback(self, msg: String):
        """
        System1이 report_and_wait 액션에 의해 상황을 보내주는 경우.
        1) 현장을 로그로 요약 통보
        2) 터미널에서 운영자 자연어 명령을 input() 으로 입력 받음
        """
        self.get_logger().info(f"[Report 수신] {msg.data}")
        # Note: The original logic for interactive report_and_wait 
        # would typically block the input loop, but here we log it.
        # Implementation of interactive prompt here would require coordination with the main input loop.

    def _operator_input_loop(self):
        """터미널로부터 사용자 명령을 비동기적으로 입력받습니다."""
        while rclpy.ok():
            try:
                prompt = (
                    "\n[System2] 운용자 자연어 명령을 입력하세요.\n"
                    "> "
                )
                user_cmd = input(prompt)
                if not user_cmd.strip(): continue

                self.get_logger().info(f"명령 처리 중: {user_cmd}")
                self._process_command(user_cmd)
            except EOFError:
                break
            except Exception as e:
                self.get_logger().error(f"입력 루프 오류: {e}")

    def _process_command(self, user_cmd: str):
        """사용자 명령을 기반으로 계획을 생성하고 발행합니다."""
        try:
            # LLM이 만든 JSON이 스키마와 살짝 다를 때를 대비한 전처리 함수 (llm_planner handles it via Pydantic)
            plan = build_plan(user_cmd, self.latest_state)
            
            msg = PlanCommand()
            msg.mission_id = plan.mission_id
            msg.plan_json = plan.model_dump_json()
            
            self.plan_cmd_pub.publish(msg)
            self.get_logger().info(f"새로운 계획 발행 완료: {plan.mission_id}")
            
            # Decision topic update if needed
            # self.decision_pub.publish(String(data=json.dumps({"decision": "new_plan", "mission_id": plan.mission_id})))
            
        except Exception as e:
            self.get_logger().error(f"계획 생성 실패: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = System2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
