# system1/system1/system1_executor_node.py
import rclpy
from rclpy.node import Node
import json
import threading
import time
import math
from typing import Dict, Any, List, Optional, Tuple, Callable

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from system_msgs.msg import PlanCommand
from jsonschema import Draft202012Validator

# ats_system1 유틸리티 모듈 임포트
from ats_system1.io.topics import create_publishers, create_subscriptions
from ats_system1.vision import VisionCache
from ats_system1.utils import make_state, emit_replan, eval_guard
from ats_system1.schema import HIGH_LEVEL_PLAN_SCHEMA
from ats_system1.navigation import Nav2Navigator, exec_move_to
from ats_system1.tracker import Tracker
from ats_system1.depth import DepthBuffer

class System1ExecutorNode(Node):
    """
    System-1 Executor (5 Unit Actions 전용)
    로봇의 즉각적이고 반응적인 동작을 관리하는 실행기 노드입니다.

    지원 Task:
        - move_to ...
        - scan ...
        - report ...
        - wait_for_command ...
        - track ...
        - return_to_home ...
    """
    def __init__(self):
        super().__init__("system1_executor_node")
        self.get_logger().info("System-1 Executor Node Initializing...")

        # 실행 중인 모든 단위 액션이 신뢰 가능한 최신 포즈를 참조할 수 있도록 유지
        # TF 체인 이상·시뮬레이터/실기 셋업 오류를 즉시 감지
        # -> Executor는 '자기 위치를 모르는 상태에서 행동하는 위험'을 최대한 조기에 감지하고 피하도록 설계된 실행기
        self._last_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0, "ok": True}

        # JSON 스키마 기반 플랜 검증
        # System-2로부터 온 필드 누락·오타·미정의 task를 HIGH_LEVEL_PLAN_SCHEMA로 1차 필터링
        # 잘못된 플랜은 실행 전 단계에서 즉시 차단 -> 안전성 & 예측 가능성 확보
        self.validator = Draft202012Validator(HIGH_LEVEL_PLAN_SCHEMA)

        # 단일 플랜 관리 (상위 계층(System-2 혹은 별도의 Mission Manager)으로부터 역할 및 책임 분리)
        self.current_plan: Optional[Dict[str, Any]] = None # 현재 실행 중인 전체 플랜(JSON)
        self.current_index = -1 # 몇 번째 step을 실행 중인지 가리키는 포인터

        # 상위 시스템의 세부 로그를 읽지 않고, self.queue_status로 상태 판단 가능
        self.queue_status = "idle"
        # idle – 새 플랜을 받을 준비 완료/ running – 현재 step 수행 중 / paused – guard 조건/환경 문제로 일시 중단 / done – 플랜 정상 완료
        
        # System-2는 각 플랜에는 고유한 mission_id가 부여
        # mission_id 를 통해 계층에서 "어떤 미션의 어떤 step에서 실패했는지" 를 추적
        self.mission_id = ""

        # eval_guard(): 시스템 플랜 실행 전 동작 변수에 대응하게 위한 관리 평가 함수
        self.ROE_OK = True # 사용자 규칙 충족 여부
        self.SAFE_BACKSTOP = True # 안전구역 확보 여부
        self.BATTERY_SOC = 1.0 # 배터리 잔량
        self.MAX_SPEED = 0.5 # 동작 속도 제한
        # Guard 조건이 False인 경우, 해당 step 실행 중단, 상태를 paused/errorfh wjsghks
        # 필요시 상위 시스템에 재계획 요청

        # 파라미터로 분리된 환경 의존 값 (launch파일 or YAML 파일)
        # Executor의 이식성과 확장성 향상
        # 환경이나 base 위치 변경에 의한 재빌드 필요 X
        # 환경 의존 값을 코드와 분리하는 설계
        # 어떤 로봇 플랫폼에서도 그대로 재활용할 수 있는 핵심 설계
        self.declare_parameter("frame_w_default", 1280)
        self.declare_parameter("frame_h_default", 720)

        # ----통신 구조 ----
        # 별도 I/O 관리 모듈(ats_system1.io.topics)로 분리하여 정의
        # 토픽 설정에 종속되지 않는 독립적 실행 엔진으로 만드는 설계 요소
        # 예) cmd_vel -> /ats_vel: 전체 코드에서 /ats_vel로 바꿀 필요 X
        # 네트워크/토폴로지가 자주 바뀌는 로봇 환경에서 유리한 설계
        self.pubs = create_publishers(self)
        self.pub_ats = self.pubs["ats"]
        self.pub_replan = self.pubs["replan"]
        self.pub_cmd = self.pubs["cmd_vel"]
        self.pub_gimbal = self.pubs["gimbal"]

        create_subscriptions(self, self.on_plan_cmd, self.on_vision_raw)
        
        # --- 비전 ------
        # 표준화된 비전 상태 저장소
        # 단순 버퍼가 아니라 현재 프레임에서 감지된 객체 상태를 정규화해 저장하는 중앙 저장소
        self.vision = VisionCache(1280, 720)
        
        self.depth = DepthBuffer(self)
        
        #----추적(Tracking)과 이동(Navigation) -----
        # track -> Tracker 객체에 위임 / move_to -> Nav2Navigator 에 위임
        # 위임 이유 1 : Executor는 언제 시작할지, 성공/실패를 어떻게 판단할지, 실패 시 재시도/재플랜 시점만 결정
        # 위임 이유 2 : 기능 업데이트가 발생해도 플랜 실행 로직은 수정 필요 X
        self.navigator = Nav2Navigator(self, self._on_nav_feedback)
        self._nav_feedback = {"distance_remaining": None, "stamp": time.time()}
        
        self.tracker = Tracker(self, self.pub_cmd, self.pub_gimbal, self.vision, self.depth)

        # --- 동시성과 가시성(visibility) ----
        self.lock = threading.Lock()
        # Executor는 다중 스레드·콜백이 동시에 동작
        # 동일 자원(current_plan, current_index, vision)에 동시 접근 -> 무결성 문제 발생
        # 해결 방안 : 공유 자원 접근 시 Lock 사용
        self.create_timer(0.5, self.publish_state)
        # publish_state : System-2와의 연결 고리
        # 0.5초마다 실행되어, Executor의 전체 상태를 외부로 브로드캐스트
        # -> 1) System-2, UI 모니터링 툴이 로봇의 내부 상태를 실시간으로 시각화 가능
        # -> 2) System-1 <-> System-2 간 상태 동기화 유지의 핵심 매커니즘

    def _on_nav_feedback(self, feedback):
        self._nav_feedback.update(feedback)

    def on_vision_raw(self, msg: String):
        """원시 비전 입력의 진입점"""
        # 비전 모듈 설계 핵심 의도
        # 모델 독립성(Model Independence) 유지
        # vision 데이터 일관성 및 해석 가능성 보장
        # 정리 : normalize_raw_vision & on_vision_raw
        # 두 함수 덕분에 Executor가 특정 비전 알고리즘에 종속되지 않고 범용 실행 엔진으로 유지
        self.vision.update_from_msg(msg.data, self.get_logger())

    def on_plan_cmd(self, msg: PlanCommand):
        """입력된 고수준 플랜의 검증 절차를 실행하는 함수"""
        # 검증 원칙 1. 플랜의 신뢰성을 코드 수준에서 강제
        # msg.plan_json으로 JSON 문자열 전달 후 json.loads를 통해 파싱
        # 원칙 의미 : "LLM이 생성해도, 규격을 만족하지 않으면 실행하지 않는다" -> 물리 시스템의 첫 번째 안전장치
        try:
            plan = json.loads(msg.plan_json)
        except Exception as e:
            self.get_logger().error(f"Plan JSON 파싱 실패: {e}")
            return

        # 검증 원칙 2. cancel을 최우선 인터럽트 취급
        # 플랜이 dict, intent 가 "cancel" 이면 중단으로 해석
        # 모든 액션을 중단하기 위해 _hard_stop() 실행
        # 원칙 의미 : 상위 시스템이 중단을 요청할 때 즉각 반응
        if plan.get("intent") == "cancel":
            self.get_logger().info("미션 중단 명령 수신")
            self._hard_stop()
            self.queue_status = "idle"
            self.publish_state()
            return

        # self.validator.iter_errors(plan)로 검증 수행
        errs = list(self.validator.iter_errors(plan))
        if errs:
            self.get_logger().error(f"플랜 검증 실패: {errs[0].message}")
            emit_replan(self, f"Schema validation failed: {errs[0].message}", plan.get("mission_id", ""))
            return

        # 검증 원칙 3. 실행과 수신을 분리하는 비동기 구조
        # 스키마 검증으로 플랜이 유효하면 with self.lock을 통해 파라미터 저장
        # threading.Thread로 self._run_loop를 별도 스레드 처리
        # 원칙 의미 : 콜백은 가볍게하고 실행은 전용 루프
        with self.lock:
            self.current_plan = plan
            self.mission_id = plan.get("mission_id", "")
            self.current_index = 0
            self.queue_status = "running"
            self.get_logger().info(f"새로운 미션 시작: {self.mission_id}")

        threading.Thread(target=self._run_loop, daemon=True).start()

    def _hard_stop(self):
        """모든 동작을 즉시 중단합니다."""
        with self.lock:
            self.queue_status = "idle"
            self.current_plan = None
            self.current_index = -1
        stop_msg = Twist()
        self.pub_cmd.publish(stop_msg)
        self.pub_gimbal.publish(stop_msg)

    def state_string(self) -> str: # Executor의 상태를 “한 줄” 문자열로 만들기 위한 함수
        if self.queue_status == "running" and self.current_plan:
            steps = self.current_plan.get("steps", [])
            if 0 <= self.current_index < len(steps):
                return steps[self.current_index].get("task", "unknown")
        return self.queue_status

    def publish_state(self):
        """현재 항목을 하나의 상태 메시지로 통합해 발송"""
        vision_snapshot = self.vision.snapshot()
        msg = make_state(
            self, self._last_pose, self.mission_id,
            self.state_string(), self.queue_status,
            self.current_plan, self.current_index,
            self.ROE_OK, self.SAFE_BACKSTOP, self.BATTERY_SOC, self.MAX_SPEED,
            vision_snapshot
        )
        self.pub_ats.publish(msg)

    def _run_loop(self):
        """검증된 플랜을 실행하는 함수 (실행 원칙: 현재 상태를 기준으로 "다시 판단"하면서 진행)"""
        while rclpy.ok():
            with self.lock:
                plan = self.current_plan
                idx = self.current_index
                status = self.queue_status

            if plan is None or status != "running":
                break

            steps = plan.get("steps", [])
            if idx >= len(steps):
                self.get_logger().info("모든 미션 스텝 완료")
                self.queue_status = "done"
                break

            # 각 Step의 4요소 : System-2와 System-1 사이의 최소 실행 단위 계약
            step = steps[idx]
            task = step.get("task")
            params = step.get("params", {})
            guard = step.get("guard", "")
            
            # 플랜 실행 설계 보장 사항: 안전 조건을 만족할 때까진 해당 step을 절대 실행하지 않는다
            if guard:
                syms = {
                    "ROE_OK": self.ROE_OK,
                    "SAFE_BACKSTOP": self.SAFE_BACKSTOP,
                    "BATTERY_SOC": self.BATTERY_SOC
                }
                if not eval_guard(guard, syms):
                    self.get_logger().warn(f"Guard 조건 미충족: {guard}. 일시 정지합니다.")
                    self.queue_status = "paused"
                    break

            # task 분기 : 모든 단위 액션이 "동기 함수 호출 + True/False 반환" -> `_run_loop`는 플로우 제어에 집중
            # 설계 장점 1 : 단위 액션이 늘어나도 _run_loop 로직은 고정
            # 설계 장점 2 : 액션 내부 구현 변경·교체에도 True/False 계약만 맞으면 그대로 재사용
            # 설계 장점 3 : 실패·재시도·재플랜 처리가 모든 액션에 대해 일관된 방식으로 동작
            self.get_logger().info(f"Step {idx} 실행 중: {task}")
            ok = self._execute_task(task, params, plan)

            if ok:
                with self.lock:
                    self.current_index += 1
            else:
                retry = step.get("retry", 0)
                if retry > 0:
                    step["retry"] -= 1
                    self.get_logger().info(f"태스크 실패. 재시도 중 (남은 횟수: {step['retry']})")
                    time.sleep(1.0)
                else:
                    # retry 횟수 소진 : `emit_replan(...)` 호출 후 상위 시스템에 진행 중단 로그 발송(`self.queue_status = "error"`)
                    self.get_logger().error(f"태스크 '{task}' 최종 실패. 재플랜을 요청합니다.")
                    self.queue_status = "error"
                    emit_replan(self, f"Task {task} failed", self.mission_id)
                    break

    def _execute_task(self, task: str, params: Dict[str, Any], plan: Dict[str, Any]) -> bool:
        """개별 태스크를 실행합니다."""
        if task == "move_to":
            # Executor는 move_to()의 True / False 결과로 스텝 진행 / 재시도 / System-2 replan 여부를 결정
            return exec_move_to(self, self.navigator, self._nav_feedback, params.get("goal", {}), plan.get("replan_rules", {}))
        elif task == "scan":
            return self._do_scan(params)
        elif task == "track":
            # Detection-Tracking 이 아닌 통합된 3축 통합 제어(gimbal + yaw + distance)
            # System1은 “트래킹 제어의 성공 여부”만 판단, 세부 제어는 Tracker 내부 처리
            return self._do_track(params, plan)
        elif task == "report_and_wait":
            return self._do_report_and_wait(params)
        elif task == "return_to_home":
            return self._do_return_to_home(params)
        else:
            self.get_logger().error(f"지원하지 않는 태스크: {task}")
            return False

    def _do_scan(self, params) -> bool:
        # scan 액션은 System-2를 호출하지 않고, /scan_report 토픽에 Found 이벤트만 발행
        # scan노드는 "탐색 + found 이벤트 발행"에만 집중
        # 행동(추종, 보고, 무시 등)은 상위 로직이 자유롭게 결정
        self.get_logger().info("[Scan] 정찰 동작 수행 중...")
        time.sleep(3.0) 
        return True

    def _do_track(self, params, plan) -> bool:
        self.tracker.start(params, plan.get("replan_rules", {}))
        ok = self.tracker.wait_initial_success(timeout=10.0)
        return ok

    def _do_report_and_wait(self, params) -> bool:
        # 실제로는 여기서 System-2의 결정을 기다리는 로직이 들어갑니다 (exec_report_and_wait)
        self.get_logger().info("[Report] 상황 보고 및 대기 중...")
        time.sleep(2.0)
        return True

    def _do_return_to_home(self, params) -> bool:
        self.get_logger().info("[Return] 홈으로 복귀 중...")
        return exec_move_to(self, self.navigator, self._nav_feedback, {"x": 0.0, "y": 0.0, "yaw": 0.0}, {})

def main(args=None):
    rclpy.init(args=args)
    node = System1ExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
