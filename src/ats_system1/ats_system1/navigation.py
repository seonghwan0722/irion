# ats_system1/navigation.py
import rclpy
import time
import math
from typing import Dict, Any, Optional, Callable
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.node import Node

class Nav2Navigator: # Nav2 NavigateToPose 액션을 감싸는 전용 래퍼 -> 스레드 구조 때문에 폴링 기반 비동기 대기 사용
    """
    Nav2 NavigateToPose 액션을 감싸는 전용 래퍼입니다.
    """
    def __init__( # class Nav2Navigator의 초기화 함수
        self,
        node: Node, # Node 입력 -> ActionClient 생성
        feedback_cb: Optional[Callable[[Dict[str, Any]], None]] = None,
        action_name: str = "navigate_to_pose", # 액션 이름(NavigateToPose)과 기준 좌표계(frame)를 설정
        frame_id: str = "map", # 기본 frame은 map
    ):
        self._node = node
        self._feedback_cb = feedback_cb
        self._action_name = action_name
        self._frame_id = frame_id

        self._client = ActionClient(node, NavigateToPose, self._action_name)
        self._last_goal_handle = None

        self._node.get_logger().info(
            f"[Nav2Navigator] created for action '{self._action_name}' in frame '{self._frame_id}'"
        )

    def start(self, goal: Dict[str, Any]): # 좌표계 호출
        if not isinstance(goal, dict): # goal이 딕셔너리 형태인지 확인
            self._node.get_logger().error("[Nav2Navigator] goal 형식 오류 (dict 아님)")
            return None, None

        try:# goal의 데이터 로드
            x = float(goal.get("x", 0.0)) # goal["x"], goal["y"], goal["yaw"]를 모두 float()로 캐스팅
            y = float(goal.get("y", 0.0)) # Nav2에 사용될 좌표계를 실수화
            yaw = float(goal.get("yaw", 0.0)) 
        except Exception as e:
            self._node.get_logger().error(f"[Nav2Navigator] goal 파싱 실패: {e}")
            return None, None

        t0 = time.time()
        while not self._client.wait_for_server(timeout_sec=0.2): # 주기적으로 노드 종료 상태로 판단
            if not rclpy.ok():
                self._node.get_logger().error(
                    "[Nav2Navigator] rclpy 종료 상태에서 server wait"
                )
                return None, None

            if time.time() - t0 > 5.0: # 루프 시작 시각(t0) 기준으로 5초 이상 서버가 안 뜨면 연결 실패 로그 출력/ 추후, Executor 레벨에서 replan 발생
                self._node.get_logger().error(
                    f"[Nav2Navigator] action server '{self._action_name}' 연결 실패"
                )
                return None, None
        
        msg = NavigateToPose.Goal() # Nav2가 이해 가능한 목표 pose로 변환
        msg.pose = PoseStamped()
        msg.pose.header.stamp = self._node.get_clock().now().to_msg()
        msg.pose.header.frame_id = self._frame_id
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = self._yaw_to_quat(yaw) # yaw를 쿼터니언으로 변환

        send_goal_future = self._client.send_goal_async( # 비동기 goal 전송
            msg,
            feedback_callback=self._on_feedback,
        )

        while rclpy.ok() and not send_goal_future.done(): # 결과 future가 완료될 때 까지 대기
            time.sleep(0.01)

        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted: # future가 완료되면 goal이 수락 여부를 확인
            self._node.get_logger().warn("[Nav2Navigator] goal rejected")
            return None, None

        self._last_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()  #추가로 result_future를 받아 결과 풀링 준비
        self._node.get_logger().info("[Nav2Navigator] goal accepted")
        return goal_handle, result_future

    def cancel(self, goal_handle):
        if goal_handle:
            goal_handle.cancel_goal_async()

    def _on_feedback(self, feedback_msg):
        if self._feedback_cb:
            fb = {
                "distance_remaining": feedback_msg.feedback.distance_remaining,
                "stamp": time.time()
            }
            self._feedback_cb(fb)

    def _yaw_to_quat(self, yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q

def exec_move_to( # System-1 Executor에서 이동을 담당하는 실제 단위 액션 함수
    # Nav2 피드백을 모니터링
    # 정상 도착 시 -> True 반환/ 중간 장애·실패 감지 시 -> False 반환
    node: Node,     # 로그, TF, _last_pose 같은 현재 상태에 접근할 수 있는 핸들
    navigator: Nav2Navigator, # Nav2 래퍼 객체
    nav_feedback: Dict[str, Any], # 피드백 콜백이 값을 덮어써 가며 공유하는 딕셔너리
    goal: Dict[str, Any], # 플랜에서 넘어온 파라미터 -> '어디까지 이동할지(목표)'와 '얼마 동안 진행이 없으면 stuck으로 판단할지' 같은 정책을 포함
    replan_rules: Optional[Dict[str, Any]] = None,
) -> bool:

    if not isinstance(goal, dict): # goal이 제대로 들어왔는지 확인
        # goal을 잘못 넘기면 실패 반환 -> run_loop 에서 재시도 -> emit_replan으로 system 2 에 실패 트리거
        node.get_logger().error("[move_to] goal 파라미터 없음/형식 오류")
        return False

    goal_handle, result_future = navigator.start(goal)
    if not goal_handle or not result_future:
        node.get_logger().warn("[move_to] navigator.start 실패")
        return False

    rules = dict(replan_rules or {})
    hard_stuck = float(rules.get("hard_stuck_timeout_sec", 20.0)) # hard_stuck_timeout_sec : 진행이 없으면 goal 실패로 보는 최대 대기 시간
    grace_sec = float(rules.get("progress_grace_sec", 5.0)) # progress_grace_sec : 출발 직후 stuck 판정을 유예하는 시간
    eps_m = float(rules.get("progress_epsilon_m", 0.03)) # progress_epsilon_m : 진행으로 인정할 최소 거리 감소량(Delta distance)

    accept_t = time.time() # Nav2 goal이 수락된 시각
    last_prog = accept_t # 마지막으로 “유의미한 진행”이 감지된 시각
    best_dist = float("inf") # 지금까지 관찰한 distance_remaining 중 가장 작은 값 => 이 세 값을 계속 갱신하며, 현재 로봇의 움직임 상태를 판단
    last_print = 0.0

    while rclpy.ok():
        time.sleep(0.2)
        now = time.time()
        
        # Check if node status changed
        if hasattr(node, "queue_status") and node.queue_status != "running":
            navigator.cancel(goal_handle)
            return False

        dist = nav_feedback.get("distance_remaining")
        if now - last_print > 1.0: # 1초 마다 TF에서 가져온 현재 pose(x, y, yaw)와 Nav2 피드백의 남은 거리 값을 함께 로그로 출력
            # TF 조회 성공 : pose(map)=..., dist_remain=... 형태로 현재 위치 + 남은 거리를 계속 모니터링
            # TF 조회 실패 : pose=Unknown(TF fail)이라고 명시, 남은 거리 값 여부 확인
            dist_dbg = dist
            last_pose = getattr(node, "_last_pose", {})
            if last_pose.get("ok", False):
                node.get_logger().info(
                    "[move_to] pose(map)=({:.2f}, {:.2f}, yaw={:.2f}), dist_remain={}".format(
                        last_pose.get("x", 0.0),
                        last_pose.get("y", 0.0),
                        last_pose.get("yaw", 0.0),
                        dist_dbg if dist_dbg is not None else "None",
                    )
                )
            else:
                node.get_logger().info(
                    f"[move_to] pose=Unknown(TF fail), dist_remain={dist_dbg if dist_dbg is not None else 'None'}"
                )
            last_print = now

        if dist is not None:
            if dist < (best_dist - eps_m): # 값이 줄어드는 경우
                best_dist = dist
                last_prog = now # 진행이 있다 판단 -> best_dist와 last_prog 갱신

        if (now - accept_t) > grace_sec and (now - last_prog) > hard_stuck: # stuck으로 판단하는 검사
            # 정체(stuck)로 판단 시, Nav2 goal 즉시 cancel() 호출
            # False 반환 -> Executor에서 이 step을 실패로 처리
            # 이후 retry 할지 / System-2에 replan 요청할지 결정
            node.get_logger().warn(
                "[move_to] 진행 정체 감지 -> Nav2 goal cancel & 실패 반환"
            )
            try:
                navigator.cancel(goal_handle)
            except Exception as e:
                node.get_logger().warn(f"[move_to] navigator.cancel 실패: {e}")
            return False

        if result_future.done():
            try:
                res = result_future.result()
            except Exception as e:
                node.get_logger().warn(f"[move_to] result_future 예외: {e}")
                return False

            status = getattr(res, "status", None)
            ok = bool(status == 4)  # SUCCEEDED # Nav2 성공 여부 조건
            if ok: # 성공 : "Nav2 SUCCEEDED" 로그 + True 반환
                node.get_logger().info("[move_to] Nav2 SUCCEEDED")
            else: # 실패 : "Nav2 FAILED, status=..." 로그 + False 반환
                node.get_logger().warn(f"[move_to] Nav2 FAILED, status={status}")
            return ok # -> 루프가 도는 동안 rclpy.ok()가 False가 되면 노드 종료로 판단, False 반환
    return False
