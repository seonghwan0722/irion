class System1ExecutorNode(Node):
    """
    System-1 Executor (5 Unit Actions 전용)

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

        # --------- TF ---------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self) # 다른 노드에 의존하지 않아도 바로 최신 Pose 사용 가능
        self._last_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0, "ok": False} # ok : 현재 TF 값이 신뢰 가능한 상태인지 여부까지 판단
        self.create_timer(1.0, self._log_tf_pose)
        # 1초마다 self._log_tf_pose ( map -> body ) 변환을 조회
            # 변환 성공 -> _last_pose 갱신
            # 변환 실패 -> _last_pose["ok"] = False 설정 + 경고 출력

        # 실행 중인 모든 단위 액션이 신뢰 가능한 최신 포즈를 참조할 수 있도록 유지
        # TF 체인 이상·시뮬레이터/실기 셋업 오류를 즉시 감지
            # -> Executor는 '자기 위치를 모르는 상태에서 행동하는 위험'을 최대한 조기에 감지하고 피하도록 설계된 실행기

        self.validator = Draft202012Validator(HIGH_LEVEL_PLAN_SCHEMA) 
        # JSON 스키마 기반 플랜 검증
            # System-2로부터 온 필드 누락·오타·미정의 task를 HIGH_LEVEL_PLAN_SCHEMA로 1차 필터링
            # 잘못된 플랜은 실행 전 단계에서 즉시 차단 -> 안전성 & 예측 가능성 확보
        # 단일 플랜 관리 (상위 계층(System-2 혹은 별도의 Mission Manager)으로부터 역할 및 책임 분리)
        self.current_plan: Optional[Dict[str, Any]] = None # 현재 실행 중인 전체 플랜(JSON)
        self.current_index = -1 # 몇 번째 step을 실행 중인지 가리키는 포인터

        # 상위 시스템의 세부 로그를 읽지 않고, self.queue_status로 상태 판단 가능
        self.queue_status = "idle"
        # idle – 새 플랜을 받을 준비 완료/ running – 현재 step 수행 중 / paused – guard 조건/환경 문제로 일시 중단 / done – 플랜 정상 완료
        self.mission_id = "" # 수행중인 미션 추적용
        # System-2는 각 플랜에는 고유한 mission_id가 부여
        # mission_id 를 통해 계층에서 "어떤 미션의 어떤 step에서 실패했는지" 를 추적

        # eval_guard(): 시스템 플랜 실행 전 동작 변수에 대응하게 위한 관리 평가 함수
        self.ROU_OK = False # 사용자 규칙 충족 여부
        self.SAFE_BACKSTOP = True # 안전구역 확보 여부
        self.BATTERY_SOC = 0.5 # 배터리 잔량
        self.MAX_SPEED = 0.5 # 동작 속도 제한
        # Guard 조건이 False인 경우, 해당 step 실행 중단, 상태를 paused/errorfh wjsghks
        # 필요시 상위 시스템에 재계획 요청

        # 파라미터로 분리된 환경 의존 값 (launch파일 or YAML 파일)
            # Executor의 이식성과 확장성 향상
            # 환경이나 base 위치 변경에 의한 재빌드 필요 X
            # 환경 의존 값을 코드와 분리하는 설계
                # 어떤 로봇 플랫폼에서도 그대로 재활용할 수 있는 핵심 설계
        self.declare_parameter("frame_w_default", 1280) # 카메라 기본 해상도, Home 좌표를 파라미터 형태로 외부로부터 입력
        self.declare_parameter("frame_h_default", 720)
        # ...
        self.declare_parameter("home_x", 0.0)
        # ...
        self._home_pose = {...}

# ----통신 구조 ----
        pubs = create_publishers(self) 
        # 별도 I/O 관리 모듈(ats_system1.io.topics)로 분리하여 정의
        # 토픽 설정에 종속되지 않는 독립적 실행 엔진으로 만드는 설계 요소
            # 예) cmd_vel -> /ats_vel: 전체 코드에서 /ats_vel로 바꿀 필요 X
        # 네트워크/토폴로지가 자주 바뀌는 로봇 환경에서 유리한 설계
        self.pub_ats = pubs["ats"]
        self.pub_replan = pubs["replan"]
        self.pub_cmd = pubs["cmd_vel"]
        self.pub_gimbal = pubs["gimbal"]

        create_subscriptions(self, self.on_plan_cmd, self.on_vision) # 플랜 명령 & 비전 정보 등록
        self.create_subscription(String, "/vision_context_raw", self.on_vision_raw, 10)
        # 비정형 JSON 스트림(/vision_context_raw)도 선택적으로 수용
        # self.on_plan_cmd : System-2 플랜 입력
        # self.on_vision : 정규화된 비전 정보
            # self.on_plan_cmd 와 self.on_vision 은 동일한 비전 캐시 객체(VisionCache)로 통합



# --- 비전 ------
self.vision = VisionCache(self._frame_w_default, self._frame_h_default) # 표준화된 비전 상태 저장소
    # 단순 버퍼가 아니라 현재 프레임에서 감지된 객체 상태를 정규화해 저장하는 중앙 저장소

def _normalize_raw_vision(self, raw): # 다양한 비전 출력 포맷을 하나의 표준 구조로 통합하는 핵심 함수
    frame_w = self._frame_w_default   # Executor 내부 모든 액션에서 일관된 구조로 비전 정보를 접근 가능
    frame_h = self._frame_h_default

    if isinstance(raw, dict) and "objects" in raw:
        try:
            frame_w = int(raw.get("frame_w", frame_w))
            frame_h = int(raw.get("frame_h", frame_h))
        except Exception:
            pass
        src_list = raw.get("objects", [])
    else:
        src_list = raw if isinstance(raw, list) else []

    targets = [] # 감지된 모든 객체 리스트
    primary_id = None # 리스트에서 가장 먼저 감지된 주요 객체 (track/report 단계의 "주 대상"으로 사용)

    for obj in (src_list or []):
        tid = obj.get("id")
        cls = obj.get("class") or obj.get("class_name") or "object"

        cx = cy = None
        if isinstance(obj.get("center"), dict):
            cx = obj["center"].get("x")
            cy = obj["center"].get("y")

        rng = None
        if obj.get("range_m") is not None:
            try:
                rng = float(obj.get("range_m"))
            except Exception:
                rng = None

        bbox = None
        if isinstance(obj.get("bbox"), dict):
            w = obj["bbox"].get("w")
            h = obj["bbox"].get("h")
            if (
                w is not None
                and h is not None
                and cx is not None
                and cy is not None
            ):
                x = cx - w / 2.0
                y = cy - h / 2.0
                bbox = [x, y, w, h]

        tgt = {
            "id": tid,
            "class": cls,
            "bbox": bbox,
            "range_m": rng,
            "center": {"x": cx, "y": cy}
            if (cx is not None and cy is not None)
            else None,
        }
        targets.append(tgt)
        if primary_id is None and tid is not None:
            primary_id = tid

    lost_sec = 0.0 if targets else 999.0  
    return {    # 이번 프레임에서 아무 객체도 감지되지 않은 누적 시간
        "targets": targets, # 값이 커지면 "타겟 손실"로 판단 -> 다음 액션 전환 트리거
        "primary_id": primary_id,
        "lost_sec": lost_sec,
        "frame_w": frame_w,
        "frame_h": frame_h,
    }   # -> 정규화된 데이터는 VisionCache에 저장

    def on_vision_raw(self, msg: String): # 원시 비전 입력의 진입점
    # 1. /vision_context_raw 토픽으로 들어온 원시(raw) JSON 데이터를 수신
    # 2. 원시(raw) JSON 파싱 후 _normalize_raw_vision() 로 데이터 표준화
    # 3. 정규화된 결과를 VisionCache에 반영

    try:
        raw = json.loads(msg.data) if msg and msg.data else {}
        norm = self._normalize_raw_vision(raw)
        self.vision.update_from_msg(json.dumps(norm), self.get_logger())
    except Exception as e:
        self.get_logger().warn(f"[vision/raw] parse failed: {e}")

    # 비전 모듈 설계 핵심 의도
        # 모델 독립성(Model Independence) 유지
            # 예) 비전 모델이 YOLOv8 에서 YOLOv10으로 교체 -> _normalize_raw_vision() 만 수정 ok
        # vision 데이터 일관성 및 해석 가능성 보장
            # 만약 비전 데이터가 불안정하면 System-2의 계획(Plan) 품질도 저하
    # 정리 : normalize_raw_vision & on_vision_raw
        # 두 함수 덕분에 Executor가 특정 비전 알고리즘에 종속되지 않고 범용 실행 엔진으로 유지
        # 비전 알고리즘 교체에도 Executor 영향 X


#----추적(Tracking)과 이동(Navigation) -----
# track -> Tracker 객체에 위임 / move_to -> Nav2Navigator 에 위임
    # 위임 이유 1 : Executor는 언제 시작할지, 성공/실패를 어떻게 판단할지, 실패 시 재시도/재플랜 시점만 결정
    # 위임 이유 2 : 기능 업데이트가 발생해도 플랜 실행 로직은 수정 필요 X
self.depth = DepthBuffer(self, topic="/depth") # 깊이 센서 데이터를 실시간으로 관리
    # Executor는 "현재 타겟까지 거리"를 즉시 조회 가능
    # 별도 TF 변환·센서 구독 로직 없이 거리 정보만 단순·안정적으로 제공
self.tracker = Tracker( # 카메라 중심 기준 타겟 위치 추적
    self,
    self.pub_cmd,
    self.pub_gimbal,
    self.vision,
    self.depth,
    ats_twist_topic="/ats_twist",
    publish_legacy_array=True,
    tf_buffer=self.tf_buffer,
    tf_base_frame="body",
    tf_camera_frame="Camera",
    use_tf_align=True,
    camera_forward_axis="z",
)   # 짐벌(pitch, yaw) + Spot 본체의 이동속도(vx, wz) 를 동시에 계산
    # Executor와 완전히 독립 -> 내부 제어 로직 변경 시 Executor는 영향 없음

self._nav_feedback = {"distance_remaining": None, "stamp": time.time()}
self.navigator = Nav2Navigator(self, self._nav_feedback.update) # move_to 목표 수행에 Nav2를 직접 호출
    # Navigation 내부 상태(goal_reached, aborted 등)는 Navigator가 모두 처리
    # Executor는 Navigator가 제공하는 최소 상태(_nav_feedback)만 참조
        # 예) distance_remaining / stamp

# => Executor는 “도달/실패/재시도/재플랜” 판단에만 집중하여 복잡도 최소화


# --- 동시성과 가시성(visibility) ----
self.lock = threading.Lock()
self.create_timer(0.5, self.publish_state)
# Executor는 다중 스레드·콜백이 동시에 동작
    # 예) run_loop() : 플랜 실행 스레드
    # on_plan_cmd() : 새로운 플랜 수신 콜백
    # on_vision_raw() : 비전 정보 업데이트 콜백
    # publish_state() : 상태 퍼블리시 타이머
# 동일 자원(current_plan, current_index, vision)에 동시 접근 -> 무결성 문제 발생
    # 해결 방안 : 공유 자원 접근 시 Lock 사용
# publish_state : System-2와의 연결 고리
    # 0.5초마다 실행되어, Executor의 전체 상태를 외부로 브로드캐스트
    # 현재 실행 중인 mission_id
    # 실행 중인 플랜 단계(index)
    # queue_status (idle / running / paused / done / error)
    # 현재 guard 값 (ROE_OK, BATTERY_SOC etc)
    # Vision 스냅샷 (primary_id, lost_sec etc)
# -> 1) System-2, UI 모니터링 툴이 로봇의 내부 상태를 실시간으로 시각화 가능
# -> 2) System-1 $\leftrightarrow$ System-2 간 상태 동기화 유지의 핵심 매커니즘


# --- Executor의 플랜 검증 과정 ---
def on_plan_cmd(self, msg): # 입력된 고수준 플랜의 검증 절차를 실행하는 함수
    # 검증 원칙 1. 플랜의 신뢰성을 코드 수준에서 강제
    # msg.plan_json으로 JSON 문자열 전달 후 json.loads를 통해 파싱
        # JSON 파싱 실패 -> 실패 로그 반환
    # self.validator.iter_errors(plan)로 검증 수행
        # HIGH_LEVEL_PLAN_SCHEMA 기반 스키마 검증
        # 이때 오류 발생? emit_replan(...)을 호출, 플랜 제거
    # 원칙 의미 : "LLM이 생성해도, 규격을 만족하지 않으면 실행하지 않는다"
        # -> 물리 시스템의 첫 번째 안전장치
    try:
        plan = json.loads(msg.plan_json)
    except Exception as e:
        # ...
        return

    # 검증 원칙 2. cancel을 최우선 인터럽트 취급
    # if isinstance(plan, dict) and plan.get("intent") == "cancel": 로 중단 여부 해석
        # 플랜이 dict, intent 가 "cancel" 이면 중단으로 해석
    # 모든 액션을 중단하기 위해 _hard_stop() 실행
        # 프로세스를 종료하지 않고 상태만 idle로 만들어 새로운 플랜 받을 준비
    # 원칙 의미 : 상위 시스템이 중단을 요청할 때 즉각 반응
    if isinstance(plan, dict) and plan.get("intent") == "cancel":
        # ...
        self._hard_stop()
        # ...
        self.queue_status = "idle"
        self.publish_state()
        return

    errs = sorted(self.validator.iter_errors(plan), key=lambda e: e.path)
    if errs:
        emit_replan(...)
        return

    #검증 원칙 3. 실행과 수신을 분리하는 비동기 구조
    # 스키마 검증으로 플랜이 유효하면 with self.lock을 통해 파라미터 저장
        # 내부 상태(current_plan, mission_id, current_index)를 기록
    # threading.Thread로 self._run_loop를 별도 스레드 처리
        # on_plan_cmd : 플랜을 받고, 검증하고, 시작 지점을 세팅하는 역할
        # _run_loop : 플랜을 실행하고, 성공/실패/재시도/에러를 관리하는 별도 실행 엔진
    # 원칙 의미 : 콜백은 가볍게하고 실행은 전용 루프
    with self.lock:
        self.current_plan = plan
        self.mission_id = plan.get("mission_id", "")
        self.current_index = 0
        self.queue_status = "running"

    threading.Thread(target=self._run_loop, daemon=True).start()

def _run_loop(self): # 검증된 플랜을 실행하는 함수
# 실행 원칙 . 현재 상태를 기준으로 "다시 판단"하면서 진행
# with self.lock을 통해 이중 검증
    # current_plan, current_index, queue_status 로드
# 실행은 queue_status == "running" 상태에서만 진행
    # 플랜이 없거나 상태가 running이 아니면 즉시 종료
# if plan is None or status != "running": 으로 스텝 수행 여부 확인
    # if idx >= len(steps): 로 step 배열 범위 확인 후 step 수행 종료
    # 모든 step 수행 이후 queue_status를 done
# 각 Step의 4요소 : System-2와 System-1 사이의 최소 실행 단위 계약
    while rclpy.ok():
        with self.lock: #
            plan = self.current_plan
            idx = self.current_index
            status = self.queue_status

            if plan is None or status != "running":
                break

            steps = plan.get("steps", [])
            if idx >= len(steps):
                # ...
                self.queue_status = "done"
                break

            step = steps[idx]
            task = step.get("task")
            params = step.get("params", {}) or {}
            guard = step.get("guard", "")
            retry_left = int(step.get("retry", 0))

            if guard:
                sym = {...}
                if not eval_guard(guard, sym):
                    self.queue_status = "paused"
                    break

            ok = False # ok = False로 초기화 후, task 분기 실행

            if task == "move_to":
                # ...
                pass
            elif task == "scan":
                # ...
                pass
            elif task == "report": # 현재 비전·포즈 상태를 요약해 상위에 전달하는 역할
                ok = self._do_report(params) # 상위 시스템에 정보를 올리는 경량 보고 액션
            elif task == "wait_for_command": # 지정된 시간 동안 대기하는 역할
                ok = self._do_wait_for_command(params) # System-2가 개입할 여지를 제공
            elif task == "track": # Tracker를 호출해 타깃 추적, 설정 시간 내 조건을 만족하는지 확인
                ok = self._do_track(params, plan) # 초기 정렬 및 거리 유지가 충족되는지만 판정 -> 만족 : True를 반환, 다음 step 수행/ 실패 : False를 반환, retry 또는 replan 플로우 
            elif task == "return_to_home":
                # ...
                pass
            else:
                # ...
                ok = False

            if not ok:
                if retry_left > 0:
                    step["retry"] = retry_left - 1
                else:
                    emit_replan(...)
                    self.queue_status = "error"
                    break
            else:
                self.current_index += 1

# 플랜 실행 설계 보장 사항
    # 안전 조건을 만족할 때까진 해당 step을 절대 실행하지 않는다
    # 일시 정지 상태를 상위에서 해석하고 재개할 수 있도록 남겨둔다

# task 분기 : 모든 단위 액션이 "동기 함수 호출 + True/False 반환" -> `_run_loop`는 플로우 제어에 집중
    # True : `current_index += 1` 후 다음 step 실행
    # False : retry 확인 후, 남은 재시도가 있으면 같은 step 반복
    # -> retry 횟수 소진 : `emit_replan(...)` 호출 후 상위 시스템에 진행 중단 로그 발송(`self.queue_status = "error"`)
    
# 설계 장점 1 : 단위 액션이 늘어나도 _run_loop 로직은 고정
# 설계 장점 2 : 액션 내부 구현 변경·교체에도 True/False 계약만 맞으면 그대로 재사용
# 설계 장점 3 : 실패·재시도·재플랜 처리가 모든 액션에 대해 일관된 방식으로 동작

def state_string(self) -> str: # Executor의 상태를 “한 줄” 문자열로 만들기 위한 함수
    if self.queue_status == "running": # 유효한 플랜, 인덱스 보유시 노드와 task 정보 반환
        # ...                               # step = move_to -> return "move_to" / step = track -> return "track"
        return self.current_plan["steps"][self.current_index].get("task", "")
    return self.queue_status                # 미실행 상태 or No 플랜 or 무효한 인덱스 -> return queue_status(idle, paused, done, error)


def publish_state(self): # 다음 항목을 하나의 상태 메시지로 통합해 발송
    vision_snapshot = self.vision.snapshot()
    msg = make_state(
        self,
        self._last_pose, # (현재 위치와 yaw)
        self.mission_id,
        self.state_string(),
        self.queue_status,
        self.current_plan,
        self.current_index,
        self.ROE_OK, 
        self.SAFE_BACKSTOP,
        self.BATTERY_SOC, # guard 심볼들
        self.MAX_SPEED,
        vision_snapshot,
    )
    self.pub_ats.publish(msg)


# --- 단위 액션 설계 ---
# move_to 
# Executor는 move_to()의 True / False 결과로 스텝 진행 / 재시도 / System-2 replan 여부를 결정
if task == "move_to":
    goal = params.get("goal", {})
    replan_rules = plan.get("replan_rules", {})
    ok = exec_move_to(
        self, self.navigator, self._nav_feedback, goal, replan_rules
    )
# self : 로그·TF·상태를 가진 Executor 노드
# self.navigator : Nav2 액션 서버 통신 (Nav2Navigator)
# self._nav_feedback : Nav2 피드백 딕셔너리
# goal, replan_rules : 플랜이 내려준 목표 포즈 / 재플랜 규칙




class Nav2Navigator: # Nav2 NavigateToPose 액션을 감싸는 전용 래퍼 -> 스레드 구조 때문에 폴링 기반 비동기 대기 사용
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
            x = float(goal.get("x")) # goal["x"], goal["y"], goal["yaw"]를 모두 float()로 캐스팅
            y = float(goal.get("y")) # Nav2에 사용될 좌표계를 실수화
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

    def _yaw_to_quat(yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q

        send_goal_future = self._client.send_goal_async( # 비동기 goal 전송
            msg,
            feedback_callback=self._on_feedback,
        )

        while rclpy.ok() and not send_goal_future.done(): # 결과 future가 완료될 때 까지 대기
            time.sleep(0.01)

        # 두 번째 이미지 코드
        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted: # future가 완료되면 goal이 수락 여부를 확인
            self._node.get_logger().warn("[Nav2Navigator] goal rejected")
            return None, None

        self._last_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()  #추가로 result_future를 받아 결과 풀링 준비
        self._node.get_logger().info("[Nav2Navigator] goal accepted")
        return goal_handle, result_future

    # Nav2쪽 래퍼의 역할
        # Nav2Navigator 는 변환된 목표 좌표를 받아 Nav2 액션 시작
        # Nav2 피드백의 형태는 딕셔너리로 콜백
        # 액션 종료 시 result future를 통해 성공/실패를 알려주는 중간 번역기(Wrapper) 역할 수행
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
        eps_m = float(rules.get("progress_epsilon_m", 0.03)) # progress_epsilon_m : 진행으로 인정할 최소 거리 감소량($\Delta$distance)

        accept_t = time.time() # Nav2 goal이 수락된 시각
        last_prog = accept_t # 마지막으로 “유의미한 진행”이 감지된 시각
        best_dist = float("inf") # 지금까지 관찰한 distance_remaining 중 가장 작은 값 => 이 세 값을 계속 갱신하며, 현재 로봇의 움직임 상태를 판단
        last_print = 0.0

        while rclpy.ok():
            time.sleep(0.2)
            now = time.time()

        if now - last_print > 1.0: # 1초 마다 TF에서 가져온 현재 pose(x, y, yaw)와 Nav2 피드백의 남은 거리 값을 함께 로그로 출력
            # TF 조회 성공 : pose(map)=..., dist_remain=... 형태로 현재 위치 + 남은 거리를 계속 모니터링
            # TF 조회 실패 : pose=Unknown(TF fail)이라고 명시, 남은 거리 값 여부 확인
            dist_dbg = nav_feedback.get("distance_remaining", None)
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


# ==== scan 설계 =====
# ---1. Align ---
def _lookup_yaw_rel(tf_buffer: Buffer, base: str, cam: str, # 스캔을 위해 ‘지금 카메라가 어느 방향을 보고 있는지’ 확인
                    camera_forward_axis: str, node: Node) -> Tuple[bool, float]: # Spot의 몸체(body) 기준으로 카메라 yaw(좌우 회전 각도)를 계산하는 함수
    try:
        tf = tf_buffer.lookup_transform(base, cam, rclpy.time.Time()) #  TF 버퍼에서 body -> cam 변환 로드
        R = _quat_to_rotmat(tf.transform.rotation)  # base<-camera / 3×3 회전행렬 R 생성
        fx, fy, fz = _axis_vec(camera_forward_axis) # camera_forward_axis 축 선택 및 벡터 변환
        vx = R[0][0]*fx + R[0][1]*fy + R[0][2]*fz # R * forward 연산으로 몸체 기준 forward 방향 벡터 (vx, vy, vz) 계산
        vy = R[1][0]*fx + R[1][1]*fy + R[1][2]*fz   # 몸체 기준 평면(map XY-plane)에서 카메라 forward 방향을 투영한 벡터
        yaw_rel = -math.atan2(vy, vx) # X와 Y 성분만으로 yaw 각도 계산(부호 : ATS 실제 회전 방향과 ROS 좌표계를 맞추기 위한 보정)
        return True, float(yaw_rel) 
    except Exception as e:
        node.get_logger().warn(f"[scan] TF lookup failed ({base}->{cam}): {e}")
        return False, 0.0 # 실패 시, 경고 메시지 출력 -> scan 액션 동작에 참고(제어 정지, 루프 재시도)

    while rclpy.ok(): # 주기적으로 PID 제어를 통해 안정된 정렬 상태 유지하는 ALIGN 단계 시작 부분
        ok, yaw_rel = _lookup_yaw_rel(...) # 현재 카메라 각도 yaw_rel 계산
        err = -yaw_rel # 정면을 0°로 두고 오차 정의
        err_eff = 0.0 if abs(err) <= deadband else err

        if abs(err) <= (3.0 * deadband): # 아주 작은 오차는 무시(미세 진동/헛움직임을 줄이기 위한 소신호 억제)
            i_term += k_align_i * err_eff * dt # i_term : deadband 주변에서만 누적
            i_term = _clamp(i_term, -align_i_cap, align_i_cap) # 그 외 구간에서는 _clamp()로 +- align_i_cap 제한 -> integral wind-up 방지
        else:
            i_term = 0.0

        d_term = (err - prev_err) / dt # (err - prev_err)/dt 식에 기반하여 1차 LPF 적용한 d_lpf 사용
        d_lpf = (1.0 - 0.2) * d_lpf + 0.2 * d_term # 센서 노이즈에 덜 민감하게, 제어 신호를 부드럽게 만듦

        yaw_rate_cmd = k_align_p * err_eff + k_align_d * d_lpf + i_term # 계산 식의 최종 명령 (yaw 회전 속도)
        # slew-rate 제한 + 최대 회전 속도 제한을 거쳐 짐벌 명령으로 전달
    # => 0° 근처를 스쳐 지나가는 것이 아니라, 정면을 안정적으로 유지할 때만 SWEEP 단계로 넘어가도록 설계

# ---2. Sweep ---
    target_yaw = -half_span # 전체 스윕 범위의 절반 (예: +-60° -> half_span=60°)

    if abs((-half_span) - yaw_rel) <= eps_edge: # 끝점 전환 조건 (좌 <-> 우 왕복 스윕(sweeper 패턴) 자동 형성)
        target_yaw = +half_span # 왼쪽 끝에 도달하면 오른쪽 끝으로, 
    elif abs((+half_span) - yaw_rel) <= eps_edge:
        target_yaw = -half_span # 오른쪽 끝에 도달하면 다시 왼쪽으로

    yaw_err = target_yaw - yaw_rel
    yaw_rate_cmd = _clamp(k_yaw * yaw_err, -yaw_rate_max, +yaw_rate_max)
    # 카메라의 yaw 회전 제어 (PID가 아닌 단순 P 제어만 사용, 정밀 정렬이 아니라 넓은 영역 탐색이므로 간단한 제어로 충분)

    pitch_speed = yaw_rate_max * 0.2
    pitch_rate_cmd = pitch_speed * pitch_dir # pitch_dir : 스윕 방향이 바뀔 때마다 +1 <-> -1 반전
    # 좌 -> 우 스윕: 카메라는 위쪽으로 천천히 이동
    # 우 -> 좌 스윕: 카메라는 아래쪽으로 천천히 이동
    # 결과: 좌우+상하 지그재그 스캔 -> 사각지대(위/아래) 최소화\
    # sweep단계: _lookup_yaw_rel()로 현재 위치 파악, 방향 전환 및 제어

    snap = vision.snapshot() # SWEEP 루프 동안 매 틱마다 비전 정보 조회 (조회 대상: class, id, center, bbox 정보)
    objs = snap.get("objects") or []
    if objs and (watch_classes or watch_ids): # 목표 대상 사람을 찾고 싶으면 watch_classes = ["person"], Tracking ID를 찾는다면? watch_ids = ["17"]
        cls_set = {(c.lower() if isinstance(c, str) else str(c)) for c in (watch_classes or [])}
        # watch_classes를 모두 소문자로 통일한 집합
        id_set = {str(i) for i in (watch_ids or [])}
        # watch_ids를 문자열로 통일한 집합
        for o in objs: # class가 cls_set 또는 id가 id_set에 포함이면 목표로 간주
            oc = o.get("class")
            oid = o.get("id")
            if (cls_set and oc in cls_set) or (id_set and oid in id_set):
                handle_found(o) # 목표를 감지하면 호출
                stop_all() # 짐벌과 이동 명령 정지/ 스캔 동작 종료
                return True

# ---3. Found ---
    msg = { # event 이름, 객체의 정보 등을 포함하는 Found 객체
        "event": "FOUND", # "Found"고정
        "class": obj.get("class"), # 발견된 객체의 핵심 정보
        "id": obj.get("id"),
        "center": obj.get("center"),
        "bbox": obj.get("bbox"),
        "time": time.time(), # 탐지시각
    } # => Json으로 직렬화하여 /scan_report 토픽으로 퍼블리시
    #  scan 액션은 System-2를 호출하지 않고, /scan_report 토픽에 Found 이벤트만 발행
    # scan노드는 "탐색 + found 이벤트 발행"에만 집중
    # 행동(추종, 보고, 무시 등)은 상위 로직이 자유롭게 결정
    # scan 액션을 독립 모듈로 유지하면서도 재사용·확장이 쉬운 구조

    report_pub.publish(String(data=json.dumps(msg)))

# --- 기타 액션: 발견 직후 보고 까지 자동 실행 액션
    # 기본 흐름: scan -> 대상 발견(FOUND) -> /scan_report 이벤트 발행 -> 스캔 종료
    if report_on_found and hasattr(node, "_do_report"): 
    # report_on_found = True일 때, FOUND 직후 _do_report()까지 호출 
    # False일때 FOUND 이벤트만 보내고 이후는 System-2 또는 다른 노드가 처리
    # hasattr(node, "_do_report") : node가 _do_report()를 가지고 있을 때만 호출
        # scan 코드는 _do_report() 구현을 전혀 모르는 느슨한 결합 구조
        node.get_logger().info("[scan] report_on_found -> _do_report() 호출")
        try:
            node._do_report({"delay_sec": 0.2}) # 즉시 보고 시작 대신 0.2초 정도 안정화 시간을 주라는 힌트
        except Exception as e:                      # 탐색 ->  발견 -> 잠깐 안정화 -> 보호 흐름을 자연스럽게
            node.get_logger().warn(f"[scan] _do_report failed: {e}")

# 전체 동작 흐름 : report_on_found=True 일때
    # 1. 스윕 중 Vision에서 목표 객체 감지
    # 2. handle_found()가 FOUND 메시지 구성
    # 3. /scan_report 로 발행
    # 4. _do_report({"delay_sec": 0.2}) 호출
    # 5. System-1 내부 “보고” 액션 실행

# ===== track 설계 =====
    # Detection-Tracking 이 아닌 통합된 3축 통합 제어(gimbal + yaw + distance)
    tracker.start(params, rules)                 # 추적 시작
    # 주기적으로 _control_step() 을 반복 실행하는 방식
    # _control_step() 는 3축 제어가 포함 -<  주기적으로 바디·짐벌·속도 명령 생성
    tracker.wait_initial_success(timeout)        # 초기 정렬 성공 대기(선택)
    # => System1은 “트래킹 제어의 성공 여부”만 판단, 세부 제어는 Tracker 내부 처리

    class DepthBuffer: # 이미지 상 bbox에서 실제 거리 뽑아내기
        """
        최신 depth 이미지를 보관하고, bbox ROI의 대표 depth(m)를 계산한다.
        - 인코딩 자동 처리(32FC1=미터, 16UC1=mm->m)
        - 프레임 크기 매핑                  # /depth 이미지를 캐시하고 bbox ROI의 대표 거리값(range_m)을 계산해 Tracker에 제공
        - 중앙부 크롭/백분위수/최소 유효비율   # ”화면 중앙”만으로는 부족 -> 사람(타깃)과 로봇 사이의 실제 거리를 알아야 vx 제어(전진/후퇴) 가능
        - 오래된 프레임 무시(max_age_sec) # 일반적으로 depth 이미지를 최신 상태로 유지, 요청 시 bbox안 타깃의 실제 거리 값 전달
        """
        # _control_step(): Track의 제어 루프 역할
        # 비전 정보 읽기 -> 타깃 선택 -> 오차 계산 -> gimbal 회전 -> body 정렬 -> 거리 유지
        # 요약: Vision 기반 타깃 선택 + Sticky Lock 유지
        v = self.vision.snapshot() # 
        objs = v.get("objects", []) # 현재 프레임에서 감지된 모든 객체 리스트 (각 객체의 bbox(x,y,w,h), center(x,y), id, class 정보)
        fw = int(v.get("frame_w", 1280)) # 현재 프레임 해상도
        fh = int(v.get("frame_h", 720))
        cx, cy = fw * 0.5, fh * 0.5 # 화면 중심 좌표 계산 , 이후 yaw/pitch 오차 계산 기준점 생성
        
        cand = self._choose_target(objs, primary_id) # 타깃 선택을 위한 중요 함수
        # 주어진 primary_id가 있고, 현재 프레임에 보이면 -> Tracker는 해당 ID를 무조건 타깃
        # 주어진 primary_id가 없고, 현재 프레임에 없다면 -> bbox 면적이 가장 큰 객체를 선택
        if cand is None:
            self._publish_zero_once() # 타깃 미검출 시 명령 값 0으로 전달
                # 처리(오차 계산, 거리 제어 등)는 모두 스킵
            return False, False, 0.0, 0.0, 0.0, 0.0, None, None

        self._update_lock(cand) 
        # 한 번 고른 타깃을 가능한 오래 유지하는 구조 (sticky lock 구조)
            # 같은 ID가 계속 보임 -> 그대로 유지(sticky)
            # 같은 ID가 사라짐 -> lock 해제 + zero command + 이번 틱 종료
            # ID가 다른 객체로 바뀜
                # 로그로 변경 사실 출력
                # PID항 초기화, anti-windup -> 새 타깃으로 부드럽게 전환
                    # PID항 초기화 이유: 다른 타깃 전환 시 이전 타깃의 오차 누적값으로 인한 오버슛 방지

        if id_changed:
            self._yaw_i = 0.0

# --- 오차 계산 ---
        bbox, center = cand.get("bbox"), cand.get("center")
        # 타깃이 중앙과 얼마나 차이가 있는지 수치적 표현하는 핵심 정보
            # 계산 방법타깃 중심 : x + w/2, y + h/2 (없으면 center["x"], center["y"] 사용)
            # 화면 중앙 : cx, cy = frame_w/2, frame_h/2
            # 픽셀 단위 오차(err_px_x, err_px_y) = 타깃 중심 - 화면 중앙
        if bbox and len(bbox) >= 4:
            x, y, w, h = bbox
            err_px_x = (x + w * 0.5) - cx
            err_px_y = (y + h * 0.5) - cy
        elif isinstance(center, dict) and "x" in center and "y" in center:
            err_px_x = float(center["x"]) - cx
            err_px_y = float(center["y"]) - cy
        else:
            self._publish_zero_once()
            return False, False, 0.0, 0.0, 0.0, 0.0, None, None

        if abs(err_px_x) < px_deadband: err_px_x = 0.0 # 데드밴드로 미세 흔들림 제거
        if abs(err_px_y) < px_deadband: err_px_y = 0.0  # 손떨림, 센터 지터 수준에서는 짐벌이 흔들리지 않도록 억제

# --- 오차 계산 (Gimbal PID 제어): yaw, pitch 각각에 대해 P, D, I 항으로 제어 ---
        derr = (err_px_x - self._yaw_prev_err) / dt2 # 오차 변화율에 해당하는 미분(D)항
        self._yaw_d_lpf = (1.0 - yaw_d_lpf_a) * self._yaw_d_lpf + yaw_d_lpf_a * derr
        # self._yaw_d_lpf: 저역통과 필터로 sudden jump 현상, 미세 진동에 따른 D항의 과잉반응 억제
        allow_i = (abs(err_px_x) <= (px_deadband * 3.0)) # 누적된 오차를 줄이는데 효과적
        yaw_i_next = self._yaw_i
        if k_yaw_i > 0.0 and allow_i: # 오차가 작아진 구간에서만 적분을 허용하여 windup 방지
            yaw_i_next = self._yaw_i + (k_yaw_i * err_px_x * dt2)
            yaw_i_next = _clamp(yaw_i_next, -i_cap, +i_cap)

        yaw_rate_cmd = -(k_yaw * err_px_x + k_yaw_d * self._yaw_d_lpf + yaw_i_next)
        # 각 P, D, I 항 계산식을 합쳐 yaw 각속도 명령 생성
        
        # Slew 제한 : 급격히 변화한 명령어를 제한
        max_step = slew_cap * dt2
        yaw_step = _clamp(yaw_rate_cmd - self._yaw_cmd_prev, -max_step, +max_step)
        # 급격히 변화한 명령어를 제한하기 위해 clamp 적용 (slew 제한)
        yaw_rate_cmd = self._yaw_cmd_prev + yaw_step

        yaw_rate_sat = _clamp(yaw_rate_cmd, -gimbal_rate_cap, gimbal_rate_cap)
        # 움직임 속도가 짐벌의 제한 속도를 초과하지 않도록 조치
        tw_ats = Twist()
        tw_ats.angular.z = float(yaw_rate) # ATS로 Twist 발행
        tw_ats.angular.y = float(pitch_rate) # -> 매 틱마다 부드럽고 안정적인 yaw/pitch 회전 유지
        self.ats_twist_pub.publish(tw_ats)   # -> 화면 중앙 기준으로 자연스럽게 타깃 추적

# --- 바디 정렬 ---
# 필요 이유 1 : 짐벌만 돌리면 카메라는 타깃을 보는데 spot은 엉뚱한 방향을 보는 자세가 오래 유지됨
# 필요 이유 2 : 이런 상태에선 이동 제어, 장애물 회피, 거리 유지가 불안정
# 필요 이유 3 : “카메리가 보는 방향 = Spot 몸 방향“ 이 되도록 바디 yaw(wz)도 함께 정렬
        ok_tf, yaw_rel, vxy = self._lookup_yaw_rel(log_period) # 카메라와 몸체 사이의 yaw 관계를 TF 기반으로 계산
        if not ok_tf: # 반드시 TF 조회 필요 (조회 실패시 -> 잘못된 자세에서 억지 회전 금지)
            t0 = Twist(); t0.linear.x = 0.0; t0.angular.z = 0.0 # 선속도(vx), 각속도(wz)에 0을 한 번 보내고 안전하게 틱 종료
            self.cmd_pub.publish(t0)
            return False, False, 999.0, 0.0, 0.0, yaw_rate, None, None

        dyaw = yaw_rel - self._prev_yaw # yaw_rel: 카메라 기준으로 몸이 얼마나 틀어져 있는지 나타내는 값(음수: 왼쪽, 양수: 오른쪽)
        dyaw_dt = dyaw / dt  # yaw_rel의 변화율
        self._dyaw_lpf = (1.0 - kd_lpf_alpha) * self._dyaw_lpf + kd_lpf_alpha * dyaw_dt
        # '타깃 흔들림’, ‘depth 기반 jitter’로 dyaw_dt가 튐을 막기 위한 필터
            # 몸 회전(wz)이 과민하게 흔들리지 않도록 안정화
        
        yaw_err_deg = abs(math.degrees(yaw_rel))
        aligned = False
        if yaw_err_deg <= float(cfg.get("yaw_deadband_enter_deg", 4.0)): # 입장 데드밴드 (yaw_err_deg 4° 이하 -> Tracker : “충분히 정렬됨” -> wz = 0 : 회전 완전히 정지)
            aligned = True
            wz = 0.0
        else:
            if yaw_err_deg < float(cfg.get("yaw_deadband_exit_deg", 6.5)): # 퇴장 데드밴드 (yaw_err_deg 4° ~6.5° -> Tracker : "거의 정렬된 상태 유지” -> wz: 최소한의 보정)
                aligned = True
            wz = float(cfg.get("yaw_align_sign", -1.0)) * (                 # 기타: yaw_err_deg 6.5° 초과 -> P, D 기반으로 적극적인 회전 보정
                float(cfg.get("kp_align", 3.0)) * yaw_rel + float(cfg.get("kd_align", 2.0)) * self._dyaw_lpf
            )
        # 기대 효과 : 거의 정렬된 상태에서 불필요한 흔들림 방지, 크게 틀어졌을 땐 확실하게 회전해서 맞추기
        # 동작 정리 : 카메리가 타깃을 본다 -> 몸도 그 방향을 향한다 -> Spot 전체가 자연스럽게 따라간다

# ---거리 유지 ---
        bbox = self._lock.get("bbox")
        fw = int(v.get("frame_w", 1280))
        fh = int(v.get("frame_h", 720))

        range_m = None # spot이 타깃과 거리 유지에 필요한 거리 정보(캐싱된 이전 유효 거리값을 재사용해 부드럽게 연결)
        if bbox and self.depth is not None and self.depth.has_image():
            range_m = self.depth.roi_mean_depth( # range_m을 계산하기 위해 아래 과정 수행
                bbox, fw, fh, # RGB bbox -> depth 프레임 좌표계로 스케일링
                depth_valid_min=depth_valid_min, depth_valid_max=depth_valid_max,
                center_crop=depth_center_crop, # 비정상 값 제거 (NaN / inf / 너무 작거나 큰 값)
                use_percentile=depth_use_percentile, # 중앙부 crop + 백분위수(percentile)로 대표값 추출
                min_valid_ratio=depth_min_valid_ratio # 
            ) # 이후 vx계산의 밑거름

        if (range_m is not None) and (yaw_err_deg <= move_when_yaw_deg): # 조건 1 : range_m 유효 (depth 정상 수신) 조건 2 : 몸이 충분히 타깃 방향을 보고 있을 때만 전진/후퇴 허용 
            dist_err = (range_m - follow_dist) # 현재 거리(range_m)와 목표 거리(follow_dist)의 차이
            vx = _clamp(k_follow * dist_err, -vx_cap, +vx_cap) # dist_err > 0 -> 너무 멀다 -> 앞으로 이동 (+vx)
            if abs(vx) < vx_min_abs:                           # dist_err < 0 -> 너무 가깝다 -> 뒤로 이동 (-vx)
                vx = 0.0 # dist_err에 비례해 결정, vx의 상,하한을 vx_cap으로 제한
        else:           # 자잘한 움직임이 vx_min_abs이하일 때, 0으로 처리
            dist_err = None
            vx = 0.0

        tw = Twist()
        tw.linear.x = float(vx)
        tw.angular.z = float(wz)
        self.cmd_pub.publish(tw) 
        # 트위스트 메시지로 계산된 선속도 vx + 회전속도 wz를 spot에 전송
            # 짐벌과 바디가 이미 타깃 쪽을 보고 있는 상태에서 Spot이 앞/뒤로 자연스럽게 거리 유지
        # 이후 _control_step()은 매 틱마다 aligned, moving, yaw_err, wz, vx, range_m, dist_err 를 반환
        # 상위 루프(_thread_main() / run())는 이 값을 이용해 초기 성공 판정, 추적 유지/타임아웃 판단을 수행
            # => 빠른 환경에서도 track 동작 전체를 안정적으로 관리할 수 있는 구조


# === report_and_wait ===
# --- report ---
def _summarize_context( # 사람이 읽기 쉬운 한 줄 짜리 상황 요약 문자열 생성
    mission_id: str,    # 현재 미션 식별자
    pose: Dict[str, Any],   # x, y, yaw, ok (TF 기준 map 좌표 + 유효 여부)
    vision_snapshot: Dict[str, Any], # primary_id, targets 리스트
) -> str:
    """사람이 읽기 쉬운 한 줄 요약 문자열 생성."""
    x = pose.get("x", 0.0)
    y = pose.get("y", 0.0)
    yaw = pose.get("yaw", 0.0)
    pose_ok = pose.get("ok", False)

    primary_id = vision_snapshot.get("primary_id")
    targets = vision_snapshot.get("targets", []) or []
    num_targets = len(targets)

    if targets: # targets [0]을 대표 타깃으로 사용해 class, range_m을 얻고 읽기 쉬운 문장으로 로그 출력
        t0 = targets[0]     # 예) "3개 타깃 탐지, 주요 타깃: id=35, person (거리 약 4.2m)"/"감지된 타깃 없음"
        cls = t0.get("class", "object")
        rng = t0.get("range_m")
        desc = f"{cls}"
        if rng is not None:
            desc += f" (거리 약 {rng:.1f}m)"
        target_str = f"{num_targets}개 타깃 감지, 주요 타깃: id={primary_id}, {desc}"
    else:
        target_str = "감지된 타깃 없음"

    if pose_ok: # TF 성공 여부에 따라 exec_report_and_wait에서 다른 로그 출력
        pose_str = f"map 기준 위치=({x:.2f}, {y:.2f}), yaw={yaw:.2f}rad"
    else:       # 디버깅에 유용, 운용자가 "어디서, 어떤 미션으로, 무엇을 보고있는지"를 한 눈에 파악
        pose_str = "map 기준 위치=Unknown(TF 실패)"

    return (
        f"[REPORT] mission={mission_id}, "
        f"{pose_str}, "
        f"{target_str}"
    )


def exec_report_and_wait( # 단위 액션 엔트리 포인트
    node: Node,
    vision,
    last_pose: Dict[str, Any],
    mission_id: str,
    params: Optional[Dict[str, Any]] = None,
) -> bool:
    """
    System-1 단위 액션: report_and_wait

    1) 현재 pose + vision_snapshot를 요약해서 로그로 출력
    2) publish_state() 호출해서 /ats_state 최신화
    3) 동일한 컨텍스트를 /system2/report_context 로 JSON 형태로 publish
    4) /system1/report_decision 을 기다리면서 블로킹
       - decision='continue' -> True 반환 (다음 task로 진행)
       - decision='new_plan' -> True 반환 (곧 /high_level_plan으로 플랜 교체)
       - decision='end' -> True/False는 정책에 따라, 일단 True
    모든 것이 task: "report_and_wait" 명령으로 수행 가능    
    운용자 입력 + LLM 호출 + 새 플랜 생성은 전부 System-2에서 처리.
    """

    params = params or {} # 옵션 파라미터 기본값 정리 (비어있을 경우 빈 딕셔너리로)

    vision_snapshot = vision.snapshot() # 비전 상태 딕셔너리 반환
    summary = _summarize_context(mission_id, last_pose, vision_snapshot) # 사람이 읽기 쉽도록 한 줄짜리 문장 생성
    node.get_logger().info(summary) # _summarize_context 로 생성한 문장으로 로봇의 상태를 출력

    try:
        if hasattr(node, "publish_state"): # publish_state 메서드 호출 전, 메서드 존재 여부 확인
            node.publish_state() # => Node에 따라 publish_state가 없을 수도 있으므로 방어적 수단으로 사용
    except Exception as e:
        node.get_logger().warn(f"[report_and_wait] publish_state 예외: {e}")

    payload = {
        "mission_id": mission_id, # 현재 미션을 식별하기 위한 키
        "context": {
            "pose": last_pose,
            "vision": vision_snapshot,
            "state_string": getattr(node, "state_string", lambda: "")(), # 현재 실행중인 단위 액션 이름
    },                      # getattr()로 존재 여부 확인 후, 메서드 없으면 빈 문자열로 fallback
        "source": "system1_report_and_wait", # System-2에서 “어떤 경로로 올라온 컨텍스트인지” 구분하는 태그
    }

    pub_ctx = getattr(node, "pub_report_context", None) #퍼블리셔 존재 유무를 담는 변수
    if pub_ctx is None:                                     # pub_report_context 없음 -> System-2과 미연동 상황으로 간주 -> 경고 로그
        node.get_logger().warn(
            "[report_and_wait] pub_report_context가 없어 System-2로 컨텍스트를 전송하지 못했습니다. "
            "System2 연동 전까지는 로그 확인용으로만 동작합니다."
        )
        # 그래도 흐름만 유지
        time.sleep(float(params.get("fallback_delay_sec", 0.5)))
        return True

    msg = String()
    msg.data = json.dumps(payload, ensure_ascii=False) # ensure_ascii : JSON 문자열 생성 시, 한글 / 유니코드 지원 여부 결정 False: 한글/ 유니코드 지원 True: ASCII 코드만 지원
    pub_ctx.publish(msg)
    node.get_logger().info("[report_and_wait] System-2로 report_context 전송 완료")


    decision_holder = {"value": None}

    def decision_cb(dec_msg: String):
        try:
            data = json.loads(dec_msg.data)
        except Exception as e:
            node.get_logger().warn(
                f"[report_and_wait] /system1/report_decision JSON parse 실패: {e}"
            )
            return

        mid = data.get("mission_id")
        if mid and mid != mission_id: # JSON 메시지 내, mission_id가 현재 미션과 맞는 경우에만 처리
            # 다른 미션이면 무시 # 미션의 병렬 진행, 큐잉 구조를 염두
            return

        if decision_holder["value"] is not None:
            return

        decision_holder["value"] = data.get("decision", "continue")
        # 콜백 밖에서 공유하기 위한 딕셔너리
        node.get_logger().info(
            f"[report_and_wait] decision 수신: {decision_holder['value']}"
        )

    sub = node.create_subscription( # 실제 구독자를 생성
        String,
        "/system1/report_decision",
        decision_cb,
        10,
    )

    timeout_sec = float(params.get("wait_timeout_sec", 600.0)) # 기본 600초 초과 여부 체크 후 decision 없는 것으로 간주
    t0 = time.time()

    try:
        while rclpy.ok() and decision_holder["value"] is None:
            if timeout_sec > 0.0 and (time.time() - t0) > timeout_sec:
                node.get_logger().warn( # 타임아웃 시 경고 로그 남기고, decision 없으면 "continue"로 간주
                    "[report_and_wait] decision timeout -> 'continue' 로 처리"
                )
                break
            time.sleep(0.1)
    finally:
        # 구독자 정리
        try:
            node.destroy_subscription(sub) # 반복 호출 시, 구독자 초기화
        except Exception:
            pass
    decision = decision_holder["value"] or "continue"
    node.get_logger().info(f"[report_and_wait] 최종 decision={decision}")

    # 아래와 같은 이유로 의도적으로 모든 경우에 True 반환
    # 현재 플랜의 한 스텝 역할만 담당 (실제 시나리오 제어는 상위로직에서 처리)
    if decision == "continue": # 아무 것도 못받거나 TIMEOUT일 때
        return True
    elif decision == "new_plan": # System-2가 이미 /high_level_plan을 publish 했다고 가정
        return True
    elif decision == "end": # 이후 플랜 종료/정리 정책은 Excecutor 상위 레벨에서 결정
        return True
    else:
        return True