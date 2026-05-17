class System2Node(Node):
    def __init__(self):
        super().__init__('system2_node') # 시스템 동기화를 위한 구독(Subscription) 설정

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

        self.decision_pub = self.create_publisher(
            String,
            '/system1/report_decision',
            10,
        ) # report_and_wait 상태일 때, 운영자의 승인/지시(Decision) 를 즉시 전달하는 별도 제어 채널

        self._keyboard_thread = threading.Thread(
            target=self._operator_input_loop,
            daemon=True,
        ) # 별도의 데몬 스레드로 _operator_input_loop 실행 하여 spin 루프의 블로킹 문제를 방지
                # 로봇의 자율 동작은 계속 유지, 운영자는 콘솔로 언제든지 비동기적(Asynchronously) 으로 명령 입력 가능
        self._keyboard_thread.start()

    # LLM이 만든 JSON이 스키마와 살짝 다를 때를 대비한 전처리 함수
        # 좌표 데이터(객체 형태)가 아닌 플랫한 형태로 반환되는것을 스키마 규격에 맞게 강제 변환
        # System-2 전체의 Robustness(강건성) 를 높이는 방어적 프로그래밍
    def _normalize_plan_for_schema(self, plan: Dict[str, Any]) -> Dict[str, Any]:
        """ 
        HIGH_LEVEL_PLAN_SCHEMA 검증 전에...
        - task == "move_to" 인 step 은 params.goal dict 로 묶어줌
        """
        # ... (중략) ...
        if task == "move_to":
            goal = params.get("goal")
            if not isinstance(goal, dict):
                x = params.get("x")
                y = params.get("y")
                # ... (중략) ...
                goal_dict = {"x": float(x), "y": float(y), "yaw": float(yaw)}
                params["goal"] = goal_dict

# 수신한 현장 데이터를 로그/요약 형태로 출력 -> input()을 통해 운영자의 추가 명령을 동기적으로 대기
    def report_context_callback(self, msg: String):
        """
        System1이 report_and_wait 액션에 의해 상황을 보내주는 경우.
        1) 현장을 로그로 요약 통보
        2) 터미널에서 운영자 자연어 명령을 input() 으로 입력 받음
        """
        # ... (중략: 메시지 파싱 및 로그 출력) ...

        prompt = (
            "\n[System2/report_and_wait] 위 스크린 기준으로 추가 명령을 입력하세요.\n"
            "> "
        )
        user_cmd = input(prompt)


    # ... (중략) ...
    plan_dict = build_plan_dict(
        user_command=user_cmd,
        system1_state=self.latest_state,
        extra_context=extra_context,
    ) # self.latest_state + extra_context + user_cmd를 합쳐 즉시 Re-planning
        # 새로 생성된 플랜을 /system2/plan_cmd로 publish 하고 새로운 계획이 하달됨을 전송 -> System1 대기 상태 해제
    # ... (중략: 플랜 publish) ...

    publish_decision("new_plan", "new high_level_plan published")

    # 새로운 명령이 없을 경우 no_command를 전송해 로봇이 계속 임무 수행하도록 처리
    if _should_treat_as_no(user_cmd):
        publish_decision("no_command", "operator chose no additional command")
        return 