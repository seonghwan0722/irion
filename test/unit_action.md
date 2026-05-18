# move_to
- Nav2 서버 준비 여부, 이동 진행 여부, Nav2 최종 결과를 종합, 이동 성공 여부 동기 액션 함수
- 성공 여부에 따라 다음 step으로 갈지, retry 또는 replan갈지 판단
- 이동 로직을 바꿔도 플랜 실행 구조는 그대로 재사용 가능

## 기대흐름
1. [Plan] : 플랜 파싱·스키마 검증 통과
2. `_run_loop` : task=move_to 실행
3. Nav2Navigator : "navigate_to_pose" 액션 서버 연결을 시도
4. 준비 되면? : `[Nav2Navigator] send goal: x=2.500, y=1.000, yaw=0.000` 로그 출력
5. 주기적으로 `[move_to] pose(map)=(...), dist_remain=...` 로그와 남은 거리 값 출력

A 상황 또는 B 상황

- **A 상황(정상) : distance_remaining 지속 감소**
1. `[move_to] Nav2 SUCCEEDED` 로그 출력
2. `exec_move_to` → True / queue_status: running → done
3. `/ats_state` echo
    - mission_id = test_move_001
    - state_string: "move_to" → "done"
- **B 상황(실패): 실패 / stuck 케이스**

7. 남은 거리값이 거의 줄지 않은 상태가 grace 4초 이후 15초 이상 지속?

- `[move_to] 진행 정체 감지 -> Nav2 goal cancel & 실패 반환`
- Nav2가 직접 FAILED 반환 →  `[move_to] Nav2 FAILED, status=...`

8. 두 경우 모두 `exec_move_to` → False

9. Executor는 step을 실패로 기록

10. retry 남아 있으면 재시도, 없으면 `emit_replan` 으로 System-2에 재계획 요청

# scan
로봇 위치는 그대로 두고, ATS 카메라만 회전시키면서 주변을 탐지하는 액션

## 기대흐름
**플랜 수신 & 준비**

1. `/system2/plan_cmd` → `PlanCommand.plan_json` 수신
2. `on_plan_cmd()`로 JSON 파싱
3. `HIGH_LEVEL_PLAN_SCHEMA`로 스키마 검증
4. 스키마 검증 성공 시, `current_plan` 저장
5. `current_index = 0` 초기화
6. `queue_status = "running"` 설정
7. 별도 스레드에서 `_run_loop()` 시작 (step 순차 실행)

**scan step 선택 & exec_scan 진입**

1. 플랜은 steps에 단 한 개의 step(`task: "scan"`)
2. `_run_loop()`의 `elif task == "scan"` 분기로 진입
3. params에 담긴 값들을 그대로 `exec_scan()` 인자로 전달
4. 이후 제어 흐름은 온전히 `exec_scan()` 내부로 이동
5. 내부에서 ALIGN → SWEEP → FOUND 처리까지 한 번에 수행

**ALIGN 목표**

1. 카메라의 상대 yaw(`yaw_rel`)를 body 기준 정면 ($0^\circ$) 으로 정렬
2. `_lookup_yaw_rel()`로 매 루프마다 현재 yaw 측정
    - `deadband` : 작은 오차는 0으로 취급 → 신동/과제어 방지
    - `i-term` : deadband 근처에서만 적분 누적 (그 외 구간에서는 적분 리셋 → wind-up 방지)
    - `d-term` : LPF를 거쳐 노이즈 완화
    - `yaw_rate_cmd` : slew-rate 제한 + 최대 속도 제한 후 짐벌로 전송
3. ALIGN 성공 기준을 만족하면 로그 출력 후 SWEEP 단계 진입
    - 로그: `[scan] ALIGN done (yaw=0°)`

---

- **SWEEP 단계**
    1. **yaw 제어** : 카메라 좌·우 왕복 스윕(`half_span` ~ `+half_span`)
        - 예: sweep_deg = 120 → half_span = 60° → 스캔 범위: -60° ~ +60°
    2. **pitch 연동하여 상·하 왕복 스윕**
        - yaw 속도 기반으로 `pitch_rate_cmd` 생성
    3. **제어 상태 및 비진 인식 객체 로그 출력**
        - 로그 예시:
            
            `[scan/SWEEP] yaw_rel=-57.8°, target=-60.0°, cmd_yaw_rate=-3.4°/s`
            
            `[scan] vision objects=2`
            
            `[scan] vision sample -> class=person, id=35, center={'x': ..., 'y': ...}`
            
    4. SWEEP 동안 `VisionCache`가 `/vision_context_raw`를 계속 구독
    5. `exec_scan()`은 주기적으로 `snapshot`을 읽어, 객체 (`watch_classes` or `watch_ids`) 여부 검사

---

- **FOUND (발견)**
    1. 조건 만족 시, `handle_found()` 호출
    2. `/scan_report`로 FOUND 이벤트 발행
        - 로그: `[scan] FOUND -> {"event": "FOUND", "class": "person", "id": "35", ... }`
    3. 추가로 `report_on_found`가 true라면? `_do_report()`까지 자동으로 호출
        - 로그: `[scan] report_on_found -> _do_report() 호출`
    4. `_do_report()` 호출 후 스윕 중단, True 반환
    5. `_run_loop()`는 ok == True 확인
    6. `current_index++`
    7. 남은 step 없으면 플랜 완료 처리
- **FOUND (미발견)**
    1. `duration_sec` (예: 25초) 기준으로 ALIGN+SWEEP 전체 시간 체크
    2. 시간이 초과되면 로그 출력
        - 로그: `[scan] duration elapsed - done`
    3. `exec_scan()`은 True 반환

# track

## 기대흐름
- **Tracker 시작**
    1. `start(params, rules)` 호출 → “이 타깃을 이런 조건으로 추적해라” 파라미터 전달
    2. 내부 제어 스레드가 켜짐과 동시에 짐벌 제어 + 바디 yaw 정렬 + 거리 유지 동작 시작
    3. 이미 추적 중이었다면? 새 스레드는 만들지 않고 파라미터만 갱신
        - ⇒ 상황 변화에 따라 부드럽게 추적 조건 업데이트
- **초기 정렬 성공 대기**
    1. `wait_initial_success(timeout)` 호출 → 아래와 같은 조건 평가
        - 타깃이 화면 중앙 근처에 들어왔는가?
        - 바디 yaw 정렬 오차가 기준 이하인가?
        - 거리가 너무 가깝거나 멀지 않은가?
    2. 조건들이 일정 시간 만족되면 True 반환 → “초기 lock 완료, 다음 단위 액션 동작”
- **추적 중단 및 상태 조회**
    1. `stop(flush=True)` 호출 → ATS 짐벌 / 바디에 0 속도 명령을 여러 번 전송
    2. 잔여 명령 없이 “멈춘 뒤 안정화”까지 포함된 정지
- **추적 중단 및 상태 조회**
    1. `status()` 호출 → 정렬 오차, 최근 depth / range, 현재 속도 명령 등 내부 스냅샷 조회 가능

# report_and_wait
- 컨텍스트 보고 + System-2 결정 수신까지 대기 + 다음 스텝으로 안전하게 복귀
- 물리적 액션 이후 현재 로봇의 위치 및 비전 상태를 System-2에게 전달 + 반환 받은 의사결정으로 다음 플랜

## 기대흐름



# return_to_home


## 기대 흐름