from typing import List, Optional, Dict, Any, Literal
# typing: List, Dict, Optional, Literal 등 타입 힌트 제공
from pydantic import BaseModel, Field
# 데이터가 우리가 정한 규칙에 맞는지 검사
    # LLM이 만들어낸 결과를 사전에 걸러주는 거름망 역할

class Step(BaseModel): # 로봇이 수행할 "행동 하나"를 표현하는 모델
    task: Literal[ # task 환각에 대비-> 5개 문자열만 인정, 나머지는 에러 처리
        "move_to", # 허용된 문자열
        "scan:",
        "report_and_wait",
        "return_to_base",
    ]
    params: Dict[str, Any] = Field(default_factory=dict)  # task별로 필요한 데이터가 다르기에 행동마다 데이터 성향에 맞게 유연하게 설정
    guard: Optional[str] = None
    retty: int=Field(0,ge=0) # Field(0, ge=0) -> retty는 0 이상만 허용, 음수는 에러 처리

class ReplanRules(BaseModel): # 미션 도중 돌방 상황 발생 시 어떻게 대응할지를 정하는 규칙 세트
    lost_target_sec: float = 5.0 # 추적 대상 놓쳤을 때 최대 5초까지는 다시 찾아보고, 넘으면 포기
    battery_rtb: float = Field(0.2, ge=0, le=1.0)  # 배터리 잔량이 20% 이하일 때 귀환 명령
    hard_stuck_timeout_sec: float = 10.0     # 행동이 10초 이상 진행 중인데 변화가 없다면 재계획 필요

class HighLevelPlan(BaseModel): # System-2가 만들어내는 "고수준 계획 1개"를 통째로 표현하는 모델
    version: Literal["1.0.0"] = "1.0.0"  # 계획의 버전 정보, 향후 계획 형식이 변경될 때 참조
    mission_id: str  # 미션 ID
    intent: str  # 의도("A구역 정찰 후 복귀" 등)를 사람 눈으로도 이해 가능하게 기록
    constraints: List[str] = Field(default_factory=list)  # 행동 계획을 세울 때 고려해야 하는 제약 조건들
    steps: List[Step]  # 앞에서 정의한 Step들이 순서대로 들어가 로봇이 따라갈 시나리오 완성
    replan_rules: ReplanRules = ReplanRules()  # 이 미션을 수행할 때 적용할 안전, 재계획 규칙을 함께 전달

class VisionSnapshot(BaseModel): # 로봇이 시각적으로 인식한 정보를 표현하는 모델
    summary: Optional[str] = None  # 시각적 정보에 대한 간단한 요약

class SystemState(BaseModel): # 로봇의 현재 상태(System-1) 종합 보고서
    mission_id: Optional[str] = None  # 현재 수행 중인 미션의 ID
    system1_state: Optional[str] = None  # 대기/이동/수행 중 등 전체 상태
    current_task: Optional[str] = None  # 현재 수행 중인 행동
    step_index: Optional[int] = None  # 현재 행동이 몇 번째 행동인지
    pose: Optional[Dict[str, float]] = None  # 로봇의 자세, 현재 위치 전달
    vision_snapshot: Optional[str] = None  # 시각적 인식 정보, 한줄 요약
    notes: Optional[str] = None  # 시스템이 추가로 기록하는 메모나 정보
    Battery: Optional[float] = None  # 배터리 잔량 정보 (예: 75.5)
    # system-2가 해당 정보를 바탕으로 명령 또는 판단하는데 사용
    