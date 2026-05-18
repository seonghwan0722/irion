# system2/system2/llm_planner.py
# chain=prompt|llm|parser로 모호한 자연어를 검증된 파이썬 객체로 반환
import json
import os
from typing import List, Optional, Dict, Any
from langchain_openai import ChatOpenAI 
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.output_parsers import PydanticOutputParser

from .models import HighLevelPlan, SystemState

# LLM 설정 (환경 변수에 따라 OpenAI 또는 로컬 LLM 선택)
# USE_LOCAL_LLM=true 설정 시 로컬 모델 사용
# LOCAL_LLM_TYPE: 'ollama', 'openai-compatible', 'llamacpp' 중 선택 (기본 ollama)
USE_LOCAL = os.getenv("USE_LOCAL_LLM", "false").lower() == "true"
LOCAL_LLM_TYPE = os.getenv("LOCAL_LLM_TYPE", "ollama").lower()
LOCAL_MODEL = os.getenv("LOCAL_MODEL_NAME", "llama3") # Ollama/OpenAI용 모델명
LOCAL_MODEL_PATH = os.getenv("LOCAL_MODEL_PATH", "models/llama-3-8b.Q4_K_M.gguf") # LlamaCpp용 경로
LOCAL_API_BASE = os.getenv("LOCAL_API_BASE", "http://localhost:11434/v1") # 로컬 서버 주소

if USE_LOCAL:
    if LOCAL_LLM_TYPE == "ollama":
        # 1. Ollama를 사용하는 경우 (추천)
        # pip install langchain-ollama
        from langchain_ollama import ChatOllama
        llm = ChatOllama(
            model=LOCAL_MODEL,
            temperature=0.1,
        )
    elif LOCAL_LLM_TYPE == "llamacpp":
        # 2. LlamaCpp를 직접 사용하는 경우 (GGUF 모델 파일 필요)
        # pip install llama-cpp-python langchain-community
        from langchain_community.llms import LlamaCpp
        llm = LlamaCpp(
            model_path=LOCAL_MODEL_PATH,
            temperature=0.1,
            max_tokens=2048,
            n_ctx=4096,
            verbose=False,
        )
    else:
        # 3. OpenAI 호환 로컬 서버를 사용하는 경우 (LM Studio, vLLM, LocalAI 등)
        llm = ChatOpenAI(
            model=LOCAL_MODEL,
            base_url=LOCAL_API_BASE,
            api_key="not-needed",
            temperature=0.1,
        )
else:
    # 4. 기본 OpenAI API 사용
    llm = ChatOpenAI(
        model="gpt-4o",
        temperature=0.1,
    )

parser = PydanticOutputParser(pydantic_object=HighLevelPlan)
# LLM을 제어하기 위한 LangChain의 강력한 유틸리리
    # 포맷 지시문 자동 생성 parser.get_format_instructions() -> LLM에게 "답변은 HighLevelPlan 모델 형식"이라는 상세 규격 텍스트 자동 생성
    # 검증 및 객체화
        # 파서는 이 LLM 문자열을 Json으로 파싱하고, HighLevelPlan 스키마에 맞는지 유효성 검사 수행

system_template = """
너는 Spot+ATS 시스템의 고수준 디션 플래너(System-2)이다.

역할:
- 운용자의 자연어 명령과 System-1의 현재 상태를 바탕으로,
- System-1이 바로 실행할 수 있는 고수준 미션 계획(HighLevelPlan)을 설계한다.
- 이 계획은 여러 개의 단위 액션(step)으로 구성되며, System-1 Executor가 순차적으로 수행한다.

단위 액션(task)은 다음 5가지만 사용할 수 있다.
- move_to        : 특정 위치/구역으로 이동
- scan           : 주변을 탐색/정찰
- track          : 특정 타겟(예: 사람)을 추적
- report_and_wait: 상황을 보고하고 추가 지시를 기다림
- return_to_home : 사전에 정의된 홈 위치로 복귀

지도는 다음 네 구역으로 나뉜다. (좌표 단위: map 프레임, 단위 m)
구역 A: x_min <= x < x_c, y_min <= y < y_c
구역 B: x_c <= x <= x_max, y_min <= y < y_c
구역 C: x_min <= x < x_c, y_c <= y <= y_max
구역 D: x_c <= x <= x_max, y_c <= y <= y_max

여기서 (x_min, y_min) = (-10.7, -12.6),
(x_max, y_max) = (9.8, 18.3),
x_c = -0.45, y_c = 2.85 이다.
운용자가 "A 구역으로 이동해"라고 말하면,
해당 구역의 중앙 근처 좌표를 목표 지점으로 설정하라.
예: A 구역 중앙 = ((x_min + x_c)/2, (y_min + y_c)/2) = (-5.575, -7.725)

중요 규칙:
- 출력은 반드시 HighLevelPlan 스키마를 따르는 JSON 객체 하나여야 한다.
- 스키마의 상세 형식과 필드는 아래 {format_instructions} 내용을 정확히 따른다.
- 각 step의 params에는 System-1이 이해할 수 있는 수준의 필드만 넣는다
  (예: move_to는 좌표, scan은 sweep_deg / yaw_rate_dps / watch_classes 등).
- System-1 상태(state_text)에는 미션 ID, 현재 task, step_index, 위치, 시각 정보
  제약조건(위험 구역, 승인 필요 구역, 배터리 상황 등)이 들어올 수 있으며,
  이 정보를 반드시 반영해서 합리적인 플랜을 만든다.
- 추가 컨텍스트(extra_context_text)에는 report_and_wait 등에서 넘어온 pose/vision/state_string 등의
  상세 상황이 요약되어 있을 수 있으며, 이 정보도 함께 고려하여 계획을 설계한다.


state_text 해석 원칙:
- "상태 정보 없음" 또는 초기 상태인 경우 -> 새로운 미션의 초깃 플랜을 설계한다.
- mission_id, current_task, step_index 등이 있는 경우 ->
  진행 중인 미션의 남은 부분을 재계획하거나, 상황 변화(운용자 추가 명령, 타겟 상실 등)를 반영한 후속 플랜을 생성한다.
  필요하면 이전 플랜의 나머지 단계를 단순 반복하기보다, 현재 위치/상황 기준으로 최적화하여 다시 설계해도 된다.

제약조건 반영:
- state_text나 운용자 명령에 "위험 구역, 출입 제한, 승인 필요, 배터리 부족, 안전 거리 유지" 등의 제약이 언급되면,
  해당 제약을 어기지 않는 방향으로 steps를 구성해야 한다.
  예: 승인 필요 구역 -> report_and_wait를 통해 승인 요청 후 진입, 배터리 부족 -> return_to_home 포함 등.

출력 형식:
- 반드시 유효한 JSON만 출력한다.
- JSON의 앞뒤에 설명 문장, 코멘트, 마크다운, 자연어 텍스트를 절대 추가하지 않는다.
"""

user_template = """
[현재 System-1 상태 요약]
{state_text}

[추가 컨텍스트]
{extra_text}

[운용자 명령]
{user_command}

위의 상태와 명령을 모두 고려하여,
Spot+ATS가 수행해야 할 HighLevelPlan JSON을 설계하라.
"""

prompt = ChatPromptTemplate.from_messages([
    ("system", system_template),
    ("user", user_template),
])


# 데이터 변환함수: 시스템을 효율적으로 다루기 위한'통역사'와 '필터'
    # 불필요한 데이터를 필터링하여 토큰 낭비 및 정보의 과부화 방지


def state_to_text(state: Optional[SystemState]) -> str: # 신호 대 잡음비 개선
    if state is None:
        return "상태 정보 없음 (초기 계획으로 간주)."

    parts = []
    if state.mission_id:
        parts.append(f"- mission_id: {state.mission_id}")
    if state.system1_state:
        parts.append(f"- System-1 상태(queue): {state.system1_state}")
    if state.current_task is not None:
        parts.append(f"- 현재 task: {state.current_task} (step_index={state.step_index})")
    if state.pose:
        x = state.pose.get("x")
        y = state.pose.get("y")
        yaw = state.pose.get("yaw") # original code used yaw_deg, but pose in models is Dict[str, float]
        if x is not None and y is not None and yaw is not None:
            parts.append(f"- 현재 위치: x={x:.2f}, y={y:.2f}, yaw={yaw:.1f}deg") # AI가 판단하는데 필요한 소수점 둘째 자리까지 확보
    if state.vision_snapshot and state.vision_snapshot.summary:
        parts.append(f"- 시각 정보: {state.vision_snapshot.summary}")
    if state.notes:
        parts.append(f"- 비고: {state.notes}")

    return "\n".join(parts) if parts else "요약 가능한 상태 정보 없음."

# 비정형 데이터의 요약
def extra_context_text(extra_context: Optional[Dict[str, Any]]) -> str:
    """
    report_context 등에서 넘어온 pose/vision/state_string 등
    LLM이 이해하기 쉬운 형태 요약 텍스트로 전환.
    """
    if not extra_context:
        return "추가 컨텍스트 없음."

    lines = []

    pose = extra_context.get("pose") or {}
    if pose:
        x = pose.get("x")
        y = pose.get("y")
        yaw = pose.get("yaw")
        ok = pose.get("ok", False)
        try:
            if ok:
                lines.append(f"- pose: map 기준 (x={float(x):.2f}, y={float(y):.2f}, yaw={float(yaw):.2f}rad)")
                # linse.append 센서가 뿜어내는 복잡한 JSON에서 Primary Target, 클래스, 거리만 뽑아서 한줄로 요약
                    # 사람이 브리핑해주는 듯한 직관적인 보고서 형태로 LLM에 전달
            else:
                lines.append("- pose: map 기준 위치 불명 (값 존재)")
        except Exception:
            # 숫자 캐스팅 실패 시에도 최소한 정보는 남겨둔다.
            lines.append(f"- pose(raw): {pose}")

    vision = extra_context.get("vision") or {}
    targets = vision.get("targets") or []
    primary_id = vision.get("primary_id")
    if targets:
        t0 = targets[0]
        cls = t0.get("class", "object")
        rng = t0.get("range_m")
        if rng is not None:
            try:
                lines.append(f"- vision: 주요 타겟 id={primary_id}, class={cls}, range={float(rng):.1f}m")
            except Exception:
                lines.append(f"- vision: 주요 타겟 id={primary_id}, class={cls}")
        else:
            lines.append(f"- vision: 주요 타겟 id={primary_id}, class={cls}")
        lines.append(f"- vision: 감지된 타겟 수={len(targets)}")
    else:
        if vision:
            lines.append("- vision: 감지된 타겟 없음 (vision 정보는 존재)")
        else:
            lines.append("- vision: 정보 없음")

    state_str = extra_context.get("state_string")
    if state_str:
        lines.append(f"- state_string: {state_str}")

    other_keys = [k for k in extra_context.keys() if k not in ["pose", "vision", "state_string"]]
    if other_keys:
        lines.append(f"- 기타 컨텍스트 키들: {', '.join(other_keys)}")

    return "\n".join(lines) if lines else "추가 컨텍스트 없음."

def build_plan(
    user_command: str,
    system1_state: Optional[SystemState],
    extra_context: Optional[Dict[str, Any]] = None,
) -> HighLevelPlan:

    state_text = state_to_text(system1_state)
    extra_text = extra_context_text(extra_context)
    format_instructions = parser.get_format_instructions() # -> HighLevelPlan JSON 스키마 지시문 자동 생성
    chain = prompt | llm | parser # 프롬프트 채우기 -> LLM 호출 -> 결과 파싱까지 하나의 파이프라인으로 연결

    plan: HighLevelPlan = chain.invoke( 
        { # 운용자 명령 + 상태 요약 등을 AI(LLM)에게 전달 -> AI(LLM)는 검증된 HighLevelPlan 객체로 반환
            "user_command": user_command,
            "state_text": state_text,
            "extra_text": extra_text,
            "format_instructions": format_instructions,
        }
    )
    return plan
