# Spot+ATS System Control

이 프로젝트는 Spot 로봇과 ATS(Gimbal) 시스템을 위한 2계층 제어 아키텍처(System-1 & System-2)를 구현합니다.

## 패키지 구성 (Packages)

1.  **system_msgs**: 로봇 제어를 위한 커스텀 메시지 (`PlanCommand.msg`)
2.  **ats_system1**: System-1에서 사용하는 핵심 유틸리티 라이브러리 (비전 캐시, 추적 제어기, 내비게이션 래퍼 등)
3.  **system1**: 실시간 단위 액션(Atomic Actions)을 수행하는 실행기 노드 (`executor`)
4.  **system2**: LLM(GPT-4o)을 사용하여 고수준 계획을 생성하는 플래너 노드 (`planner`)

## 주요 기능 및 사용법 (Key Functions & Usage)

### System-1 (Executor)
- **역할**: System-2로부터 하달된 JSON 계획을 검증하고, 로봇의 하드웨어를 직접 제어합니다.
- **주요 태스크**:
    - `move_to`: Nav2를 사용하여 특정 좌표로 이동합니다.
    - `scan`: 주변을 지그재그로 탐색하며 목표를 찾습니다.
    - `track`: 짐벌과 로봇 몸체를 타겟에 정렬하고 거리를 유지합니다.
    - `report_and_wait`: 현재 상황을 보고하고 운용자의 승인을 대기합니다.

### System-2 (Planner)
- **역할**: 운용자의 자연어 명령(예: "A구역 정찰해줘")을 이해하고, 로봇이 수행할 단계별 계획을 생성합니다.
- **작동 방식**:
    - 터미널에 명령어를 입력하면 LLM이 `HighLevelPlan` 스키마에 맞는 JSON을 생성합니다.
    - 생성된 계획은 `system_msgs/PlanCommand`를 통해 System-1로 전송됩니다.

## 빌드 및 실행 방법 (How to Build & Run)

### 1. 빌드 (Build)
```bash
cd ~/ros2_ws
colcon build --packages-select system_msgs ats_system1 system1 system2
source install/setup.bash
```

### 2. 실행 (Run)
**터미널 1 (System-1 Executor):**
```bash
ros2 run system1 executor
```

**터미널 2 (System-2 Planner):**
```bash
# OpenAI API 키 설정이 필요합니다.
export OPENAI_API_KEY='your-api-key-here'
ros2 run system2 planner
```

## 코드 주석 및 가이드 (Annotations)
각 소스 코드 파일(`system1_executor_node.py`, `system2_node.py`, `llm_planner.py` 등)에는 함수의 역할과 사용법에 대한 상세한 한글 주석이 포함되어 있습니다.
