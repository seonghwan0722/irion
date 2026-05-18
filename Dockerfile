# 1. 젯슨 나노(L4T) 및 ROS 2 Humble 호환 베이스 이미지 설정
# JetPack 4.6.x (Ubuntu 18.04 기반) 환경에서 ROS 2 Humble을 구동하기 위해
# dustynv의 L4T 최적화 이미지를 사용합니다.
FROM dustynv/ros:humble-desktop-l4t-r32.7.1

# 2. 쉘 환경 설정 (빌드 중 source 명령 사용을 위함)
SHELL ["/bin/bash", "-c"]

# 3. 필수 시스템 패키지 및 빌드 도구 설치
# 기존 Dockerfile의 오타를 수정하고 젯슨 환경에 필요한 유틸리티를 추가했습니다.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    usbutils \
    v4l-utils \
    libv4l-dev \
    && rm -rf /var/lib/apt/lists/*

# 4. 파이썬 의존성 설치 (YOLO 및 LLM 플래너)
# 젯슨 나노의 메모리 부족을 방지하기 위해 --no-cache-dir을 사용합니다.
# llama-cpp-python 등은 컴파일이 필요할 수 있으므로 빌드 도구가 포함된 상태에서 설치합니다.
RUN pip3 install --no-cache-dir --upgrade pip && \
    pip3 install --no-cache-dir \
    # 데이터 검증 및 유틸리티
    numpy<2 \
    pydantic \
    jsonschema \
    typing-extensions>=4.4.0 \
    # 비전 및 AI (YOLO)
    ultralytics==8.4.6 \
    lap>=0.5.12 \
    opencv-python>=4.8.1.78 \
    # LLM 및 LangChain
    langchain \
    langchain-core \
    langchain-openai \
    langchain-ollama \
    langchain-community \
    llama-cpp-python

# 5. ROS 2 워크스페이스 설정
WORKDIR /ros2_ws
# .dockerignore에서 제외된 파일을 제외한 모든 프로젝트 파일 복사
COPY . .

# 6. ROS 의존성 설치 및 빌드
# --parallel-workers 1: 젯슨 나노의 RAM 부족으로 인한 시스템 멈춤 방지
RUN source /opt/ros/humble/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install --parallel-workers 1

# 7. 환경 설정 자동 로드
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# 컨테이너 시작 시 기본 쉘 실행
CMD ["bash"]
