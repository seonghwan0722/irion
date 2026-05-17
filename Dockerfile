# 1. 베이스 이미지 설정
FROM ros:humble

# 2. 필수 패키지 설치
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-desktop \
    && rm -rf /var/lib/apt/lists/*

# 3. ROS 2 워크스페이스 설정
WORKDIR /ros2_ws
COPY ./src ./src

# 4. 의존성 설치 및 빌드
RUN . /opt/ros/humble/setup.sh && \
    python3-colcon-common-extensions \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install

# 4. 환경 설정 자동 로드 (처음 빌드 시에는 src가 비어있을 수 있으므로 빌드 명령은 일단 생략하거나 주석 처리)
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc