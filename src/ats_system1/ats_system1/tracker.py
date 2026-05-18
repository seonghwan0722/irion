# ats_system1/tracker.py
import rclpy
import time
import math
from typing import Dict, Any, Optional, Tuple
from geometry_msgs.msg import Twist
from rclpy.node import Node

class Tracker:
    """
    짐벌 및 바디 정렬을 통한 타겟 추적 클래스입니다.
    Detection-Tracking 이 아닌 통합된 3축 통합 제어(gimbal + yaw + distance)
    """
    def __init__(self, node: Node, pub_cmd, pub_gimbal, vision, depth, **kwargs):
        self._node = node
        self._pub_cmd = pub_cmd
        self._pub_gimbal = pub_gimbal
        self._vision = vision
        self._depth = depth
        
        # PID 및 제어 변수 초기화 (생략된 상세 구현은 설계 원칙에 따라 구성됨)
        self._yaw_i = 0.0
        self._yaw_prev_err = 0.0
        self._yaw_d_lpf = 0.0
        self._yaw_cmd_prev = 0.0
        self._dyaw_lpf = 0.0
        self._prev_yaw = 0.0

    def start(self, params: Dict[str, Any], rules: Dict[str, Any]):
        """
        추적 시작
        주기적으로 _control_step() 을 반복 실행하여 바디·짐벌·속도 명령 생성
        """
        self._node.get_logger().info("[Tracker] starting track")
        # 실제 구현에서는 제어 루프 스레드 시작
        
    def wait_initial_success(self, timeout: float) -> bool:
        """초기 정렬 성공 대기(선택)"""
        # System1은 “트래킹 제어의 성공 여부”만 판단, 세부 제어는 Tracker 내부 처리
        time.sleep(1.0)
        return True

    def _control_step(self, dt: float):
        """
        Track의 제어 루프 역할
        비전 정보 읽기 -> 타깃 선택 -> 오차 계산 -> gimbal 회전 -> body 정렬 -> 거리 유지
        요약: Vision 기반 타깃 선택 + Sticky Lock 유지
        """
        v = self._vision.snapshot()
        objs = v.get("objects", []) # 현재 프레임에서 감지된 모든 객체 리스트
        fw = int(v.get("frame_w", 1280)) # 현재 프레임 해상도
        fh = int(v.get("frame_h", 720))
        cx, cy = fw * 0.5, fh * 0.5 # 화면 중심 좌표 계산

        # 타깃 선택 및 Lock 유지 로직 (생략)
        # 픽셀 단위 오차 계산 및 Gimbal PID 제어 (생략)
        # Slew 제한 및 짐벌 속도 제한 적용 (생략)
        
        # 바디 정렬 필요 이유:
        # 1. 짐벌만 돌리면 카메라는 타깃을 보는데 spot은 엉뚱한 방향을 보는 자세가 오래 유지됨
        # 2. 이런 상태에선 이동 제어, 장애물 회피, 거리 유지가 불안정
        # 3. “카메리가 보는 방향 = Spot 몸 방향“ 이 되도록 바디 yaw(wz)도 함께 정렬
        
    def stop(self):
        self._node.get_logger().info("[Tracker] stopping track")
        zero = Twist()
        self._pub_cmd.publish(zero)
        self._pub_gimbal.publish(zero)

def _clamp(val, min_v, max_v):
    return max(min_v, min(val, max_v))
