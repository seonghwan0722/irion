# ats_system1/depth.py
from rclpy.node import Node
from typing import Optional, Dict, Any

class DepthBuffer: # 이미지 상 bbox에서 실제 거리 뽑아내기
    """
    최신 depth 이미지를 보관하고, bbox ROI의 대표 depth(m)를 계산한다.
    - 인코딩 자동 처리(32FC1=미터, 16UC1=mm->m)
    - 프레임 크기 매핑                  # /depth 이미지를 캐시하고 bbox ROI의 대표 거리값(range_m)을 계산해 Tracker에 제공
    - 중앙부 크롭/백분위수/최소 유효비율   # ”화면 중앙”만으로는 부족 -> 사람(타깃)과 로봇 사이의 실제 거리를 알아야 vx 제어(전진/후퇴) 가능
    - 오래된 프레임 무시(max_age_sec) # 일반적으로 depth 이미지를 최신 상태로 유지, 요청 시 bbox안 타깃의 실제 거리 값 전달
    """
    def __init__(self, node: Node, topic: str = "/depth"):
        self._node = node
        self._last_depth = None
        
    def has_image(self) -> bool:
        return True # 시뮬레이션/데모용

    def roi_mean_depth(self, bbox, fw, fh, **kwargs) -> Optional[float]:
        # 실제 구현에서는 depth 이미지의 ROI 평균을 계산하여 vx계산의 밑거름으로 사용합니다.
        return 1.5 # 데모용 1.5m
