# ats_system1/vision.py
import json
import time
from typing import Dict, Any, List, Optional

class VisionCache:
    """
    표준화된 비전 상태 저장소
    단순 버퍼가 아니라 현재 프레임에서 감지된 객체 상태를 정규화해 저장하는 중앙 저장소
    """
    def __init__(self, frame_w_default: int, frame_h_default: int):
        self._frame_w_default = frame_w_default
        self._frame_h_default = frame_h_default
        self.data: Dict[str, Any] = {
            "targets": [],
            "primary_id": None,
            "lost_sec": 999.0,
            "frame_w": frame_w_default,
            "frame_h": frame_h_default,
            "last_update": 0.0
        }

    def update_from_msg(self, json_data: str, logger=None):
        try:
            raw = json.loads(json_data)
            norm = self._normalize_raw_vision(raw)
            self.data.update(norm)
            self.data["last_update"] = time.time()
        except Exception as e:
            if logger:
                logger.warn(f"[VisionCache] update failed: {e}")

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
        }

    def snapshot(self) -> Dict[str, Any]:
        return self.data.copy()
