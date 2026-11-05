"""관측은 보존하고, 지속 포화·진행 정지의 핵심 시점만 이미지로 선택."""

import math
from pathlib import Path

from snapshot_storage import read_snapshot


POLICY_VERSION = 2
SATURATION_OBSERVATION_SECONDS = 120
MINIMUM_WAITING_SECONDS = 120


def snapshot_second(relative_path):
    return float(Path(relative_path).name.removeprefix("시각_").removesuffix("초.json.gz"))


def select_key_snapshot(output_directory, event, references, regions):
    """사진 선별 조건이며, 이 함수는 데드락 확정을 판정하지 않는다."""
    evidence = event.get("evidence", {})
    last_observed = event["last_observed_second"]
    if event["label"] == "순환대기후보":
        if evidence.get("stable_dependency_duration_seconds", 0) < 60:
            return None, "순환 의존 60초 관측 전: 기록만 보존"
        candidates = [name for name in references if snapshot_second(name) <= last_observed]
        if not candidates:
            return None, "검사할 상태 파일 없음"
        return candidates[-1], "60초 이상 관측된 순환 대기 후보. 교착 확정 아님"
    cap = evidence.get("cap")
    if (not cap or event["label"] != "제어용량포화" or
            evidence.get("maximum_waiting_duration_seconds", 0) < MINIMUM_WAITING_SECONDS or
            evidence.get("waiting_vehicle_count", 0) < max(2, math.ceil(cap * 0.5))):
        return None, "단순 고점유·단기 대기: 상태 기록만 보존"
    region_name = next((name for name in event["region_names"] if name.startswith("고리_")), None)
    if region_name is None or region_name not in regions:
        return None, "제어 용량을 확인할 고리 없음"
    nodes = set(regions[region_name]["nodes"])
    candidates = [name for name in references
                  if last_observed - 300 <= snapshot_second(name) <= last_observed]
    stable_since = None
    previous_positions = None
    selected = None
    previous_second = None
    for name in candidates:
        snapshot = read_snapshot(output_directory / name)
        now = snapshot["simulation_second"]
        values = snapshot["regions"].get(region_name, {})
        full = ((values.get("controlled_capacity_utilization") or 0) >= 1
                and values.get("waiting_vehicle_count", 0) >= max(2, math.ceil(cap * 0.5)))
        positions = {vehicle["oht_id"]: (vehicle["node"], vehicle["total_distance"],
                     vehicle["position_x_millimeters"], vehicle["position_y_millimeters"])
                     for vehicle in snapshot["vehicles"] if vehicle["node"] in nodes}
        if not full or not positions:
            stable_since = previous_positions = None
        elif (previous_positions != positions or previous_second is None or now - previous_second > 60):
            stable_since = now
            previous_positions = positions
        elif stable_since is not None and now - stable_since >= SATURATION_OBSERVATION_SECONDS:
            selected = name
            break
        previous_second = now
    if selected is None:
        return None, "포화·동일 차량 집단의 진행 정지 120초 관측 미충족"
    return selected, "제어 용량 100% 이상·대기 집단 진행 정지 120초 관측. 교착 확정 아님"
