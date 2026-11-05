"""후보 중복·진행 재개·PPT 번호 혼동을 막는 합성 상태 검증."""

import copy

from candidate_review import CandidateReviewer
from experiment_support import EXPERIMENT_ROOT, write_json
from test_deadlock_proof import model, snapshot


def main():
    graph, regions = model([("a", "b"), ("b", "c"), ("c", "d"), ("d", "a")],
        {"고리_0": {"nodes": ["a", "b", "c", "d"], "cap": 4},
         "베이_1": {"nodes": ["a", "b", "c", "d"], "cap": None}})
    current = snapshot(graph, {"a": "b", "b": "c", "c": "d", "d": "a"}, now=700)
    current["wait_dependencies"] = [{"oht_id": number, "cause": "node_lock", "resource_name": None,
        "blocking_vehicle_ids": [(number + 1) % 4], "details": {"node": vehicle["_stall_next"]}}
        for number, vehicle in enumerate(current["vehicles"])]
    previous = copy.deepcopy(current)
    previous["simulation_second"] = 580
    for vehicle in previous["vehicles"]:
        vehicle["waiting_duration_seconds"] -= 120
    reviewer = CandidateReviewer(graph, regions)
    candidates = reviewer.review(current, previous)
    checks = []
    assert len(candidates) == 1 and candidates[0]["waiting_vehicle_ids"] == [0, 1, 2, 3]
    checks.append("겹치는 고리·베이에서 같은 차량 집단을 한 후보로 묶음")
    assert candidates[0]["priority"] == 1 and len(candidates[0]["corroborated_stationary_vehicle_ids"]) == 4
    checks.append("장기 대기와 120초 전 실제 진행 정지 대조 후 1순위 선택")
    assert candidates[0]["probability"] is None and "미확정" in candidates[0]["verdict"]
    checks.append("검토 우선순위를 교착 발생 확률·교착 확정으로 표시하지 않음")
    assert {item["type"] for item in candidates[0]["ppt_type_evidence"]} == {"TYPE I"}
    checks.append("단일 고리 순환도 부족한 근거를 남긴 유형 후보로만 표시")
    moving_previous = copy.deepcopy(previous)
    for vehicle in moving_previous["vehicles"]:
        vehicle["total_distance"] -= 100
    assert reviewer.review(current, moving_previous)[0]["priority"] != 1
    checks.append("같은 구간에 있어도 실제 누적 이동거리가 달라지면 진행 정지 1순위 제외")
    brief = copy.deepcopy(current)
    for vehicle in brief["vehicles"]:
        vehicle["waiting_duration_seconds"] = 10
    assert not reviewer.review(brief, previous)
    checks.append("오래 존재한 구간이라도 현재 차량 대기가 짧으면 장기 대기 후보로 오인하지 않음")
    full_exit = copy.deepcopy(current)
    full_exit["wait_dependencies"].extend([
        {"oht_id": 0, "cause": "ring_capacity", "resource_name": "ring:0", "blocking_vehicle_ids": [1, 2, 3], "details": {"cap": 4}},
        {"oht_id": 0, "cause": "merge_permission", "resource_name": "merge:b", "blocking_vehicle_ids": [1], "details": {"outgoing_segment_full": True}}])
    candidate = reviewer.review(full_exit, previous)[0]
    types = {item["type"] for item in candidate["ppt_type_evidence"]}
    assert "TYPE IV" in types and "TYPE V" not in types
    assert types <= {"TYPE I", "TYPE II", "TYPE III", "TYPE IV"}
    checks.append("10가지 시나리오 자료의 출구 차단 TYPE IV 사용·5~10번 유형 제외")
    assert all(item["missing"] and item["support_level"] == "부분 근거" for item in candidate["ppt_type_evidence"])
    checks.append("유형 근거와 검증 미완료 항목을 함께 보존")
    print("후보 우선 검토 검증 통과", len(checks), "항목", flush=True)


if __name__ == "__main__":
    main()
