"""실제 발생 결과와 구분하는 합성 상태 검증: 네 유형·회복·핵심/선행 상태 저장."""

from collections import deque
import copy
import gzip
import json
from pathlib import Path
import tempfile

from experiment_support import EXPERIMENT_ROOT, current_timestamp, file_sha256, write_json
from integrated_deadlock_observer import CandidateTracker, IntegratedDeadlockRecorder, PROTOCOL_VERSION
from test_deadlock_proof import model, snapshot


def add_dependencies(state):
    state["wait_dependencies"] = [{"oht_id": vehicle["oht_id"], "cause": "node_lock",
        "blocking_vehicle_ids": [state["NODE_HOLDER"][vehicle["_stall_next"]]],
        "resource_name": None, "details": {"node": vehicle["_stall_next"]}}
        for vehicle in state["vehicles"] if vehicle.get("_stall_next") in state["NODE_HOLDER"]]
    return state


def confirmed_types(actions):
    return {item["type"] for kind, proof in actions if kind == "확정교착" for item in proof["confirmed_types"]}


def main():
    checks = []
    graph, regions = model([("a", "b"), ("b", "c"), ("c", "d"), ("d", "a")])
    first = add_dependencies(snapshot(graph, {"a": "b", "b": "c", "c": "d", "d": "a"}))
    tracker = CandidateTracker(graph, regions)
    first_actions = tracker.update(first)
    assert "TYPE I" in confirmed_types(first_actions)
    checks.append("TYPE I: 실제 노드 경로가 단일 루프인 닫힌 점유 구조 검출")
    second_graph, second_regions = model([("a", "b"), ("b", "a"), ("c", "d"), ("d", "c"), ("a", "c"), ("c", "a")],
        {"고리_1": {"nodes": ["a", "b"], "cap": 2}, "고리_2": {"nodes": ["c", "d"], "cap": 2}})
    second = add_dependencies(snapshot(second_graph, {"a": "c", "b": "a", "c": "a", "d": "c"}))
    assert "TYPE II" in confirmed_types(CandidateTracker(second_graph, second_regions).update(second))
    checks.append("TYPE II: 별개 루프의 차량이 서로 막는 닫힌 구조 검출")
    third_graph, third_regions = model([("a", "b"), ("b", "c"), ("c", "d"), ("d", "a"), ("x", "b"), ("y", "d")])
    third = add_dependencies(snapshot(third_graph, {"a": "b", "b": "c", "c": "d", "d": "a"}))
    third["vehicles"][0]["_merge_wait"] = ["b", 0]
    third["vehicles"][2]["_merge_wait"] = ["d", 2]
    assert "TYPE III" in confirmed_types(CandidateTracker(third_graph, third_regions).update(third))
    checks.append("TYPE III: 서로 다른 합류부 선두의 닫힌 대기 검출")
    fourth_tracker = CandidateTracker(third_graph, third_regions)
    earlier = copy.deepcopy(third)
    earlier["simulation_second"] = 20
    for number in (0, 3):
        earlier["vehicles"][number]["_stall_next"] = None
        earlier["vehicles"][number]["_stuck_since"] = None
    add_dependencies(earlier)
    fourth_tracker.update(earlier)
    assert "TYPE IV" in confirmed_types(fourth_tracker.update(third))
    assert "TYPE IV" not in confirmed_types(CandidateTracker(third_graph, third_regions).update(third))
    checks.append("TYPE IV: 출구 차단 선행·대기열 확산·출구로 돌아오는 관계와 시간 순서 확인")

    # 각 유형의 닫힌 점유를 깨고, 모든 차량에 빈 대체 출구를 제공한다.
    for name, original_graph, original_regions, original_state in (
        ("TYPE I", graph, regions, first), ("TYPE II", second_graph, second_regions, second),
        ("TYPE III", third_graph, third_regions, third), ("TYPE IV", third_graph, third_regions, third)):
        counter_graph = copy.deepcopy(original_graph)
        counter_graph["nodes"]["free"] = [50000, 50000]
        for vehicle in original_state["vehicles"]:
            counter_graph["edges"].append([vehicle["node"], "free"])
            counter_graph["links"].append({"a": vehicle["node"], "b": "free", "length": 2000})
        counter_tracker = CandidateTracker(counter_graph, original_regions)
        for second_value in range(100, 117):
            counter = copy.deepcopy(original_state)
            counter["simulation_second"] = second_value
            assert not confirmed_types(counter_tracker.update(counter))
        recovering = copy.deepcopy(counter)
        recovering["simulation_second"] = 117
        for vehicle in recovering["vehicles"]:
            vehicle["total_distance"] += 2000
            vehicle["_stall_next"] = vehicle["_stuck_since"] = None
        recovering["wait_dependencies"] = []
        actions = counter_tracker.update(recovering)
        assert any(kind == "후보변화" and evidence["outcome"] == "원래 차량 전원 진행 재개" for kind, evidence in actions)
        checks.append(name + ": 대체 출구가 있고 이후 진행한 사례는 후보 해소로 기록·확정 금지")

    moved = copy.deepcopy(first)
    moved["simulation_second"] += 1
    moved["vehicles"][0]["total_distance"] += 1
    assert any(kind == "판정반례" for kind, _ in tracker.update(moved))
    checks.append("충분조건 판정 이후 움직임 반례가 나타나면 판정 철회")

    # 실제 저장 함수를 실행해 확정 시점과 그 이전 지도·독립 상태 파일을 검증한다.
    temporary = tempfile.TemporaryDirectory(prefix="oht-observation-test-")
    output = Path(temporary.name)
    (output / "핵심상태").mkdir(exist_ok=True)
    recorder = object.__new__(IntegratedDeadlockRecorder)
    recorder.output_directory = output
    recorder.graph = graph
    recorder.saved_times = {}
    recorder.buffer = deque()
    recorder.scene_buffer = deque()
    for second_value in (0, 180, 240):
        earlier = copy.deepcopy(first)
        earlier["simulation_second"] = second_value
        recorder.buffer.append(earlier)
        recorder.scene_buffer.append({"simulation_second": second_value, "positions": [
            (vehicle["oht_id"], vehicle["node"], vehicle["_stall_next"],
             vehicle["position_x_millimeters"], vehicle["position_y_millimeters"],
             vehicle["waiting_duration_seconds"], None, None, None, None) for vehicle in earlier["vehicles"]]})
    recorder.event_handle = (output / "시험사건.jsonl").open("w", encoding="utf-8")
    current = copy.deepcopy(first)
    current["simulation_second"] = 301
    for vehicle in current["vehicles"]:
        vehicle["_stuck_since"] = 300
        vehicle["waiting_duration_seconds"] = 1
    recorder.retain_wait_onsets(current)
    proof = next(evidence for kind, evidence in CandidateTracker(graph, regions).update(current) if kind == "확정교착")
    recorder.persist_action("확정교착", proof, current)
    recorder.event_handle.close()
    directory = output / "확정교착기록" / ("확정_" + proof["confirmation_key"][:16])
    certificate = json.loads((directory / "판정근거.json").read_text())
    assert len(certificate["before_states"]) == 3
    for reference in certificate["before_states"]:
        with gzip.open(output / reference["state_path"], "rt", encoding="utf-8") as handle:
            assert json.load(handle)["simulation_second"] == reference["actual_simulation_second"]
        assert Path(reference["image_path"]).exists()
    assert (directory / "전체시뮬레이션.png").exists() and (directory / "문제구간확대.png").exists()
    checks.append("확정 시점 전체·확대 지도, 이전 300·120·60초 전체 지도 및 독립 상태의 실제 저장/읽기 검증")
    recorder.graph = counter_graph
    recorder.event_handle = (output / "후보시험사건.jsonl").open("w", encoding="utf-8")
    candidate_tracker = CandidateTracker(counter_graph, third_regions)
    saved_candidate = None
    for second_value in range(300, 421):
        current = copy.deepcopy(third)
        current["simulation_second"] = second_value
        for vehicle in current["vehicles"]:
            vehicle["waiting_duration_seconds"] = second_value - 1
        recorder.retain_wait_onsets(current)
        for kind, evidence in candidate_tracker.update(current):
            assert kind != "확정교착"
            recorder.persist_action(kind, evidence, current)
            if kind == "핵심후보":
                saved_candidate = evidence
    recorder.event_handle.close()
    assert saved_candidate is not None
    candidate_directory = Path(saved_candidate["image_directory"])
    assert (candidate_directory / "전체시뮬레이션.png").exists()
    assert (candidate_directory / "문제구간확대.png").exists()
    assert saved_candidate["verdict"] == "후보·교착 미확정"
    checks.append("해소 선택지가 있는 장기 순환도 핵심 후보 사진·선행 상태는 남기고 교착으로 확정하지 않음")
    print(json.dumps({"passed": True, "checks": len(checks)}, ensure_ascii=False))
    temporary.cleanup()


if __name__ == "__main__":
    main()
