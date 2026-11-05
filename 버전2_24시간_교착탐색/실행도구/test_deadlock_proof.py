"""교착 충분조건·네 유형 분류 및 오인 방지의 합성 상태 검증. 실제 실험 결과 아님."""

import copy
import gzip
import json
from pathlib import Path
import tempfile

from deadlock_proof import DeadlockProofInspector
from experiment_support import EXPERIMENT_ROOT, write_json
from key_event_image_policy import select_key_snapshot
from monitor_confirmed_deadlocks import ExperimentMonitor


def model(edges, rings=None):
    nodes = sorted({node for edge in edges for node in edge})
    return {"nodes": {node: [index * 2000.0, (index % 2) * 2000.0] for index, node in enumerate(nodes)},
            "edges": [list(edge) for edge in edges],
            "links": [{"a": first, "b": second, "length": 2000.0} for first, second in edges]}, rings or {}


def snapshot(graph, next_nodes, now=100):
    vehicles = []
    for number, (node, next_node) in enumerate(next_nodes.items()):
        target = next_node
        x, y = graph["nodes"][node]
        vehicles.append({"oht_id": number, "node": node, "state": "to_dest", "job_state": "carrying",
            "job_id": number, "to_node": target, "_stall_next": next_node, "_stuck_since": 1,
            "_cur_speed": 0, "waiting_duration_seconds": now - 1, "_ring_waiting": False,
            "_merge_wait": None, "position_x_millimeters": x, "position_y_millimeters": y,
            "remaining_path": [node, next_node], "remaining_path_is_complete": True,
            "path_target_node": target, "total_distance": 0, "_seg_start_t": 0,
            "_seg_profile": {"dur": 1}})
    return {"simulation_second": now, "vehicles": vehicles,
            "NODE_HOLDER": {vehicle["node"]: vehicle["oht_id"] for vehicle in vehicles},
            "wait_dependencies": [], "regions": {}, "MERGE_HOLDER": {}, "MERGE_EXPIRE": {},
            "MERGE_RESERVERS": {}, "MERGE_ADMITTED": {}, "RING_MEMBERS": [], "counters": {}}


def type_names(proofs):
    return {item["type"] for proof in proofs for item in proof["confirmed_types"]}


def main():
    temporary = tempfile.TemporaryDirectory(prefix="oht-proof-test-")
    checks = []
    graph, regions = model([("a", "b"), ("b", "c"), ("c", "d"), ("d", "a")])
    state = snapshot(graph, {"a": "b", "b": "c", "c": "d", "d": "a"})
    proofs = DeadlockProofInspector(graph, regions).inspect(state)
    assert proofs and "TYPE I" in type_names(proofs)
    checks.append("TYPE I: 닫힌 단일 순환과 이미지 입력 근거 생성")

    edges = [("a", "b"), ("b", "a"), ("c", "d"), ("d", "c"), ("a", "c"), ("c", "a")]
    graph_two, regions_two = model(edges, {"고리_1": {"nodes": ["a", "b"], "cap": 2},
                                         "고리_2": {"nodes": ["c", "d"], "cap": 2}})
    state_two = snapshot(graph_two, {"a": "c", "b": "a", "c": "a", "d": "c"})
    assert "TYPE II" in type_names(DeadlockProofInspector(graph_two, regions_two).inspect(state_two))
    checks.append("TYPE II: 별개 두 고리의 실제 상호 의존")

    graph_three, regions_three = model([("a", "b"), ("b", "c"), ("c", "d"), ("d", "a"),
                                        ("x", "b"), ("y", "d")])
    state_three = snapshot(graph_three, {"a": "b", "b": "c", "c": "d", "d": "a"})
    state_three["vehicles"][0]["_merge_wait"] = ["b", 0]
    state_three["vehicles"][2]["_merge_wait"] = ["d", 2]
    assert "TYPE III" in type_names(DeadlockProofInspector(graph_three, regions_three).inspect(state_three))
    checks.append("TYPE III: 두 합류부의 대기 순환")

    inspector = DeadlockProofInspector(graph_three, regions_three)
    earlier = copy.deepcopy(state_three)
    earlier["simulation_second"] = 20
    # b의 출구 c는 먼저 막혀 있고, 아직 a·d는 움직일 수 있는 상태.
    earlier["vehicles"][3]["_stall_next"] = None
    earlier["vehicles"][3]["_stuck_since"] = None
    earlier["vehicles"][0]["_stall_next"] = None
    earlier["vehicles"][0]["_stuck_since"] = None
    inspector.inspect(earlier)
    assert "TYPE IV" in type_names(inspector.inspect(state_three))
    checks.append("TYPE IV: 선행 출구 차단·상류 대기 증가·출구로 돌아오는 폐쇄 의존")
    assert "TYPE IV" not in type_names(DeadlockProofInspector(graph_three, regions_three).inspect(state_three))
    checks.append("TYPE IV: 시간 순서 증거 없으면 유형 확정 금지")

    # 분기점에 빈 대체 출구가 있으면 현재 경로 순환만으로 확정 금지.
    escape_graph, escape_regions = model([("a", "b"), ("b", "a"), ("a", "free"), ("b", "free"), ("free", "a")])
    escape_state = snapshot(escape_graph, {"a": "b", "b": "a"})
    escape_inspector = DeadlockProofInspector(escape_graph, escape_regions)
    assert not escape_inspector.inspect(escape_state)
    later_escape = copy.deepcopy(escape_state);later_escape["simulation_second"] = 10000
    assert not escape_inspector.inspect(later_escape)
    checks.append("빈 대체 출구가 있는 분기점 순환은 10,000초 지속해도 자동 확정 금지")
    recovering = copy.deepcopy(state)
    recovering["vehicles"][0]["_seg_start_t"] = 99
    recovering["vehicles"][0]["_seg_profile"] = {"dur": 10}
    assert not DeadlockProofInspector(graph, regions).inspect(recovering)
    checks.append("이동 중 차량이 점유한 노드는 영구 차단으로 간주하지 않음")
    permission_only = copy.deepcopy(escape_state)
    permission_only["NODE_HOLDER"] = {"a": 0, "b": 1}
    permission_only["vehicles"][0]["_stall_next"] = "free"
    permission_only["vehicles"][1]["_stall_next"] = "free"
    assert not DeadlockProofInspector(escape_graph, {}).inspect(permission_only)
    checks.append("허가·예약·용량 대기만으로 확정 금지")

    # 실제 DIST 기준은 1,150mm 미만의 경로 노드만 차단 근거로 인정.
    distance_graph, _ = model([("a", "empty"), ("empty", "b"), ("b", "a")])
    distance_state = snapshot(distance_graph, {"a": "empty", "b": "a"})
    distance_state["vehicles"][0]["remaining_path"] = ["a", "empty", "b"]
    distance_state["vehicles"][0]["to_node"] = "b"
    dependency_inspector = DeadlockProofInspector(distance_graph, {})
    assert not dependency_inspector.physical_dependencies(distance_state)[0]
    dependency_inspector.edge_lengths[("empty", "b")] = 1000
    assert dependency_inspector.physical_dependencies(distance_state)[0][0]["cause"] == "follow_distance"
    dependency_inspector.edge_lengths[("empty", "b")] = 1150
    assert not dependency_inspector.physical_dependencies(distance_state)[0]
    checks.append("DIST 1,000mm 차단·1,150mm 경계 비차단 확인")

    route_graph, _ = model([("a", "j"), ("j", "b"), ("j", "escape"),
                            ("b", "k"), ("k", "a"), ("k", "escape"), ("escape", "a")])
    route_graph["nodes"].update(a=[0, 0], j=[1000, 0], b=[1000, 1000], k=[0, 1000], escape=[2000, 500])
    for link in route_graph["links"]:
        if (link["a"], link["b"]) in (("a", "j"), ("j", "b"), ("b", "k"), ("k", "a")):
            link["length"] = 1000
    route_state = snapshot(route_graph, {"a": "j", "b": "k"})
    for vehicle, path in zip(route_state["vehicles"], [["a", "j", "b"], ["b", "k", "a"]]):
        vehicle["remaining_path"] = path
        vehicle["to_node"] = vehicle["path_target_node"] = path[-1]
    route_inspector = DeadlockProofInspector(route_graph, {})
    assert not route_inspector.inspect(route_state)
    route_later = copy.deepcopy(route_state);route_later["simulation_second"] += 10
    route_proofs = route_inspector.inspect(route_later)
    assert route_proofs and "재할당 불가" in route_proofs[0]["proof_kind"]
    checks.append("비분기·운반 중 고정 경로의 빈 노드 너머 거리 차단 순환 확인")
    incomplete = copy.deepcopy(route_later);incomplete["simulation_second"] += 10
    incomplete["vehicles"][0]["remaining_path_is_complete"] = False
    assert not route_inspector.inspect(incomplete)
    checks.append("고정 경로 증거가 불완전하면 교착 확정 금지")

    assert ExperimentMonitor.contradicting_vehicle(proofs[0], state) is None
    moved_later = copy.deepcopy(state);moved_later["simulation_second"] += 1
    moved_later["vehicles"][0]["total_distance"] += 1
    assert ExperimentMonitor.contradicting_vehicle(proofs[0], moved_later) is not None
    checks.append("과거 충분조건과 이후 이동 기록이 모순되면 확정 전 반례 검출")

    image_directory = Path(temporary.name) / "핵심이미지선별검증"
    image_directory.mkdir(exist_ok=True)
    references = []
    ring_regions = {"고리_0": {"nodes": ["a", "b", "c", "d"], "cap": 4}}
    for second in (0, 60, 120):
        image_state = copy.deepcopy(state);image_state["simulation_second"] = second
        image_state["regions"] = {"고리_0": {"controlled_capacity_utilization": 1.0, "waiting_vehicle_count": 4}}
        name = f"시각_{second:010.1f}초.json.gz"
        with gzip.open(image_directory / name, "wt", encoding="utf-8") as handle:json.dump(image_state, handle)
        references.append(name)
    image_event = {"label": "제어용량포화", "last_observed_second": 120, "region_names": ["고리_0"],
        "evidence": {"cap": 4, "maximum_waiting_duration_seconds": 240, "waiting_vehicle_count": 4}}
    assert select_key_snapshot(image_directory, image_event, references, ring_regions)[0] == references[-1]
    image_event["evidence"]["maximum_waiting_duration_seconds"] = 20
    assert select_key_snapshot(image_directory, image_event, references, ring_regions)[0] is None
    checks.append("핵심 사진: 포화·집단 진행 정지 120초 선택, 짧은 대기는 사진 제외")

    # 실제 확정 저장 경로를 실행해 판정·자립 상태 파일·전체/확대 이미지 확인.
    output = Path(temporary.name) / "확정교착저장_합성상태검증"
    output.mkdir(parents=True, exist_ok=True)
    monitor = object.__new__(ExperimentMonitor)
    monitor.directory = output
    monitor.output = output / "확정교착기록";monitor.output.mkdir(exist_ok=True)
    monitor.graph = graph;monitor.confirmed = {};monitor.disabled = False
    if not list(monitor.output.glob("확정*")):
        monitor.save_confirmation(state, proofs[0], output / "검증상태.json.gz", historical=False)
    certificate = next(monitor.output.glob("확정*/판정근거.json"))
    assert (certificate.parent / "전체시뮬레이션.png").exists()
    assert (certificate.parent / "교착구간확대.png").exists()
    with gzip.open(certificate.parent / "확정시점상태.json.gz", "rt", encoding="utf-8") as handle:
        assert json.load(handle)["simulation_second"] == state["simulation_second"]
    checks.append("확정 시점 전체 상태·전체 지도·확대 지도·근거 자동 저장")

    print("검증 통과", len(checks), "항목", flush=True)
    temporary.cleanup()


if __name__ == "__main__":
    main()
