"""버전 2를 변경하지 않는 매초 관측·후보 추적·충분조건 검사와 핵심 상태 보관.

합성 검증은 검출 경로를 시험하며 실제 지도에서 네 유형의 발생을 보장하지 않는다.
허가·용량 의존 순환은 후보이고, 구조적 진행 불능 충분조건과 구분한다.
"""

from collections import defaultdict, deque
import copy
import gzip
import hashlib
import json
from pathlib import Path
import resource
import shutil
import time

from candidate_review import CandidateReviewer, connected_components, dependency_graph
from deadlock_proof import DeadlockProofInspector
from experiment_support import current_timestamp, write_json
from render_event_maps import draw_map
from simulation_observer import SimulationRecorder, strongly_connected_groups
from segment_deadlock_proof import SegmentDeadlockProofInspector


PROTOCOL_VERSION = "10가지자료_1~4번_최초정체기준_선행장면고정_20261106"
TARGET_TYPES = {"TYPE I", "TYPE II", "TYPE III", "TYPE IV"}


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, ensure_ascii=False,
                                    separators=(",", ":")).encode()).hexdigest()


class ConservativeProofInspector(DeadlockProofInspector):
    def inspect(self, snapshot):
        if snapshot.get("occupancy_mode") == "segment":
            if not hasattr(self, "segment_inspector"):
                self.segment_inspector = SegmentDeadlockProofInspector(self.graph, self.regions)
            return self.segment_inspector.inspect(snapshot)
        return super().inspect(snapshot)
    def fixed_loaded_route(self, vehicle, snapshot):
        if vehicle.get("_replan") or vehicle.get("_abort_leg"):
            return False
        return super().fixed_loaded_route(vehicle, snapshot)

    def classify(self, group, vehicles, dependencies, exits):
        types, regions = super().classify(group, vehicles, dependencies, exits)
        # 단순한 차량 의존 고리만으로 공간상 단일 루프라고 부르지 않는다.
        segments = defaultdict(set)
        for number in group:
            path = vehicles[number].get("remaining_path", [])
            blocked_nodes = {edge["node"] for edge in dependencies.get(number, [])
                             if edge["blocker"] in group}
            for first, second in zip(path, path[1:]):
                segments[first].add(second)
                if second in blocked_nodes:
                    break
        segment_nodes = set(segments) | {node for destinations in segments.values() for node in destinations}
        simple_loop = bool(segment_nodes) and all(len(segments[node]) == 1 for node in segment_nodes)
        simple_loop = simple_loop and any(set(component) == segment_nodes
                                        for component in strongly_connected_groups(segments))
        if not simple_loop:
            types = [item for item in types if item["type"] != "TYPE I"]
        return [item for item in types if item["type"] in TARGET_TYPES], regions


class CandidateTracker:
    """대기 관계·차량·위치를 함께 추적한다. 지역 사건 수를 교착 수로 세지 않는다."""
    def __init__(self, graph, regions):
        self.reviewer = CandidateReviewer(graph, regions)
        self.inspector = ConservativeProofInspector(graph, regions)
        self.active = {}
        self.finished = []
        self.confirmed = {}
        self.last_second = None
        self.next_number = 1
        self.inspection_count = 0
        self.refutations = []

    @staticmethod
    def progress(vehicle):
        return (vehicle["node"], vehicle["total_distance"],
                vehicle["position_x_millimeters"], vehicle["position_y_millimeters"])

    def update(self, snapshot, previous=None):
        now = snapshot["simulation_second"]
        if self.last_second is not None and now <= self.last_second:
            return []
        gap = self.last_second is not None and now - self.last_second > 1.000001
        vehicles = {vehicle["oht_id"]: vehicle for vehicle in snapshot["vehicles"]}
        actions = []
        # 확정 충분조건 이후 실제 움직임은 반드시 판정 반례로 보존한다.
        for key, record in self.confirmed.items():
            if record.get("revoked"):
                continue
            moved = [number for number, progress in record["progress"].items()
                     if self.progress(vehicles[number]) != progress]
            if moved:
                record["revoked"] = True
                evidence = {"confirmation_key": key, "simulation_second": now,
                            "moving_vehicle_ids": moved, "verdict": "확정 판정 철회·추가 분석 필요"}
                self.refutations.append(evidence)
                actions.append(("판정반례", evidence))
        proofs = self.inspector.inspect(snapshot)
        self.inspection_count += 1
        for proof in proofs:
            key = digest([(number, vehicles[number]["node"]) for number in proof["vehicle_ids"]])
            if key not in self.confirmed:
                self.confirmed[key] = {"proof": proof, "revoked": False,
                    "progress": {number: self.progress(vehicles[number]) for number in proof["vehicle_ids"]}}
                actions.append(("확정교착", {**proof, "confirmation_key": key}))

        directed, undirected = dependency_graph(snapshot)
        current = {}
        cycle_members = set()
        for component in strongly_connected_groups(directed):
            group = {int(vertex.split(":")[1]) for vertex in component if vertex.startswith("vehicle:")}
            if len(group) < 2:
                continue
            waiting = {number for number in group if vehicles[number].get("_stall_next") is not None
                       and vehicles[number].get("_stuck_since") is not None}
            if len(waiting) < 2:
                continue
            cycle_members.update(group)
            dependencies = [item for item in snapshot["wait_dependencies"] if item["oht_id"] in group]
            edges = sorted((first, second) for first in component for second in directed.get(first, ())
                           if second in component)
            key = "순환_" + digest([sorted((number, vehicles[number]["node"]) for number in group), edges])
            evidence = self.reviewer.type_evidence(group, waiting, vehicles, dependencies, directed, previous)
            current[key] = {"vehicle_ids": sorted(group), "kind": "순환 대기 후보", "ppt_type_evidence": evidence,
                "wait_dependencies": dependencies, "internal_wait_edges": edges,
                "maximum_waiting_duration_seconds": max(vehicles[number]["waiting_duration_seconds"] for number in waiting)}
        # 폐쇄 고리가 생기기 전부터, 용량 포화와 연결된 같은 차량의 장기 정지를 추적한다.
        # 포화율 자체를 교착으로 판정하지 않으며 구역별로 사건을 중복 생성하지 않는다.
        covered_members = set(cycle_members)
        for component in connected_components(undirected):
            group = {int(vertex.split(":")[1]) for vertex in component if vertex.startswith("vehicle:")}
            waiting = {number for number in group if vehicles[number]["waiting_duration_seconds"] >= 60}
            if len(waiting) < 3 or waiting & cycle_members:
                continue
            names = {name for number in waiting for name in vehicles[number].get(
                "physical_region_names", self.reviewer.node_regions[vehicles[number]["node"]])}
            saturated = [name for name in names if (
                (snapshot["regions"].get(name, {}).get("controlled_capacity_utilization") or 0) >= 1
                or snapshot["regions"].get(name, {}).get("occupied_node_count", 0) >=
                    0.8 * max(1, len(self.reviewer.regions[name]["nodes"])))]
            if not saturated:
                continue
            anchor = max(waiting, key=lambda number: (vehicles[number]["waiting_duration_seconds"], -number))
            start = now - vehicles[anchor]["waiting_duration_seconds"]
            key = f"포화징후_차량{anchor}_노드{vehicles[anchor]['node']}_대기시작{start:.1f}"
            dependencies = [item for item in snapshot["wait_dependencies"] if item["oht_id"] in group]
            current[key] = {"vehicle_ids": sorted(group), "oht_id": anchor, "kind": "포화와 연결된 진행 정지 징후",
                "saturated_region_names": sorted(saturated), "waiting_vehicle_ids": sorted(waiting),
                "maximum_waiting_duration_seconds": vehicles[anchor]["waiting_duration_seconds"],
                "ppt_type_evidence": self.reviewer.type_evidence(group, waiting, vehicles, dependencies, directed, previous),
                "wait_dependencies": dependencies,
                "stability_definition": "동일 선두 차량의 진행 정지. 집단 전체의 동일 차량 연속 정지 시간과 구분"}
            covered_members.update(group)
        # 순환이 입증되지 않아도 오래 풀리지 않는 차단 집단은 버리지 않는다.
        if int(now) % 10 == 0:
            for candidate in self.reviewer.review(snapshot, previous):
                if candidate["priority"] != 1 or set(candidate["waiting_vehicle_ids"]) & covered_members:
                    continue
                key = "장기_" + candidate["candidate_key"]
                current[key] = {**candidate, "vehicle_ids": candidate["related_vehicle_ids"], "kind": "장기 미해소 후보"}
        else:
            for key, event in self.active.items():
                if key.startswith("장기_"):
                    anchor = event["evidence"]["oht_id"]
                    if self.progress(vehicles[anchor]) == event["initial_progress"][anchor] and vehicles[anchor].get("_stall_next"):
                        current[key] = event["evidence"]

        for key in set(self.active) - set(current):
            event = self.active.pop(key)
            moved = [number for number, progress in event["initial_progress"].items()
                     if self.progress(vehicles[number]) != progress]
            event.update(ended_second=now,
                outcome="원래 차량 전원 진행 재개" if len(moved) == len(event["initial_progress"]) else
                        "일부 진행·나머지 별도 관측" if moved else "대기 관계 변경·해소 미확인",
                original_vehicle_ids_with_progress=moved, observation_gap=gap)
            if event["reported"]:
                self.finished.append(event)
                actions.append(("후보변화", event))
        for key, evidence in current.items():
            if key not in self.active or gap:
                self.active[key] = {"event_number": self.next_number, "candidate_key": key,
                    "first_observed_second": now, "last_observed_second": now,
                    "stable_dependency_duration_seconds": 0, "reported": False, "image_saved": False,
                    "initial_progress": {number: self.progress(vehicles[number]) for number in evidence["vehicle_ids"]},
                    "evidence": evidence, "verdict": "후보·교착 미확정"}
                self.next_number += 1
            event = self.active[key]
            event["last_observed_second"] = now
            event["stable_dependency_duration_seconds"] = now - event["first_observed_second"]
            event["evidence"] = evidence
            if not event["reported"] and (event["stable_dependency_duration_seconds"] >= 15 or key.startswith("장기_")):
                event["reported"] = True
                actions.append(("후보발생", event))
            if event["reported"] and not event["image_saved"] and (
                    event["stable_dependency_duration_seconds"] >= 120 or key.startswith("장기_")):
                event["image_saved"] = True
                actions.append(("핵심후보", event))
        self.last_second = now
        return actions


class IntegratedDeadlockRecorder(SimulationRecorder):
    def __init__(self, simulator, output_directory, sim_duration, case_directory):
        # 이전 대량 저장 초기화는 호출하지 않는다. 읽기 전용 자료 추출 메서드만 재사용한다.
        self.simulator = simulator
        self.output_directory = Path(output_directory)
        self.sim_duration = sim_duration
        self.observation_interval_seconds = 1
        self.previous_vehicle_progress = {}
        self.waiting_since = {}
        self.previous_vehicle_regions = {}
        self.region_entries = defaultdict(int)
        self.region_exits = defaultdict(int)
        self.region_waiting_seconds = defaultdict(float)
        self.regions = {}
        self.node_regions = defaultdict(set)
        for directory in ("구간별시계열", "중간집계", "최종집계", "후보별기록", "확정교착기록", "실행로그", "핵심상태"):
            (self.output_directory / directory).mkdir(parents=True, exist_ok=True)
        self.create_region_index()
        self.graph = json.loads((Path(case_directory) / "agv_graph.json").read_text())
        self.tracker = CandidateTracker(self.graph, self.regions)
        self.buffer = deque(maxlen=65)
        self.scene_buffer = deque(maxlen=302)
        self.wait_onset_frames = {}
        self.event_onset_contexts = {}
        self.last_second = None
        self.latest_snapshot = None
        self.saved_times = {}
        self.previous_delta = {}
        self.node_waiting_seconds = defaultdict(float)
        self.physical_waiting_locations = {}
        self.wall_started = time.monotonic()
        self.delta_handle = gzip.open(self.output_directory / "실행로그" / "매초대기상태변화.jsonl.gz", "wt", encoding="utf-8", compresslevel=3)
        self.region_handle = gzip.open(self.output_directory / "구간별시계열" / "60초구간집계.jsonl.gz", "wt", encoding="utf-8", compresslevel=3)
        self.event_handle = (self.output_directory / "후보별기록" / "사건목록.jsonl").open("a", encoding="utf-8")
        self.region_bin = defaultdict(lambda: {"samples": 0, "occupied_sum": 0, "waiting_sum": 0,
            "peak_waiting": 0, "maximum_wait": 0.0})
        self.bin_start = 0
        write_json(self.output_directory / "실행로그" / "관측저장기준.json", {
            "protocol": PROTOCOL_VERSION, "target_types": sorted(TARGET_TYPES), "observation_interval_seconds": 1,
            "candidate_stable_seconds": 15, "key_cycle_photo_stable_seconds": 120,
            "long_wait_photo_seconds": 600, "long_wait_stationary_comparison_seconds": 120,
            "before_scene_reference": "관련 차량 중 최초 실제 대기 시작 시각. 그 시각의 300·120·60초 전 위치 장면을 대기 관측 즉시 메모리에 고정",
            "before_scene_limits": "실행 시작 이전·관측 누락으로 없는 장면은 미확보로 기록. 사진 선택이 늦어져도 최초 기준 시각 유지",
            "confirmation_photo": "구조적 충분조건 충족 즉시 전체·확대 지도와 독립 전체 상태 저장",
            "delta_reconstruction": "최초 관측을 빈 상태에서 시작. changed 필드 덮어쓰기, removed 키 삭제. waiting_started_second로 대기 시간 복원",
            "scope": "매초 관측 사이의 짧은 현상과 충분조건 밖 교착의 부재는 보장하지 않음",
            "region_storage": "매초 관측을 60초 구간별 합계·최대로 집계. 누적 노드 대기 시간 별도 저장"})

    def snapshot(self, now, waiting, counts, dependencies):
        result = super().snapshot(now, waiting, counts, dependencies)
        for record, vehicle in zip(result["vehicles"], self.simulator.OHT_LIST):
            record["remaining_path"] = list((vehicle._leg_path or [])[vehicle._leg_idx:])
            record["remaining_path_is_complete"] = True
            record["_seg_profile"] = dict(vehicle._seg_profile or {})
            for name in ("_replan", "_abort_leg", "_ring_detour_node", "_merge_deny_ring", "_no_alt_node"):
                record[name] = getattr(vehicle, name, None)
        result["MERGE_HOLDER"] = dict(result["MERGE_HOLDER"])
        result["MERGE_EXPIRE"] = dict(result["MERGE_EXPIRE"])
        return result

    def save_state(self, snapshot):
        now = snapshot["simulation_second"]
        scope = "_선행위치" if snapshot.get("snapshot_scope") else ""
        key = (now, scope)
        if key not in self.saved_times:
            relative = f"핵심상태/시각_{now:010.1f}초{scope}.json.gz"
            with gzip.open(self.output_directory / relative, "wt", encoding="utf-8", compresslevel=3) as handle:
                json.dump(snapshot, handle, ensure_ascii=False, separators=(",", ":"))
            self.saved_times[key] = relative
        return self.saved_times[key]

    def get_onset_frames(self, onset_second):
        frames = {}
        for seconds in (300, 120, 60):
            frames[seconds] = next((state for state in reversed(self.scene_buffer)
                if state["simulation_second"] <= onset_second - seconds), None)
        return frames

    def retain_wait_onsets(self, snapshot):
        if not hasattr(self, "wait_onset_frames"):
            self.wait_onset_frames = {}
            self.event_onset_contexts = {}
        active = {}
        for vehicle in snapshot["vehicles"]:
            onset = vehicle.get("_stuck_since")
            if onset is None or vehicle.get("_stall_next") is None:
                continue
            key = (vehicle["oht_id"], onset)
            active[key] = self.wait_onset_frames.get(key) or {
                "reference_simulation_second": onset, "anchor_vehicle_id": vehicle["oht_id"],
                "first_wait_observation_second": snapshot["simulation_second"],
                "frames": self.get_onset_frames(onset)}
        self.wait_onset_frames = active

    def get_event_onset(self, snapshot, vehicle_ids):
        options = [context for (number, _onset), context in self.wait_onset_frames.items() if number in vehicle_ids]
        if options:
            context = min(options, key=lambda item: item["reference_simulation_second"])
            return {**context, "reference_basis": "관련 차량 중 최초 실제 대기 시작"}
        onset = snapshot["simulation_second"]
        return {"reference_simulation_second": onset, "reference_basis": "대기 시작 근거 없음·최초 관측 시각 대체",
                "frames": self.get_onset_frames(onset)}

    def persist_action(self, kind, evidence, snapshot):
        now = snapshot["simulation_second"]
        if kind == "판정반례":
            write_json(self.output_directory / "확정교착기록" / "판정반례.json", evidence)
        elif kind == "확정교착":
            directory = self.output_directory / "확정교착기록" / ("확정_" + evidence["confirmation_key"][:16])
            evidence["state_path"] = self.save_state(snapshot)
            context = self.get_event_onset(snapshot, evidence["vehicle_ids"])
            evidence["before_states"] = self.save_before_states(directory, snapshot, evidence["vehicle_ids"], images=True, context=context)
            write_json(directory / "판정근거.json", evidence)
            self.save_images(directory, snapshot, evidence["vehicle_ids"], "구조적 교착 충분조건 충족")
        else:
            directory = self.output_directory / "후보별기록" / f"후보{evidence['event_number']:05d}"
            evidence["last_state_path"] = self.save_state(snapshot)
            context = self.event_onset_contexts.setdefault(evidence["event_number"],
                self.get_event_onset(snapshot, evidence["evidence"]["vehicle_ids"]))
            if kind == "후보발생":
                evidence["before_states"] = self.save_before_states(directory, snapshot, evidence["evidence"]["vehicle_ids"], context=context)
            if kind == "핵심후보":
                evidence["image_state_path"] = evidence["last_state_path"]
                self.save_images(directory, snapshot, evidence["evidence"]["vehicle_ids"], "우선 검토 후보·교착 미확정")
                evidence["key_before_states"] = self.save_before_states(directory, snapshot, evidence["evidence"]["vehicle_ids"], images=True, context=context)
                evidence["image_directory"] = str(directory)
            write_json(directory / "후보근거.json", evidence)
        self.event_handle.write(json.dumps({"kind": kind, "simulation_second": now, "evidence": evidence},
                                          ensure_ascii=False, separators=(",", ":")) + "\n")

    def save_before_states(self, directory, snapshot, vehicle_ids, images=False, context=None):
        context = context or self.get_event_onset(snapshot, vehicle_ids)
        references = []
        for seconds in (300, 120, 60):
            reference = {"requested_seconds_before": seconds,
                "reference_simulation_second": context["reference_simulation_second"],
                "reference_basis": context["reference_basis"],
                "requested_simulation_second": context["reference_simulation_second"] - seconds}
            compact = context["frames"][seconds]
            if compact is None:
                references.append({**reference, "available": False, "unavailable_reason": "실행 시작 이전 또는 해당 시각 관측 미확보"})
                continue
            state = {"simulation_second": compact["simulation_second"], "snapshot_scope": "시각화 전용 위치 상태",
                "occupancy_mode": snapshot.get("occupancy_mode", "node"), "wait_dependencies": [],
                "vehicles": [{"oht_id": row[0], "node": row[1], "_stall_next": row[2],
                    "position_x_millimeters": row[3], "position_y_millimeters": row[4],
                    "waiting_duration_seconds": row[5], "transfer_node": row[6], "port_id": row[7],
                    "link_index": row[8], "link_offset_mm": row[9]} for row in compact["positions"]]}
            reference.update(available=True, actual_simulation_second=state["simulation_second"],
                             observation_time_difference_seconds=reference["requested_simulation_second"] - state["simulation_second"],
                             state_path=self.save_state(state))
            if images:
                filename = f"최초정체전{seconds}초_전체지도.png"
                draw_map(state, self.graph, directory / filename, "최초 정체 이전 상태",
                         {"vehicle_ids": vehicle_ids}, status_caption="최초 실제 대기 시작 기준·확정 시각과 구분")
                reference["image_path"] = str(directory / filename)
            references.append(reference)
        return references

    def save_images(self, directory, snapshot, vehicle_ids, caption):
        for cropped, filename in ((False, "전체시뮬레이션.png"), (True, "문제구간확대.png")):
            draw_map(snapshot, self.graph, directory / filename, caption,
                     {"vehicle_ids": vehicle_ids}, cropped=cropped, status_caption=caption)

    def capture_failure(self, now):
        """안전 위반 검사 재호출 없이, 오류가 난 바로 그 물리 상태를 보존."""
        controller = self.simulator.get_segment_motion()
        waiting = {vehicle.oht_id: now - record["since_second"]
                   for vehicle, record in controller.wait_records.items()}
        dependencies, _ = self.read_wait_dependencies(now, waiting)
        state = self.snapshot(now, waiting, self.collect_counts(waiting), dependencies)
        directory = self.output_directory / "주행검증실패"
        write_json(directory / "실패근거.json", {"simulation_second": now,
            "safety_failures": controller.safety_failures, "state_path": self.save_state(state),
            "is_valid_experiment": False})
        vehicle_ids = sorted({record["oht_id"] for record in controller.safety_failures}
                             | {record["front_oht_id"] for record in controller.safety_failures if "front_oht_id" in record})
        self.save_images(directory, state, vehicle_ids, "주행 안전 검사 실패·무효 실행")

    def write_delta(self, snapshot):
        waiting = {}
        for vehicle in snapshot["vehicles"]:
            if vehicle.get("_stall_next") is not None and vehicle.get("_stuck_since") is not None:
                record = dict(vehicle)
                duration = record.pop("waiting_duration_seconds")
                record["waiting_started_second"] = snapshot["simulation_second"] - duration
                waiting[str(vehicle["oht_id"])] = record
        values = {"waiting_vehicles": waiting, "wait_dependencies": snapshot["wait_dependencies"],
                  **{name: snapshot[name] for name in ("NODE_HOLDER", "MERGE_HOLDER", "MERGE_EXPIRE", "MERGE_RESERVERS", "MERGE_ADMITTED", "RING_MEMBERS")}}
        changed, removed = {}, {}
        for name, current in values.items():
            previous = self.previous_delta.get(name)
            if isinstance(current, dict):
                previous = previous or {}
                updates = {key: value for key, value in current.items() if previous.get(key) != value}
                deleted = sorted(set(previous) - set(current))
                if updates:
                    changed[name] = updates
                if deleted:
                    removed[name] = deleted
            elif previous != current:
                changed[name] = current
        self.delta_handle.write(json.dumps({"simulation_second": snapshot["simulation_second"], "changed": changed,
            "removed": removed}, ensure_ascii=False, separators=(",", ":")) + "\n")
        self.previous_delta = values

    def flush_regions(self, now):
        for name, values in self.region_bin.items():
            self.region_handle.write(json.dumps({"start_second": self.bin_start, "end_second": now,
                "region_name": name, **values}, ensure_ascii=False, separators=(",", ":")) + "\n")
        self.region_bin.clear()
        self.bin_start = now

    def observe(self, now, force_snapshot=False):
        if self.last_second is not None and now <= self.last_second:
            return
        waiting = self.vehicle_waits(now)
        dependencies, _ = self.read_wait_dependencies(now, waiting)
        counts = self.collect_counts(waiting)
        state = self.snapshot(now, waiting, counts, dependencies)
        self.scene_buffer.append({"simulation_second": now, "positions": [
            (vehicle["oht_id"], vehicle["node"], vehicle["_stall_next"],
             vehicle["position_x_millimeters"], vehicle["position_y_millimeters"],
             vehicle["waiting_duration_seconds"], vehicle.get("transfer_node"), vehicle.get("port_id"),
             vehicle.get("link_index"), vehicle.get("link_offset_mm")) for vehicle in state["vehicles"]]})
        self.retain_wait_onsets(state)
        previous = next((item for item in reversed(self.buffer) if item["simulation_second"] <= now - 120), None)
        actions = self.tracker.update(state, previous)
        for event in self.tracker.active.values():
            self.event_onset_contexts.setdefault(event["event_number"],
                self.get_event_onset(state, event["evidence"]["vehicle_ids"]))
        for kind, evidence in actions:
            self.persist_action(kind, evidence, state)
        active_numbers = {event["event_number"] for event in self.tracker.active.values()}
        self.event_onset_contexts = {number: context for number, context in self.event_onset_contexts.items() if number in active_numbers}
        self.write_delta(state)
        for name, values in counts.items():
            row = self.region_bin[name]
            row["samples"] += 1
            row["occupied_sum"] += len(values["occupied_nodes"])
            row["waiting_sum"] += len(values["waiting_vehicle_ids"])
            row["peak_waiting"] = max(row["peak_waiting"], len(values["waiting_vehicle_ids"]))
            row["maximum_wait"] = max(row["maximum_wait"], values["maximum_wait"])
        if self.last_second is not None:
            for vehicle in state["vehicles"]:
                if vehicle["oht_id"] in waiting:
                    self.node_waiting_seconds[vehicle.get("region_reference_node", vehicle["node"])] += now - self.last_second
                    if state.get("occupancy_mode") == "segment":
                        key = (f"노드:{vehicle['node']}" if vehicle["link_index"] is None else
                               f"{vehicle['link_index']}:{vehicle['link_offset_mm']:.3f}")
                        record = self.physical_waiting_locations.setdefault(key, {
                            "link_index": vehicle["link_index"], "link_offset_mm": vehicle["link_offset_mm"],
                            "position_x_millimeters": vehicle["position_x_millimeters"],
                            "position_y_millimeters": vehicle["position_y_millimeters"],
                            "transfer_node": vehicle["transfer_node"], "port_id": vehicle["port_id"],
                            "vehicle_wait_seconds": 0.0})
                        record["vehicle_wait_seconds"] += now - self.last_second
        if now and int(now) % 60 == 0:
            self.flush_regions(now)
        if int(now) % 10 == 0:
            self.buffer.append(copy.deepcopy(state))
        self.latest_snapshot = state
        self.last_second = now
        if int(now) % 300 == 0 or force_snapshot:
            self.checkpoint(now)

    def checkpoint(self, now):
        simulator = self.simulator
        active = [event for event in self.tracker.active.values() if event["reported"]]
        confirmed = [record["proof"] for record in self.tracker.confirmed.values() if not record["revoked"]]
        progress = {"updated_at": current_timestamp(), "simulation_second": now, "sim_duration": self.sim_duration,
            "num_oht": len(simulator.OHT_LIST), "injected_lots": simulator._JOB_SCHEDULE_IDX[0],
            "completed_lots": len(simulator.LOT_STATS), "completed_transfers": len(simulator.JOB_STATS),
            "pending_jobs": len(simulator.PENDING_JOBS), "active_candidate_count": len(active),
            "confirmed_structural_deadlock_count": len(confirmed), "proof_refutation_count": len(self.tracker.refutations),
            "inspected_observation_count": self.tracker.inspection_count,
            "wall_elapsed_seconds": time.monotonic() - self.wall_started,
            "maximum_resident_memory_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
            "storage_free_bytes": shutil.disk_usage(self.output_directory).free}
        write_json(self.output_directory / "중간집계" / "현재진행상태.json", progress)
        write_json(self.output_directory / "중간집계" / "유형후보현황.json", {
            "updated_at": current_timestamp(), "active_candidates": active, "confirmed": confirmed,
            "refutations": self.tracker.refutations, "scope": "시작부터 매초 검사. 충분조건 밖 사례는 후보이며 교착 부재 보장 아님"})
        self.delta_handle.flush()
        self.region_handle.flush()
        self.event_handle.flush()
        if progress["storage_free_bytes"] < 5 * 1024**3:
            raise RuntimeError("남은 실제 저장공간 5GiB 미만: 자료 손상을 피하기 위해 종료")

    def close(self, now, completed):
        if self.latest_snapshot is not None:
            for event in self.tracker.active.values():
                if event["reported"]:
                    event.update(outcome="관측 종료 미해소·이후 해소 여부 미관측", observation_ended_second=now)
                    self.persist_action("관측종료", event, self.latest_snapshot)
            self.save_state(self.latest_snapshot)
        self.flush_regions(now)
        self.checkpoint(now)
        write_json(self.output_directory / "최종집계" / "탐지요약.json", {
            "protocol": PROTOCOL_VERSION, "completed": completed, "simulation_second": now,
            "inspected_observation_count": self.tracker.inspection_count,
            "reported_candidate_count": len(self.tracker.finished) + sum(event["reported"] for event in self.tracker.active.values()),
            "confirmed_structural_deadlock_count": sum(not record["revoked"] for record in self.tracker.confirmed.values()),
            "proof_refutations": self.tracker.refutations,
            "limits": "매초 관측 및 보수적 충분조건 검사. 0건은 모든 가능한 교착 부재를 보장하지 않음"})
        write_json(self.output_directory / "최종집계" / "노드별누적대기초.json", self.node_waiting_seconds)
        if getattr(self.simulator, "ENABLE_SEGMENT_OCCUPANCY", False):
            write_json(self.output_directory / "최종집계" / "실제위치별누적대기.json", self.physical_waiting_locations)
        for handle in (self.delta_handle, self.region_handle, self.event_handle):
            handle.close()
