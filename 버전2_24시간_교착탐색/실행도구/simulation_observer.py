"""버전 2 제어 함수를 호출하지 않는 상태 관측과 사건 전후 기록.

대기 그래프의 용량 자원 간선은 '후보' 탐색용이다. 용량 반환은 여러 차량 중
하나의 이동으로도 가능하므로 순환 성분을 영구 교착의 증명으로 사용하지 않는다.
"""

from collections import defaultdict, deque
import csv
import gzip
import json
from pathlib import Path
import resource
import shutil
import time

from experiment_support import current_timestamp, write_json


def strongly_connected_groups(graph):
    """재귀 깊이에 의존하지 않는 방향 그래프 순환 성분 탐색."""
    vertices = set(graph)
    reverse_graph = defaultdict(set)
    for vertex, neighbors in graph.items():
        vertices.update(neighbors)
        for neighbor in neighbors:
            reverse_graph[neighbor].add(vertex)
    visited, finishing_order = set(), []
    for initial_vertex in sorted(vertices):
        if initial_vertex in visited:
            continue
        stack = [(initial_vertex, False)]
        while stack:
            vertex, finished = stack.pop()
            if finished:
                finishing_order.append(vertex)
            elif vertex not in visited:
                visited.add(vertex)
                stack.append((vertex, True))
                stack.extend((neighbor, False) for neighbor in sorted(graph.get(vertex, ()), reverse=True)
                             if neighbor not in visited)
    visited.clear()
    groups = []
    for initial_vertex in reversed(finishing_order):
        if initial_vertex in visited:
            continue
        group, stack = set(), [initial_vertex]
        while stack:
            vertex = stack.pop()
            if vertex in visited:
                continue
            visited.add(vertex)
            group.add(vertex)
            stack.extend(reverse_graph.get(vertex, ()))
        if len(group) > 1:
            groups.append(group)
    return groups


class SimulationRecorder:
    def __init__(self, simulator, output_directory, sim_duration,
                 observation_interval_seconds=1, snapshot_interval_seconds=10,
                 risk_hold_seconds=30, long_wait_seconds=60):
        self.simulator = simulator
        self.output_directory = Path(output_directory)
        self.sim_duration = sim_duration
        self.observation_interval_seconds = observation_interval_seconds
        self.snapshot_interval_seconds = snapshot_interval_seconds
        self.risk_hold_seconds = risk_hold_seconds
        self.long_wait_seconds = long_wait_seconds
        self.previous_vehicle_progress = {}
        self.waiting_since = {}
        self.previous_vehicle_regions = {}
        self.region_entries = defaultdict(int)
        self.region_exits = defaultdict(int)
        self.region_waiting_seconds = defaultdict(float)
        self.condition_since = {}
        self.active_events = {}
        self.all_events = []
        self.previous_candidate_groups = {}
        self.buffered_snapshots = deque()
        self.saved_snapshot_times = set()
        self.last_heartbeat_second = -300
        self.last_region_counts = {}
        self.last_wait_dependencies = []
        self.previous_region_rows = {}
        self.event_label_changed = False
        self.wall_started = time.monotonic()
        self.regions = {}
        self.node_regions = defaultdict(set)
        for directory in ("실행로그", "구간별시계열", "중간집계", "사건별기록", "사건상태공유보관", "최종집계"):
            (self.output_directory / directory).mkdir(parents=True, exist_ok=True)
        self.create_region_index()
        self.time_series_handle = gzip.open(self.output_directory / "구간별시계열" / "구간관측.csv.gz",
                                           "wt", encoding="utf-8", newline="", compresslevel=3)
        self.time_series_writer = csv.writer(self.time_series_handle)
        self.time_series_writer.writerow(["simulation_second", "region_name", "occupied_node_count",
            "node_occupancy_ratio", "waiting_vehicle_count", "maximum_waiting_duration_seconds",
            "controlled_capacity_utilization", "entries_since_start", "exits_since_start"])
        write_json(self.output_directory / "구간별시계열" / "저장형식.json", {
            "observation_interval_seconds": observation_interval_seconds,
            "storage_mode": "값이 바뀐 행만 저장. 같은 구간의 다음 행까지 마지막 값을 유지하여 1초 시계열 복원",
            "initial_row_for_every_region": True,
            "capacity_note": "controlled_capacity_utilization은 제공 고리 제어 용량 대비 값이며 물리적 공간 포화율이 아님"})
        self.event_log = (self.output_directory / "사건별기록" / "사건목록.jsonl").open("a", encoding="utf-8")
        self.control_log = gzip.open(self.output_directory / "실행로그" / "제어상태변화.jsonl.gz",
                                    "wt", encoding="utf-8", compresslevel=1)
        self.previous_control_state = None

    def create_region_index(self):
        simulator = self.simulator
        for node, number in simulator.NODE_TO_BAY_ID.items():
            self.node_regions[node].add(f"베이_{number}")
        for node, number in simulator.NODE_TO_CORRIDOR_ID.items():
            self.node_regions[node].add(f"중앙통로_{number}")
        for node, numbers in simulator.NODE_TO_MERGE_ID.items():
            for number in numbers:
                self.node_regions[node].add(f"합류부_{number}")
        for ring_number, ring in enumerate(simulator.RINGS):
            for node in ring["nodes"]:
                self.node_regions[node].add(f"고리_{ring_number}")
        for merge_node, nodes in simulator.MERGE_OUT_NODES.items():
            for node in nodes:
                self.node_regions[node].add(f"합류출구_{merge_node}")
        for node, names in self.node_regions.items():
            for name in names:
                self.regions.setdefault(name, {"nodes": [], "cap": None})["nodes"].append(node)
        for ring_number, ring in enumerate(simulator.RINGS):
            self.regions[f"고리_{ring_number}"]["cap"] = ring["cap"]
        write_json(self.output_directory / "구간별시계열" / "지도구간목록.json", self.regions)

    def vehicle_waits(self, now):
        waiting = {}
        if getattr(self.simulator, "ENABLE_SEGMENT_OCCUPANCY", False):
            controller = self.simulator.get_segment_motion()
            controller.check_safety()
            return {vehicle.oht_id: now - record["since_second"]
                    for vehicle, record in controller.wait_records.items()
                    if controller.get_position(vehicle, now)["speed_mm_per_second"] <= 1e-5}
        for vehicle in self.simulator.OHT_LIST:
            progress = (vehicle.node, vehicle.total_distance, vehicle._seg_start_t)
            blocked = vehicle._stall_next is not None and vehicle._stuck_since is not None
            if blocked:
                if (self.previous_vehicle_progress.get(vehicle.oht_id) != progress
                        or vehicle.oht_id not in self.waiting_since):
                    self.waiting_since[vehicle.oht_id] = now
                waiting[vehicle.oht_id] = now - self.waiting_since[vehicle.oht_id]
            else:
                self.waiting_since.pop(vehicle.oht_id, None)
            self.previous_vehicle_progress[vehicle.oht_id] = progress
        return waiting

    def read_wait_dependencies(self, now, waiting):
        """속성만 읽음. ring_entry_blocked·merge_request·_path_clear_ahead 호출 금지."""
        simulator = self.simulator
        dependencies = []
        graph = defaultdict(set)
        if getattr(simulator, "ENABLE_SEGMENT_OCCUPANCY", False):
            controller = simulator.get_segment_motion()
            for vehicle, record in controller.wait_records.items():
                if vehicle.oht_id not in waiting:
                    continue
                blockers = sorted({member.oht_id for member in record["blocking_vehicles"] if member is not vehicle})
                detail = dict(record["detail"])
                resource_name = None
                if record["cause"] == "merge_permission":
                    resource_name = f"merge:{detail.get('merge_node')}"
                elif record["cause"] == "port_service":
                    resource_name = f"port:{detail.get('port_id')}"
                dependencies.append({"oht_id": vehicle.oht_id, "cause": record["cause"],
                    "blocking_vehicle_ids": blockers, "resource_name": resource_name, "details": detail})
                graph[vehicle.oht_id].update(blockers)
            return dependencies, graph

        def add(vehicle, cause, blockers, resource_name=None, details=None):
            blocker_ids = sorted({member.oht_id for member in blockers if member is not vehicle})
            origin = f"vehicle:{vehicle.oht_id}"
            if resource_name is None:
                for blocker_id in blocker_ids:
                    graph[origin].add(f"vehicle:{blocker_id}")
            else:
                graph[origin].add(resource_name)
                for blocker_id in blocker_ids:
                    graph[resource_name].add(f"vehicle:{blocker_id}")
            dependencies.append({"oht_id": vehicle.oht_id, "cause": cause,
                "blocking_vehicle_ids": blocker_ids, "resource_name": resource_name, "details": details})

        for vehicle in simulator.OHT_LIST:
            if vehicle.oht_id not in waiting:
                continue
            next_node = vehicle._stall_next
            holder = simulator.NODE_HOLDER.get(next_node)
            if holder is not None and holder is not vehicle:
                add(vehicle, "node_lock", [holder], details={"node": next_node})
            path = vehicle._leg_path or []
            try:
                next_index = path.index(next_node, max(0, vehicle._leg_idx))
            except ValueError:
                next_index = len(path)
            cumulative_distance = 0.0
            for index in range(next_index, len(path) - 1):
                cumulative_distance += simulator.edge_length(path[index], path[index + 1])
                if cumulative_distance >= simulator.FOLLOW_CENTER_GAP_MM:
                    break
                holder = simulator.NODE_HOLDER.get(path[index + 1])
                if holder is not None and holder is not vehicle:
                    add(vehicle, "follow_distance", [holder], details={
                        "node": path[index + 1], "distance_from_next_node_millimeters": cumulative_distance})
            for ring_number in simulator.NODE_RINGS.get(next_node, ()):
                ring = simulator.RINGS[ring_number]
                members = simulator.RING_MEMBERS[ring_number]
                if vehicle in members or len(members) < ring["cap"]:
                    continue
                if not ring.get("hard"):
                    if vehicle._ring_exempt:
                        continue
                    if vehicle._stuck_since is not None and now - vehicle._stuck_since >= simulator.RING_WAIT_OVERRIDE_S:
                        continue
                    held = simulator.HELD_NODES.get(vehicle, ())
                    exits_full_ring = any(next_node not in simulator.RINGS[other_number]["nodes"]
                        and len(simulator.RING_MEMBERS[other_number]) >= simulator.RINGS[other_number]["cap"]
                        for held_node in held for other_number in simulator.NODE_RINGS.get(held_node, ()))
                    if exits_full_ring:
                        continue
                add(vehicle, "ring_capacity", members, f"ring:{ring_number}", {"cap": ring["cap"]})
            if vehicle._merge_wait is not None:
                merge_node, side = vehicle._merge_wait
                admitted = simulator.MERGE_ADMITTED.get(merge_node, {})
                blockers = [member for members in admitted.values() for member in members]
                outgoing = simulator.MERGE_OUT_NODES.get(merge_node, [])
                outgoing_holders = [simulator.NODE_HOLDER.get(node) for node in outgoing]
                out_full = bool(outgoing) and all(member is not None and
                    (not simulator.MERGE_OUT_FULL_STOPPED_ONLY or member._cur_speed <= 0)
                    for member in outgoing_holders)
                if out_full:
                    blockers.extend(member for member in outgoing_holders if member is not None)
                add(vehicle, "merge_permission", blockers, f"merge:{merge_node}", {
                    "side": side, "MERGE_HOLDER": simulator.MERGE_HOLDER.get(merge_node),
                    "MERGE_EXPIRE": simulator.MERGE_EXPIRE.get(merge_node),
                    "outgoing_segment_full": out_full, "_merge_deny_ring": vehicle._merge_deny_ring})
        return dependencies, graph

    def collect_counts(self, waiting):
        counts = {name: {"occupied_nodes": set(), "vehicle_ids": set(), "waiting_vehicle_ids": set(),
                         "maximum_wait": 0.0} for name in self.regions}
        if getattr(self.simulator, "ENABLE_SEGMENT_OCCUPANCY", False):
            controller = self.simulator.get_segment_motion()
            for vehicle in self.simulator.OHT_LIST:
                state = controller.get_position(vehicle)
                index = state["link_index"]
                region_node = (vehicle.node if index is None else (
                    self.simulator.LINK_A[index] if state["offset_mm"] < self.simulator.LINK_LEN[index] / 2
                    else self.simulator.LINK_B[index]))
                names = self.node_regions.get(region_node, set())
                for name in names:
                    counts[name]["occupied_nodes"].add(region_node)
                    counts[name]["vehicle_ids"].add(vehicle.oht_id)
                    if vehicle.oht_id in waiting:
                        counts[name]["waiting_vehicle_ids"].add(vehicle.oht_id)
                        counts[name]["maximum_wait"] = max(counts[name]["maximum_wait"], waiting[vehicle.oht_id])
            for name, values in counts.items():
                self.region_waiting_seconds[name] += len(values["waiting_vehicle_ids"]) * self.observation_interval_seconds
            return counts
        for node, holder in self.simulator.NODE_HOLDER.items():
            if holder is not None:
                for name in self.node_regions.get(node, ()):
                    counts[name]["occupied_nodes"].add(node)
                    counts[name]["vehicle_ids"].add(holder.oht_id)
        for vehicle in self.simulator.OHT_LIST:
            names = self.node_regions.get(vehicle.node, set())
            previous_names = self.previous_vehicle_regions.get(vehicle.oht_id, set())
            for name in names - previous_names:
                self.region_entries[name] += 1
            for name in previous_names - names:
                self.region_exits[name] += 1
            self.previous_vehicle_regions[vehicle.oht_id] = names
            if vehicle.oht_id in waiting:
                for name in names:
                    counts[name]["waiting_vehicle_ids"].add(vehicle.oht_id)
                    counts[name]["maximum_wait"] = max(counts[name]["maximum_wait"], waiting[vehicle.oht_id])
        for name, values in counts.items():
            self.region_waiting_seconds[name] += len(values["waiting_vehicle_ids"]) * self.observation_interval_seconds
        return counts

    def snapshot(self, now, waiting, counts, dependencies):
        simulator = self.simulator
        vehicles = []
        for vehicle in simulator.OHT_LIST:
            screen_x, screen_y = vehicle._interp_xy(now)
            job = vehicle.job
            remaining_path = (vehicle._leg_path or [])[vehicle._leg_idx:]
            path_is_complete = vehicle.oht_id in waiting or len(remaining_path) <= 12
            vehicles.append({"oht_id": vehicle.oht_id, "node": vehicle.node, "state": vehicle.state,
                "_stall_next": vehicle._stall_next, "_stuck_since": vehicle._stuck_since,
                "_cur_speed": vehicle._cur_speed, "_ring_waiting": vehicle._ring_waiting,
                "_merge_wait": vehicle._merge_wait,
                "waiting_duration_seconds": waiting.get(vehicle.oht_id, 0),
                "position_x_millimeters": (screen_x - simulator.MARGIN) / simulator.SCALE,
                "position_y_millimeters": (screen_y - simulator.MARGIN) / simulator.SCALE,
                "job_id": job.job_id if job is not None else None,
                "to_node": job.to_node if job is not None else None,
                "job_state": job.state if job is not None else None,
                "remaining_path": remaining_path if path_is_complete else remaining_path[:12],
                "remaining_path_is_complete": path_is_complete,
                "path_target_node": remaining_path[-1] if remaining_path else None,
                "total_distance": vehicle.total_distance,
                "_seg_start_t": vehicle._seg_start_t, "_seg_profile": vehicle._seg_profile})
        if getattr(simulator, "ENABLE_SEGMENT_OCCUPANCY", False):
            controller = simulator.get_segment_motion()
            for record, vehicle in zip(vehicles, simulator.OHT_LIST):
                actual = controller.get_position(vehicle, now)
                options, is_complete = controller.get_physical_movement_options(vehicle)
                index = actual["link_index"]
                region_node = (vehicle.node if index is None else (
                    simulator.LINK_A[index] if actual["offset_mm"] < simulator.LINK_LEN[index] / 2 else simulator.LINK_B[index]))
                record.update(link_index=actual["link_index"], link_offset_mm=actual["offset_mm"],
                    position_x_millimeters=actual["x_mm"], position_y_millimeters=actual["y_mm"],
                    total_distance=actual["total_distance_mm"], _cur_speed=actual["speed_mm_per_second"],
                    transfer_node=actual["transfer_node"], port_id=actual["port_id"],
                    region_reference_node=region_node, physical_region_names=sorted(self.node_regions.get(region_node, ())),
                    movement_options=options, movement_options_are_complete=is_complete,
                    is_route_active=bool(controller.routes.get(vehicle)),
                    physical_ring_numbers=[number for number, members in enumerate(simulator.RING_MEMBERS) if vehicle in members])
        return {"simulation_second": now, "captured_at": current_timestamp(),
            "occupancy_mode": "segment" if getattr(simulator, "ENABLE_SEGMENT_OCCUPANCY", False) else "node",
            "vehicles": vehicles, "wait_dependencies": dependencies,
            "NODE_HOLDER": {node: holder.oht_id for node, holder in simulator.NODE_HOLDER.items() if holder is not None},
            "MERGE_HOLDER": dict(simulator.MERGE_HOLDER), "MERGE_EXPIRE": dict(simulator.MERGE_EXPIRE),
            "SEGMENT_MERGE_QUEUE": {node: [
                {"oht_id": member.oht_id, "requested_second": entry[0],
                 "sequence": entry[1], "incoming_link": entry[2]}
                for member, entry in sorted(queue.items(), key=lambda item: item[1][1])]
                for node, queue in controller.merge_waiters.items() if queue}
                if getattr(simulator, "ENABLE_SEGMENT_OCCUPANCY", False) else {},
            "SEGMENT_CONTROL_CLAIMS": {str(member.oht_id): dict(claims)
                for member, claims in controller.control_claims.items() if claims}
                if getattr(simulator, "ENABLE_SEGMENT_OCCUPANCY", False) else {},
            "SEGMENT_RING_RESERVATIONS": {str(number): sorted(member.oht_id for member in members)
                for number, members in controller.ring_reservations.items() if members}
                if getattr(simulator, "ENABLE_SEGMENT_OCCUPANCY", False) else {},
            "MERGE_RESERVERS": {node: {str(side): [member.oht_id for member in members]
                                  for side, members in sides.items()}
                                  for node, sides in simulator.MERGE_RESERVERS.items()},
            "MERGE_ADMITTED": {node: {str(side): sorted(member.oht_id for member in members)
                                 for side, members in sides.items()}
                                 for node, sides in simulator.MERGE_ADMITTED.items()},
            "RING_MEMBERS": [sorted(member.oht_id for member in members) for members in simulator.RING_MEMBERS],
            "regions": {name: {"occupied_node_count": len(values["occupied_nodes"]),
                "waiting_vehicle_count": len(values["waiting_vehicle_ids"]),
                "maximum_waiting_duration_seconds": values["maximum_wait"],
                "controlled_capacity_utilization": len(values["vehicle_ids"]) / self.regions[name]["cap"]
                    if self.regions[name]["cap"] else None} for name, values in counts.items()},
            "counters": self.control_state()}

    def control_state(self):
        simulator = self.simulator
        return {"RING_STATS": dict(simulator.RING_STATS), "MERGE_OCS_STATS": dict(simulator.MERGE_OCS_STATS),
                "REROUTE_COUNTS": dict(simulator.REROUTE_COUNTS)}

    def save_snapshot(self, snapshot):
        now = snapshot["simulation_second"]
        relative_path = f"사건상태공유보관/시각_{now:010.1f}초.json.gz"
        if now not in self.saved_snapshot_times:
            with gzip.open(self.output_directory / relative_path, "wt", encoding="utf-8", compresslevel=6) as handle:
                json.dump(snapshot, handle, ensure_ascii=False, separators=(",", ":"))
            self.saved_snapshot_times.add(now)
        return relative_path

    def start_or_update_event(self, key, now, label, region_names, vehicle_ids, evidence, type_hypotheses):
        event = self.active_events.get(key)
        if event is None:
            directory_name = f"사건{len(self.all_events) + 1:05d}_발생시각{now:010.1f}초"
            event = {"event_number": len(self.all_events) + 1, "directory_name": directory_name,
                "label": label, "first_detection_second": now, "last_observed_second": now,
                "region_names": sorted(region_names), "vehicle_ids": sorted(vehicle_ids),
                "evidence": evidence, "type_hypotheses": type_hypotheses,
                "verdict": "관측 후보, 교착 확정 아님", "status": "관측중", "observed_labels": [label],
                "snapshot_paths": [self.save_snapshot(snapshot) for snapshot in self.buffered_snapshots]}
            directory = self.output_directory / "사건별기록" / directory_name
            for name in ("발생전후상태", "전체지도", "구간확대지도", "대기관계"):
                (directory / name).mkdir(parents=True, exist_ok=True)
            self.active_events[key] = event
            self.all_events.append(event)
            self.event_log.write(json.dumps({"transition": "발생", **event}, ensure_ascii=False) + "\n")
            self.event_log.flush()
        event.update(last_observed_second=now, evidence=evidence, vehicle_ids=sorted(vehicle_ids))
        if label not in event["observed_labels"]:
            event["observed_labels"].append(label)
            self.event_label_changed = True
        if label == "제어용량포화" or event["label"] != "제어용량포화":
            event["label"] = label
        event["type_hypotheses"] = type_hypotheses
        event["tail_until_second"] = now + 600
        return event

    def observe(self, now, force_snapshot=False):
        previous_event_count = len(self.all_events)
        self.event_label_changed = False
        waiting = self.vehicle_waits(now)
        counts = self.collect_counts(waiting)
        dependencies, graph = self.read_wait_dependencies(now, waiting)
        self.last_region_counts = counts
        self.last_wait_dependencies = dependencies
        current_keys = set()
        for name, values in counts.items():
            cap = self.regions[name]["cap"]
            pressure = bool(cap and len(values["vehicle_ids"]) >= 0.8 * cap
                            and len(values["waiting_vehicle_ids"]) >= 2)
            long_wait = len(values["waiting_vehicle_ids"]) >= 3 and values["maximum_wait"] >= self.long_wait_seconds
            if not (pressure or long_wait):
                self.condition_since.pop(name, None)
                continue
            self.condition_since.setdefault(name, now)
            if now - self.condition_since[name] < self.risk_hold_seconds:
                continue
            saturated = bool(cap and len(values["vehicle_ids"]) >= cap)
            key = f"region:{name}"
            current_keys.add(key)
            self.start_or_update_event(key, now, "제어용량포화" if saturated else "위험증가",
                {name}, values["waiting_vehicle_ids"], {
                    "occupied_vehicle_count": len(values["vehicle_ids"]), "cap": cap,
                    "waiting_vehicle_count": len(values["waiting_vehicle_ids"]),
                    "maximum_waiting_duration_seconds": values["maximum_wait"]}, [])
        current_cycle_signatures = {}
        for group in strongly_connected_groups(graph):
            vehicle_ids = {int(vertex.split(":")[1]) for vertex in group if vertex.startswith("vehicle:")}
            if len(vehicle_ids) < 2:
                continue
            nodes = {self.simulator.OHT_LIST[number].node for number in vehicle_ids}
            group_signature = tuple(sorted(group))
            since = self.previous_candidate_groups.get(group_signature, now)
            current_cycle_signatures[group_signature] = since
            if now - since < 15:
                continue
            region_names = {name for node in nodes for name in self.node_regions.get(node, ())}
            ring_names = {name for name in region_names if name.startswith("고리_")}
            hypotheses = []
            if len(ring_names) == 1:
                hypotheses.append({"type": "TYPE I", "reason": "단일 관리 고리와 순환 대기 후보가 겹침, 탈출 가능성 미검증"})
            if len(ring_names) > 1:
                hypotheses.append({"type": "TYPE II", "reason": "여러 관리 고리에 걸친 순환 대기 후보, 용량 대기 선택지 미검증"})
            if any(vertex.startswith("merge:") for vertex in group):
                hypotheses.append({"type": "TYPE III", "reason": "합류 허가 자원을 포함한 순환 의존 후보, 허가 만료·대안 미검증"})
            exit_dependencies = [dependency for dependency in dependencies
                if dependency["oht_id"] in vehicle_ids and dependency["cause"] == "merge_permission"
                and dependency["details"]["outgoing_segment_full"]]
            if exit_dependencies:
                hypotheses.append({"type": "TYPE IV", "reason": "출구 만차와 순환 의존이 함께 관측됨, 역전파 시간 순서 확인 필요"})
            # 차량 구성이 바뀌어도 같은 위치의 장기 사건으로 묶음.
            key = "cycle:" + ",".join(sorted(region_names))
            current_keys.add(key)
            self.start_or_update_event(key, now, "순환대기후보", region_names, vehicle_ids,
                {"observed_vertices": sorted(group), "stable_dependency_duration_seconds": now - since,
                 "requires_alternative_exit_and_resource_semantics_review": True}, hypotheses)
        self.previous_candidate_groups = current_cycle_signatures
        event_transition = (len(self.all_events) != previous_event_count or self.event_label_changed or any(
            key not in current_keys and event["status"] == "관측중" for key, event in self.active_events.items()))
        if (force_snapshot or int(now) % self.snapshot_interval_seconds == 0
                or event_transition or not self.buffered_snapshots):
            # 직렬화하여 시뮬레이터 딕셔너리의 미래 변경과 분리. 난수 사용 없음.
            snapshot = json.loads(json.dumps(self.snapshot(now, waiting, counts, dependencies), default=str))
            self.buffered_snapshots.append(snapshot)
            while (self.buffered_snapshots and
                   self.buffered_snapshots[0]["simulation_second"] < now - 300):
                self.buffered_snapshots.popleft()
            recent_event = any(now - event["first_detection_second"] <= 600 or event["status"] == "해소후추적중"
                               for event in self.active_events.values())
            if (force_snapshot or event_transition or int(now) % 300 == 0 or
                    self.active_events and (recent_event or int(now) % 60 == 0)):
                relative_path = self.save_snapshot(snapshot)
                for event in self.active_events.values():
                    if relative_path not in event["snapshot_paths"]:
                        event["snapshot_paths"].append(relative_path)
        for key, event in list(self.active_events.items()):
            if key not in current_keys and event["status"] == "관측중":
                event["status"] = "해소후추적중"
                event["last_condition_second"] = now
            elif key in current_keys:
                event["status"] = "관측중"
            if key not in current_keys and now >= event["tail_until_second"]:
                event["status"] = "관측해소"
                self.persist_event(event)
                self.event_log.write(json.dumps({"transition": "해소", "event_number": event["event_number"],
                                                "simulation_second": now}, ensure_ascii=False) + "\n")
                del self.active_events[key]
        for name, values in counts.items():
            cap = self.regions[name]["cap"]
            row = [len(values["occupied_nodes"]),
                round(len(values["occupied_nodes"]) / len(self.regions[name]["nodes"]), 5),
                len(values["waiting_vehicle_ids"]), round(values["maximum_wait"], 3),
                round(len(values["vehicle_ids"]) / cap, 5) if cap else "",
                self.region_entries[name], self.region_exits[name]]
            if row != self.previous_region_rows.get(name):
                self.time_series_writer.writerow([now, name, *row])
                self.previous_region_rows[name] = row
        control_state = self.control_state()
        if control_state != self.previous_control_state:
            self.control_log.write(json.dumps({"simulation_second": now, **control_state}, ensure_ascii=False) + "\n")
            self.previous_control_state = control_state
        if now - self.last_heartbeat_second >= 300 or force_snapshot:
            self.last_heartbeat_second = now
            self.checkpoint(now)

    def persist_event(self, event):
        directory = self.output_directory / "사건별기록" / event["directory_name"]
        write_json(directory / "사건정보.json", {key: value for key, value in event.items() if key != "snapshot_paths"})
        write_json(directory / "발생전후상태" / "상태파일목록.json", {
            "paths_relative_to_experiment_directory": event["snapshot_paths"],
            "note": "여러 사건의 동일 시각 상태는 공유 보관하여 중복 저장 방지"})

    def checkpoint(self, now):
        for event in self.active_events.values():
            self.persist_event(event)
        self.time_series_handle.flush()
        self.control_log.flush()
        self.event_log.flush()
        simulator = self.simulator
        progress = {"updated_at": current_timestamp(), "simulation_second": now,
            "sim_duration": self.sim_duration, "num_oht": len(simulator.OHT_LIST),
            "injected_lots": simulator._JOB_SCHEDULE_IDX[0], "completed_lots": len(simulator.LOT_STATS),
            "completed_transfers": len(simulator.JOB_STATS), "pending_jobs": len(simulator.PENDING_JOBS),
            "event_count": len(self.all_events), "active_event_count": len(self.active_events),
            "wall_elapsed_seconds": time.monotonic() - self.wall_started,
            "maximum_resident_memory_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
            "storage_free_bytes": shutil.disk_usage(self.output_directory).free}
        write_json(self.output_directory / "중간집계" / "현재진행상태.json", progress)
        print(json.dumps(progress, ensure_ascii=False), flush=True)
        if progress["storage_free_bytes"] < 5 * 1024**3:
            raise RuntimeError("저장 공간 5GiB 미만: 입력을 누락시키지 않고 실행을 오류 중단함")

    def close(self, now, completed):
        for event in self.active_events.values():
            event["status"] = "관측종료까지지속" if event["status"] == "관측중" else "해소후추적종료"
            event["right_censored"] = True
            self.persist_event(event)
        write_json(self.output_directory / "최종집계" / "사건목록.json", [{
            key: value for key, value in event.items() if key != "snapshot_paths"} for event in self.all_events])
        write_json(self.output_directory / "최종집계" / "구간별누적대기.json", {
            "simulation_second": now, "completed": completed,
            "region_waiting_vehicle_seconds": self.region_waiting_seconds})
        for handle in (self.time_series_handle, self.event_log, self.control_log):
            handle.close()


def create_observer_component(simulator, recorder):
    class ReadOnlySimulationObserver(simulator.sim.Component):
        def process(self):
            while True:
                yield self.hold(recorder.observation_interval_seconds)
                recorder.observe(self.env.now())
    return ReadOnlySimulationObserver()
