"""동일 차량의 진행 정지와 실제 차단 관계로 검토 우선순위를 정한다. 확률·교착 확정 판정 아님."""

from collections import defaultdict
from itertools import combinations

from simulation_observer import strongly_connected_groups


def movement_unchanged(previous, current):
    fields = ("node", "total_distance", "position_x_millimeters", "position_y_millimeters")
    return previous is not None and all(previous.get(field) == current.get(field) for field in fields)


def dependency_graph(snapshot):
    directed = defaultdict(set)
    undirected = defaultdict(set)
    for dependency in snapshot["wait_dependencies"]:
        origin = f"vehicle:{dependency['oht_id']}"
        resource = dependency.get("resource_name")
        destinations = [f"vehicle:{number}" for number in dependency["blocking_vehicle_ids"]]
        edges = [(origin, resource)] + [(resource, destination) for destination in destinations] if resource else [
            (origin, destination) for destination in destinations]
        for first, second in edges:
            directed[first].add(second)
            # 후보 묶음에는 120초 이상 대기 차량의 차단 관계만 사용한다.
            # 자원 내부의 여러 대안은 OR 관계일 수 있어 이 연결만으로 확정하지 않는다.
            undirected[first].add(second)
            undirected[second].add(first)
    return directed, undirected


def connected_components(graph):
    seen = set()
    for initial in graph:
        if initial in seen:
            continue
        group, pending = set(), [initial]
        while pending:
            current = pending.pop()
            if current in group:
                continue
            group.add(current)
            pending.extend(graph.get(current, ()))
        seen.update(group)
        yield group


class CandidateReviewer:
    def __init__(self, graph, regions):
        self.graph = graph
        self.regions = regions
        self.incoming = defaultdict(set)
        self.outgoing = defaultdict(set)
        for first, second in graph["edges"]:
            self.outgoing[first].add(second)
            self.incoming[second].add(first)
        self.node_regions = defaultdict(set)
        self.rings = {}
        for name, region in regions.items():
            for node in region["nodes"]:
                self.node_regions[node].add(name)
            if name.startswith("고리_"):
                self.rings[name] = set(region["nodes"])

    def type_evidence(self, group, waiting, vehicles, dependencies, directed, previous):
        evidence = []
        observed_cycles = [component for component in strongly_connected_groups(directed)
                           if len([vertex for vertex in component if vertex.startswith("vehicle:")]) >= 2
                           and any(f"vehicle:{number}" in component for number in waiting)]
        cycle_vehicles = [{int(vertex.split(":")[1]) for vertex in component if vertex.startswith("vehicle:")}
                          for component in observed_cycles]
        group_rings = {name for number in group for name in vehicles[number].get("physical_region_names", self.node_regions[vehicles[number]["node"]])
                       if name.startswith("고리_")}
        if observed_cycles and len(group_rings) == 1:
            evidence.append({"type": "TYPE I", "name": "단일 루프 포화", "support": "단일 관리 고리와 연결된 순환 의존 후보",
                "missing": ["실제 단일 고리 경로와 모든 진출 선택지의 진행 불능 검증"], "support_level": "부분 근거"})
        for first, second in combinations(sorted(group_rings), 2):
            first_members = {number for number in group if vehicles[number].get("region_reference_node", vehicles[number]["node"]) in self.rings[first] - self.rings[second]}
            second_members = {number for number in group if vehicles[number].get("region_reference_node", vehicles[number]["node"]) in self.rings[second] - self.rings[first]}
            if any(component & first_members and component & second_members for component in cycle_vehicles):
                evidence.append({"type": "TYPE II", "name": "복수 루프 간 상호 차단", "support": "서로 다른 고리에 있는 차량 사이의 순환 의존 후보",
                    "rings": [first, second], "missing": ["경계부 양방향 진입·진출 차단과 자원 해제 선택지 검증"], "support_level": "부분 근거"})
                break
        merge_nodes = {vehicles[number]["_merge_wait"][0] for number in waiting if vehicles[number].get("_merge_wait")}
        merge_nodes = {node for node in merge_nodes if len(self.incoming[node]) >= 2}
        if len(merge_nodes) >= 2:
            closed_merge_cycle = any(len({vertex for vertex in component if vertex.startswith("merge:")}) >= 2
                                     for component in observed_cycles)
            evidence.append({"type": "TYPE III", "name": "분기·합류 구간 상호 대기",
                "support": "둘 이상의 실제 합류부 선두 대기가 같은 차단 집단으로 연결됨",
                "merge_nodes": sorted(merge_nodes), "circular_return_observed": closed_merge_cycle,
                "missing": ["하류 대기 관계가 상류 이동으로 되돌아오는 순환 입증", "선두 차량의 대체 진출 가능성 검증"],
                "support_level": "부분 근거"})
        full_exits = [dependency for dependency in dependencies if dependency["cause"] == "merge_permission"
                      and (dependency.get("details") or {}).get("outgoing_segment_full")]
        if full_exits and (len(waiting) >= 2):
            earlier_full = []
            if previous:
                earlier_full = [dependency for dependency in previous["wait_dependencies"] if dependency["cause"] == "merge_permission"
                    and dependency.get("resource_name") in {item["resource_name"] for item in full_exits}
                    and (dependency.get("details") or {}).get("outgoing_segment_full")]
            evidence.append({"type": "TYPE IV", "name": "출구 차단에 따른 정체 확산",
                "support": "출구 만차와 연결된 후속 장기 대기 집단이 함께 관측됨",
                "also_present_in_previous_snapshot": bool(earlier_full),
                "missing": ["출구 차단 이후 대기열 확산의 시간 순서", "확산 대기열이 원래 출구의 배출을 다시 막는 관계", "해소 경로 검증"],
                "support_level": "부분 근거"})
        return evidence

    def review(self, snapshot, previous=None):
        vehicles = {vehicle["oht_id"]: vehicle for vehicle in snapshot["vehicles"]}
        previous_vehicles = {vehicle["oht_id"]: vehicle for vehicle in previous["vehicles"]} if previous else {}
        long_waiters = {number for number, vehicle in vehicles.items() if vehicle["waiting_duration_seconds"] >= 120
                        and vehicle.get("_stall_next") is not None and vehicle.get("_stuck_since") is not None}
        selected_dependencies = [dependency for dependency in snapshot["wait_dependencies"] if dependency["oht_id"] in long_waiters]
        directed, undirected = dependency_graph({"wait_dependencies": selected_dependencies})
        for number in long_waiters:
            undirected.setdefault(f"vehicle:{number}", set())
        candidates = []
        for component in connected_components(undirected):
            group = {int(vertex.split(":")[1]) for vertex in component if vertex.startswith("vehicle:")}
            waiting = group & long_waiters
            if not waiting:
                continue
            anchor = max(waiting, key=lambda number: (vehicles[number]["waiting_duration_seconds"], -number))
            lead = vehicles[anchor]
            corroborated = [number for number in sorted(waiting)
                if previous and snapshot["simulation_second"] - previous["simulation_second"] >= 120
                and previous_vehicles.get(number, {}).get("waiting_duration_seconds", 0) > 0
                and movement_unchanged(previous_vehicles.get(number), vehicles[number])]
            dependencies = [dependency for dependency in selected_dependencies if dependency["oht_id"] in waiting]
            maximum_wait = lead["waiting_duration_seconds"]
            priority = 1 if maximum_wait >= 600 and anchor in corroborated and dependencies else 2 if anchor in corroborated else 3
            names = sorted({name for number in group for name in self.node_regions[vehicles[number]["node"]]})
            identity = f"차량{anchor:05d}_노드{lead['node']}_대기시작{snapshot['simulation_second'] - maximum_wait:010.1f}초"
            candidates.append({"candidate_key": identity, "priority": priority,
                "priority_reason": "장기 미해소·동일 차량 진행 정지·차단 관계 확인" if priority == 1 else "대기 지속 관측·추가 구조 검토 필요",
                "simulation_second": snapshot["simulation_second"], "oht_id": anchor, "node": lead["node"],
                "observed_wait_start_second": snapshot["simulation_second"] - maximum_wait,
                "maximum_waiting_duration_seconds": maximum_wait, "waiting_vehicle_ids": sorted(waiting),
                "related_vehicle_ids": sorted(group), "corroborated_stationary_vehicle_ids": corroborated,
                "comparison_snapshot_second": previous["simulation_second"] if previous else None,
                "region_names": names, "wait_dependencies": dependencies,
                "ppt_type_evidence": self.type_evidence(group, waiting, vehicles, dependencies, directed, previous),
                "verdict": "우선 검토 후보·교착 미확정", "probability": None,
                "note": "우선순위는 검토 순서이며 발생 확률이 아님. 구역별 중복 대신 실제 차량·차단 자원 연결 집단으로 묶음"})
        return sorted(candidates, key=lambda item: (item["priority"], -item["maximum_waiting_duration_seconds"], -len(item["corroborated_stationary_vehicle_ids"])))
