"""관측 상태의 닫힌 진행 불능 구조에 대한 보수적인 충분조건 검사.

불충분한 기록은 미확정이며, 미확정을 '교착 없음'으로 해석하지 않는다.
기간만으로 확정하지 않는다. 제공 소스의 안전거리나 제어를 변경하지 않는다.
"""

from collections import defaultdict
from itertools import combinations
import math

from simulation_observer import strongly_connected_groups


SUPPORTED_SIMULATOR_SHA256 = "edded09d560d361d900db82171cbecd86c2ee3439ce835dee1b14af156c304de"


class DeadlockProofInspector:
    def __init__(self, graph, regions):
        self.graph = graph
        self.regions = regions
        self.outgoing = defaultdict(list)
        self.incoming = defaultdict(list)
        for origin, destination in graph["edges"]:
            self.outgoing[origin].append(destination)
            self.incoming[destination].append(origin)
        self.edge_lengths = {(link["a"], link["b"]): link["length"] for link in graph["links"]}
        self.rings = {name: set(region["nodes"]) for name, region in regions.items() if name.startswith("고리_")}
        self.previous_vehicles = {}
        self.previous_second = None
        self.exit_history = {}

    def stationary(self, vehicle, snapshot):
        now = snapshot["simulation_second"]
        node = vehicle["node"]
        if vehicle.get("_stall_next") is None or vehicle.get("_stuck_since") is None:
            return False
        if vehicle.get("waiting_duration_seconds", 0) < 1 or vehicle.get("_cur_speed", 1) != 0:
            return False
        if snapshot["NODE_HOLDER"].get(node) != vehicle["oht_id"]:
            return False
        if vehicle.get("_seg_start_t", now) + vehicle.get("_seg_profile", {}).get("dur", 1) > now + 1e-7:
            return False
        x, y = self.graph["nodes"][node]
        return math.hypot(vehicle["position_x_millimeters"] - x,
                          vehicle["position_y_millimeters"] - y) < 1e-4

    def physical_dependencies(self, snapshot):
        """실제 DIST 범위와 노드 점유를 제어 함수 호출 없이 재계산."""
        holders = snapshot["NODE_HOLDER"]
        dependencies = defaultdict(list)
        for vehicle in snapshot["vehicles"]:
            if vehicle.get("_stall_next") is None:
                continue
            next_node = vehicle["_stall_next"]
            holder = holders.get(next_node)
            if holder is not None and holder != vehicle["oht_id"]:
                dependencies[vehicle["oht_id"]].append({"blocker": holder, "cause": "node_lock", "node": next_node})
            path = vehicle.get("remaining_path", [])
            if not vehicle.get("remaining_path_is_complete") or len(path) < 2 or path[0] != vehicle["node"] or path[1] != next_node:
                continue
            cumulative_distance = 0.0
            for index in range(1, len(path) - 1):
                edge = (path[index], path[index + 1])
                if edge not in self.edge_lengths:
                    break
                cumulative_distance += self.edge_lengths[edge]
                if cumulative_distance >= 1150.0:
                    break
                holder = holders.get(path[index + 1])
                if holder is not None and holder != vehicle["oht_id"]:
                    dependencies[vehicle["oht_id"]].append({"blocker": holder, "cause": "follow_distance",
                        "node": path[index + 1], "distance_from_next_node_millimeters": cumulative_distance})
        return dependencies

    @staticmethod
    def groups(dependencies, eligible):
        graph = {str(number): {str(edge["blocker"]) for edge in dependencies.get(number, [])
                              if edge["blocker"] in eligible} for number in eligible}
        return [set(map(int, group)) for group in strongly_connected_groups(graph)]

    def fixed_loaded_route(self, vehicle, snapshot):
        """읽기 전용 관측 결과와 제공 헤드리스 운용의 검증된 불변조건.

        운반 중 작업은 재할당 불가. 현재 노드의 출구가 1개이고 폐쇄 순환의
        모든 차량이 이 조건을 만족하면, 정체 감시의 분기점 대상도 존재하지 않음.
        고리·합류 허가를 풀어도 별도로 확인한 물리적 차단은 그대로 남음.
        분기점·유휴·픽업 전 차량은 이 충분조건으로 확정하지 않음.
        """
        if not self.stationary(vehicle, snapshot):
            return False
        if vehicle.get("job_state") != "carrying" or vehicle["state"] != "to_dest":
            return False
        if len(self.outgoing[vehicle["node"]]) != 1:
            return False
        path = vehicle.get("remaining_path", [])
        if (not vehicle.get("remaining_path_is_complete") or len(path) < 2 or
                path[:2] != [vehicle["node"], vehicle["_stall_next"]] or
                path[-1] != vehicle.get("to_node") or vehicle["node"] == vehicle.get("to_node")):
            return False
        previous = self.previous_vehicles.get(vehicle["oht_id"])
        if previous is None or self.previous_second is None or snapshot["simulation_second"] <= self.previous_second:
            return False
        names = ("node", "job_id", "to_node", "remaining_path", "total_distance", "_seg_start_t")
        return all(previous.get(name) == vehicle.get(name) for name in names) and self.stationary(previous, {
            **snapshot, "simulation_second": self.previous_second})

    def outgoing_segment(self, merge_node):
        if len(self.outgoing[merge_node]) != 1:
            return []
        node = self.outgoing[merge_node][0]
        result, seen = [node], {node}
        while len(self.incoming[node]) == 1 and len(self.outgoing[node]) == 1:
            node = self.outgoing[node][0]
            if node in seen or len(self.incoming[node]) != 1 or len(self.outgoing[node]) != 1:
                break
            seen.add(node)
            result.append(node)
        return result

    def exit_observations(self, snapshot, dependencies):
        vehicles = {vehicle["oht_id"]: vehicle for vehicle in snapshot["vehicles"]}
        reverse = defaultdict(set)
        for number, edges in dependencies.items():
            for edge in edges:
                reverse[edge["blocker"]].add(number)
        observations = {}
        merge_nodes = {vehicle["_merge_wait"][0] for vehicle in vehicles.values() if vehicle.get("_merge_wait")}
        for merge_node in merge_nodes:
            nodes = self.outgoing_segment(merge_node)
            holders = {snapshot["NODE_HOLDER"].get(node) for node in nodes}
            if not nodes or None in holders or not all(self.stationary(vehicles[number], snapshot) for number in holders):
                continue
            reached, stack = set(holders), list(holders)
            while stack:
                for waiter in reverse.get(stack.pop(), ()):
                    if waiter not in reached:
                        reached.add(waiter)
                        stack.append(waiter)
            observations[merge_node] = {"exit_vehicle_ids": sorted(holders),
                "upstream_waiting_vehicle_ids": sorted(reached - holders), "outgoing_nodes": nodes,
                "observed_second": snapshot["simulation_second"]}
        return observations

    def classify(self, group, vehicles, dependencies, exits):
        types = []
        group_regions = {name for name, region in self.regions.items()
                         if any(vehicles[number]["node"] in region["nodes"] for number in group)}
        actual_groups = self.groups(dependencies, group)
        for first, second in combinations(self.rings, 2):
            first_members = {number for number in group if vehicles[number]["node"] in self.rings[first] - self.rings[second]}
            second_members = {number for number in group if vehicles[number]["node"] in self.rings[second] - self.rings[first]}
            if first_members and second_members and any(component & first_members and component & second_members for component in actual_groups):
                types.append({"type": "TYPE II", "evidence": "서로 다른 두 고리의 차량을 잇는 닫힌 실제 대기 관계",
                              "rings": [first, second]})
                break
        if not types and len(actual_groups) == 1 and actual_groups[0] == group and all(
                len({edge["blocker"] for edge in dependencies[number] if edge["blocker"] in component}) == 1
                for component in actual_groups for number in component):
            types.append({"type": "TYPE I", "evidence": "단일 순환 대기 고리와 진행 불능 구조가 함께 확인됨"})
        verified_merge_waits = set()
        for component in actual_groups:
            nodes = {vehicles[number]["_merge_wait"][0] for number in component if vehicles[number].get("_merge_wait")}
            nodes = {node for node in nodes if len(self.incoming[node]) >= 2}
            if len(nodes) >= 2:
                verified_merge_waits.update(nodes)
        if len(verified_merge_waits) >= 2:
            types.append({"type": "TYPE III", "evidence": "둘 이상의 실제 합류 허가 대기와 그 사이의 닫힌 진행 불능 관계",
                          "merge_nodes": sorted(verified_merge_waits)})
        for node, current in exits.items():
            previous = self.exit_history.get(node)
            if previous is None or previous["observed_second"] >= current["observed_second"]:
                continue
            new_waiters = set(current["upstream_waiting_vehicle_ids"]) - set(previous["upstream_waiting_vehicle_ids"])
            if (group & set(current["exit_vehicle_ids"]) and group & new_waiters and
                    any(component & set(current["exit_vehicle_ids"]) and component & new_waiters for component in actual_groups)):
                types.append({"type": "TYPE IV", "evidence": "선행 출구 차단, 이후 상류 대기 증가, 출구 배출을 다시 막는 폐쇄 대기 관계",
                    "merge_node": node, "exit_blockage_observed_second": previous["observed_second"],
                    "feedback_observed_second": current["observed_second"]})
        return types, sorted(group_regions)

    def inspect(self, snapshot):
        vehicles = {vehicle["oht_id"]: vehicle for vehicle in snapshot["vehicles"]}
        dependencies = self.physical_dependencies(snapshot)
        stationary = {number for number, vehicle in vehicles.items() if self.stationary(vehicle, snapshot)}
        # 현재 노드 외의 예약까지 쥔 차량은 예약 해제 가능성을 별도 검증해야 하므로 제외.
        held_count = defaultdict(int)
        for number in snapshot["NODE_HOLDER"].values():
            held_count[number] += 1
        stationary = {number for number in stationary if held_count[number] == 1}
        potential = {number: [{"blocker": snapshot["NODE_HOLDER"].get(node), "cause": "node_lock", "node": node}
                             for node in self.outgoing[vehicles[number]["node"]]
                             if snapshot["NODE_HOLDER"].get(node) in stationary]
                     for number in stationary}
        proofs = []
        used_groups = set()
        for group in self.groups(potential, stationary):
            if all(self.outgoing[vehicles[number]["node"]] and all(
                    snapshot["NODE_HOLDER"].get(node) in group for node in self.outgoing[vehicles[number]["node"]])
                    for number in group):
                proofs.append((group, "모든 순방향 이동이 같은 집단의 점유 노드에 막힌 닫힌 물리 구조"))
                used_groups.add(frozenset(group))
        fixed = {number for number in stationary if self.fixed_loaded_route(vehicles[number], snapshot)}
        for group in self.groups(dependencies, fixed):
            if frozenset(group) not in used_groups:
                proofs.append((group, "재할당 불가 운반 차량·비분기 위치·고정 진행 경로에서 노드/거리 차단이 닫힌 구조"))
                used_groups.add(frozenset(group))
        exits = self.exit_observations(snapshot, dependencies)
        results = []
        for group, proof_kind in proofs:
            types, region_names = self.classify(group, vehicles, dependencies, exits)
            results.append({"verdict": "구조적 교착 충분조건 충족", "confirmed": True,
                "simulation_second": snapshot["simulation_second"], "proof_kind": proof_kind,
                "vehicle_ids": sorted(group), "region_names": region_names, "confirmed_types": types,
                "physical_wait_edges": [{"oht_id": number, **edge} for number in sorted(group)
                                        for edge in dependencies.get(number, []) if edge["blocker"] in group],
                "vehicle_states": [vehicles[number] for number in sorted(group)],
                "all_outgoing_node_holders": {str(number): {node: snapshot["NODE_HOLDER"].get(node)
                    for node in self.outgoing[vehicles[number]["node"]]} for number in sorted(group)},
                "scope": "제공 버전 2 헤드리스 운용. 외부 수동 차단·차량 제거·설정 변경 제외",
                "not_a_duration_threshold_verdict": True,
                "type_classification_complete": bool(types)})
        self.previous_vehicles = vehicles
        self.previous_second = snapshot["simulation_second"]
        for node in set(self.exit_history) - set(exits):
            del self.exit_history[node]
        for node, observation in exits.items():
            self.exit_history.setdefault(node, observation)
        return results
