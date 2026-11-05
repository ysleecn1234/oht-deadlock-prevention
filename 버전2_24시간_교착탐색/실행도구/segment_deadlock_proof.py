"""연속 위치 모델에서 개별 안전 주행의 진행 불능 충분조건을 읽기 전용 검사.

순환 대기나 관측 종료를 증명으로 쓰지 않는다. 정상 서비스·이동·재할당 가능
차량은 증명 집합에서 제외한다. 모든 가까운 분기 선택이 같은 정지 집합의
차량에 막혀야 한다. 정보가 불완전하거나 외부 해제 여지가 있으면 확정하지 않는다.
"""
from collections import deque, defaultdict

from simulation_observer import strongly_connected_groups


class SegmentDeadlockProofInspector:
    def __init__(self, graph, regions):
        self.graph = graph
        self.regions = regions
        self.history = deque(maxlen=301)
        outgoing = defaultdict(set)
        incoming = defaultdict(set)
        for link in graph["links"]:
            outgoing[link["a"]].add(link["b"])
            incoming[link["b"]].add(link["a"])
        self.control_nodes = {node for node in graph["nodes"] if len(outgoing[node]) > 1 or len(incoming[node]) > 1}

    def inspect(self, snapshot):
        vehicles = {vehicle["oht_id"]: vehicle for vehicle in snapshot["vehicles"]}
        remaining = {number for number, vehicle in vehicles.items()
                     if vehicle.get("job_state") == "carrying"
                     and vehicle.get("_cur_speed", 1) <= 1e-5
                     and vehicle.get("_stall_next") is not None
                     and vehicle.get("is_route_active")
                     and vehicle.get("movement_options_are_complete")
                     and vehicle.get("movement_options")}
        while True:
            eligible = {number for number in remaining if all(
                option["blocking_vehicle_id"] in remaining
                for option in vehicles[number]["movement_options"])}
            if eligible == remaining:
                break
            remaining = eligible
        dependencies = {number: {option["blocking_vehicle_id"] for option in vehicles[number]["movement_options"]}
                        for number in remaining}
        proofs = []
        for group in strongly_connected_groups(dependencies):
            if len(group) < 2:
                continue
            if any(not dependencies[number] <= group for number in group):
                continue
            shared_rings = None
            all_rings = set()
            for number in group:
                rings = set(vehicles[number].get("physical_ring_numbers", ()))
                all_rings.update(rings)
                shared_rings = rings if shared_rings is None else shared_rings & rings
            types = []
            physical_edges = defaultdict(set)
            for number in group:
                for option in vehicles[number]["movement_options"]:
                    for index in option.get("link_indices", ()):
                        if 0 <= index < len(self.graph["links"]):
                            link = self.graph["links"][index]
                            physical_edges[link["a"]].add(link["b"])
            edge_nodes = set(physical_edges) | {node for destinations in physical_edges.values() for node in destinations}
            is_single_loop = (bool(edge_nodes) and all(len(physical_edges[node]) == 1 for node in edge_nodes)
                              and any(set(component) == edge_nodes for component in strongly_connected_groups(physical_edges)))
            saturated_rings = {number for number in shared_rings or ()
                               if 0 <= number < len(snapshot.get("RING_MEMBERS", []))
                               and len(snapshot["RING_MEMBERS"][number]) >=
                               (self.regions.get(f"고리_{number}", {}).get("cap") or float("inf"))}
            if is_single_loop:
                types.append({"type": "TYPE I", "name": "단일 루프 포화",
                              "evidence": "단일 물리 루프에서 모든 개별 진출 선택이 정지 집합 안에서 차단",
                              "saturated_managed_rings": sorted(saturated_rings)})
            # 중첩된 관리 고리 이름 두 개만으로 복수 루프라고 부르지 않는다.
            # 각 고리에만 속한 차량과 양방향 차단 관계가 모두 필요하다.
            mutually_blocked_rings = []
            for first_ring in sorted(all_rings):
                for second_ring in sorted(all_rings):
                    if first_ring >= second_ring:
                        continue
                    first_members = {number for number in group if first_ring in vehicles[number].get("physical_ring_numbers", ())
                                     and second_ring not in vehicles[number].get("physical_ring_numbers", ())}
                    second_members = {number for number in group if second_ring in vehicles[number].get("physical_ring_numbers", ())
                                      and first_ring not in vehicles[number].get("physical_ring_numbers", ())}
                    if (any(dependencies[number] & second_members for number in first_members)
                            and any(dependencies[number] & first_members for number in second_members)):
                        mutually_blocked_rings.append([first_ring, second_ring])
            if mutually_blocked_rings:
                types.append({"type": "TYPE II", "name": "복수 루프 간 상호 차단",
                              "evidence": "각각 다른 고리에 있는 차량의 양방향 물리 차단과 모든 진출 선택 폐쇄",
                              "ring_pairs": mutually_blocked_rings})
            merges = {vehicles[number]["_merge_wait"][0] for number in group if vehicles[number].get("_merge_wait")}
            merges.update((record.get("details") or {}).get("node") for record in snapshot.get("wait_dependencies", ())
                          if record["oht_id"] in group and (record.get("details") or {}).get("node") in self.control_nodes)
            if len(merges) >= 2:
                types.append({"type": "TYPE III", "name": "분기·합류 구간 상호 대기",
                              "evidence": "둘 이상 합류부의 대기와 모든 물리 진출 차단이 함께 입증"})
            full_exits = [record for record in snapshot["wait_dependencies"]
                          if record["oht_id"] in group and (record.get("details") or {}).get("outgoing_segment_full")]
            propagated_exits = []
            for record in full_exits:
                exit_node = record["details"].get("merge_node")
                for earlier in self.history:
                    elapsed_seconds = snapshot["simulation_second"] - earlier["simulation_second"]
                    if not 60 <= elapsed_seconds <= 300 or exit_node not in earlier["exit_waiting_members"]:
                        continue
                    earlier_members = set(earlier["exit_waiting_members"][exit_node])
                    if earlier_members and earlier_members <= group and len(group - earlier_members) >= 3:
                        propagated_exits.append({"merge_node": exit_node,
                            "earlier_simulation_second": earlier["simulation_second"],
                            "earlier_vehicle_ids": sorted(earlier_members), "current_vehicle_ids": sorted(group)})
                        break
            if propagated_exits:
                types.append({"type": "TYPE IV", "name": "출구 차단에 따른 정체 확산",
                              "evidence": "같은 출구의 선행 차단·상류 대기 차량 증가·현재 모든 물리 진출 차단",
                              "propagation": propagated_exits})
            proofs.append({"confirmed": True, "verdict": "구간 주행 모델의 구조적 교착 충분조건 충족",
                "simulation_second": snapshot["simulation_second"], "vehicle_ids": sorted(group),
                "confirmed_types": types, "proof_kind": "재할당 불가 정지 차량의 모든 가까운 진출 선택 폐쇄",
                "movement_options": {str(number): vehicles[number]["movement_options"] for number in group},
                "progress": {str(number): vehicles[number]["total_distance"] for number in group},
                "not_a_duration_threshold_verdict": True,
                "scope": "개별 차간거리 주행 모델의 충분조건. 집단 동시 출발 제어·실제 팹 교착 증명은 아님"})
        # 출구 대기자를 거슬러 올라가는 같은 차단 사슬만 비교한다. 전역
        # 대기 대수의 증가는 TYPE IV의 시간 순서 근거가 될 수 없다.
        records = snapshot.get("wait_dependencies", [])
        blocked_by = {record["oht_id"]: set(record.get("blocking_vehicle_ids", ())) for record in records}
        exit_waiting_members = {}
        for record in records:
            if not (record.get("details") or {}).get("outgoing_segment_full"):
                continue
            exit_node = record["details"].get("merge_node")
            if exit_node is None:
                continue
            upstream_members = {record["oht_id"]}
            while True:
                additional = {number for number, blockers in blocked_by.items() if blockers & upstream_members}
                if additional <= upstream_members:
                    break
                upstream_members |= additional
            exit_waiting_members.setdefault(exit_node, set()).update(upstream_members)
        self.history.append({"simulation_second": snapshot["simulation_second"],
                             "exit_waiting_members": exit_waiting_members})
        return proofs
