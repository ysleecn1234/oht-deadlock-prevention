"""실제 링크 위치와 T 정차점을 사용하는 가감속 주행시간 계산.

2026-11-06: 제공 버전2의 가감속 식, JUNCTION_SPEED, LINK_PENALTY를
사용한다. 원본의 최소 1ms 시간 및 미세거리 속도 덮어쓰기는 제외한다.
속도를 상수로 나눈 ETA를 쓰지 않는다. 미래 교통 대기는 예측하지
않으며, 충돌 방지는 주행 계층의 차간거리/통과 점유 검사가 별도로 담당한다.
"""
from collections import defaultdict, deque, OrderedDict
import math

import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra


class TravelTimeModelError(RuntimeError):
    pass


def get_segment_profile(simulator, start_speed, desired_exit_speed, speed_limit, distance_mm):
    """원본과 동일한 사다리꼴 식. 1ms 시간 하한·미세거리 속도 덮어쓰기는 제외."""
    if start_speed > speed_limit + 1e-5:
        raise TravelTimeModelError("현재 속도가 링크 제한속도를 초과함")
    start_speed = max(start_speed, 0.0)
    if distance_mm <= 0:
        return {"v0":start_speed,"v_peak":start_speed,"v_end":start_speed,
                "t_acc":0.0,"t_cruise":0.0,"t_dec":0.0,"dur":0.0}
    acceleration = simulator.OHT_ACCEL
    braking = simulator.OHT_DECEL
    minimum_exit = math.sqrt(max(start_speed**2 - 2*braking*distance_mm,0.0))
    maximum_exit = min(math.sqrt(start_speed**2 + 2*acceleration*distance_mm),speed_limit)
    exit_speed = min(max(desired_exit_speed,minimum_exit,0.0),maximum_exit)
    acceleration_distance = max(speed_limit**2-start_speed**2,0.0)/(2*acceleration)
    braking_distance = max(speed_limit**2-exit_speed**2,0.0)/(2*braking)
    if acceleration_distance + braking_distance <= distance_mm:
        peak_speed = speed_limit
        cruise_seconds = (distance_mm-acceleration_distance-braking_distance)/peak_speed
    else:
        peak_speed = math.sqrt(max((2*acceleration*braking*distance_mm + braking*start_speed**2 +
                                  acceleration*exit_speed**2)/(acceleration+braking),0.0))
        peak_speed = max(peak_speed,start_speed,exit_speed)
        cruise_seconds = 0.0
    acceleration_seconds = (peak_speed-start_speed)/acceleration
    braking_seconds = (peak_speed-exit_speed)/braking
    return {"v0":start_speed,"v_peak":peak_speed,"v_end":exit_speed,
            "t_acc":acceleration_seconds,"t_cruise":cruise_seconds,"t_dec":braking_seconds,
            "dur":acceleration_seconds+cruise_seconds+braking_seconds}


def get_travel_plan(simulator, links, start_offset_mm=0.0,
                    start_speed_mm_per_second=0.0, end_offset_mm=None,
                    terminal_speed_mm_per_second=0.0):
    """정해진 경로의 최단 자유 주행 프로파일. 앞차/허가 대기시간 제외.

    목표 T까지의 부분 링크 길이를 먼저 반영하고 정차 감속을 뒤에서 앞으로
    전파한다. 이후 실제 시작 속도에서 가능한 가속을 앞에서 뒤로 계산한다.
    """
    if not links:
        return {"duration_seconds": 0.0, "weighted_seconds": 0.0,
                "profiles": [], "lengths_mm": [], "exit_speeds_mm_per_second": []}
    lengths_mm = [float(simulator.LINK_LEN[index]) for index in links]
    lengths_mm[0] -= start_offset_mm
    if end_offset_mm is not None:
        lengths_mm[-1] += end_offset_mm - simulator.LINK_LEN[links[-1]]
    if min(lengths_mm) < -1e-6:
        raise TravelTimeModelError("목적 정차점이 현재 위치 뒤에 있음")
    lengths_mm = [max(0.0, length_mm) for length_mm in lengths_mm]
    boundary_speeds = [0.0] * (len(links) + 1)
    boundary_speeds[-1] = terminal_speed_mm_per_second
    for number in range(len(links) - 1, -1, -1):
        current = links[number]
        entry_limit = simulator.LINK_CAP[current]
        if number:
            entry_limit = min(entry_limit, simulator.JUNCTION_SPEED.get(
                (links[number - 1], current), min(simulator.LINK_CAP[links[number - 1]], entry_limit)))
        boundary_speeds[number] = min(entry_limit, math.sqrt(
            boundary_speeds[number + 1] ** 2 + 2 * simulator.OHT_DECEL * lengths_mm[number]))
    if start_speed_mm_per_second ** 2 > boundary_speeds[0] ** 2 + 2*simulator.OHT_DECEL*1e-6:
        raise TravelTimeModelError("현재 속도로는 경로의 속도 제한/정차점을 지킬 수 없음")
    speed_mm_per_second = start_speed_mm_per_second
    profiles = []
    for number, link_index in enumerate(links):
        if lengths_mm[number] <= 1e-9:
            profile = {"v0": speed_mm_per_second, "v_peak": speed_mm_per_second,
                       "v_end": speed_mm_per_second, "t_acc": 0.0, "t_cruise": 0.0,
                       "t_dec": 0.0, "dur": 0.0}
        else:
            profile = get_segment_profile(simulator, speed_mm_per_second, boundary_speeds[number + 1],
                simulator.LINK_CAP[link_index], lengths_mm[number])
        speed_mm_per_second = profile["v_end"]
        profiles.append(profile)
    numerical_terminal_adjustment_mm = max(0.0,(speed_mm_per_second**2-terminal_speed_mm_per_second**2)/(2*simulator.OHT_DECEL))
    if abs(speed_mm_per_second - terminal_speed_mm_per_second) > 1e-4:
        if numerical_terminal_adjustment_mm > 1e-6 or speed_mm_per_second < terminal_speed_mm_per_second:
            raise TravelTimeModelError(f"목적지의 요구 속도에 도달하지 못함: 잔여 속도={speed_mm_per_second}, 추가 제동거리={numerical_terminal_adjustment_mm}")
        # 끝점 거리의 부동소수점 뺄셈 오차 안에서 남은 제동도 시간에
        # 포함한다. 속도를 순간적으로 0으로 덮어쓰거나 1ms를 더하지 않는다.
        correction_seconds = (speed_mm_per_second-terminal_speed_mm_per_second)/simulator.OHT_DECEL
        profiles[-1]["t_dec"] += correction_seconds
        profiles[-1]["dur"] += correction_seconds
        profiles[-1]["v_end"] = terminal_speed_mm_per_second
    return {"duration_seconds": sum(profile["dur"] for profile in profiles),
        "weighted_seconds": sum(profile["dur"] * simulator.LINK_PENALTY[index]
                                for index, profile in zip(links, profiles)),
        "profiles": profiles, "lengths_mm": lengths_mm, "boundary_speed_limits": boundary_speeds,
        "numerical_terminal_adjustment_mm": numerical_terminal_adjustment_mm,
        "exit_speeds_mm_per_second": [profile["v_end"] for profile in profiles]}


class SegmentTravelTimeCalculator:
    """속도 상태 그래프에 실제 출발 위치와 정차점 연결을 추가한다.

    같은 링크라도 실제 진입 속도가 다르면 다른 상태다. 원본의 가속 도달
    속도 clamp 결과를 다음 링크에 그대로 넘겨, 아직 내지 못하는 속도로
    순간적으로 바꾸지 않는다. 모든 배정 조합에 A*를 개별 호출하지 않는다.
    """
    def __init__(self, simulator):
        self.simulator = simulator
        self.link_indices_by_id = {link["id"]: index for index, link in enumerate(simulator.graph_data["links"])}
        self.states = []
        self.state_indices = {}
        self.states_by_link = defaultdict(list)
        self.edge_origins = []
        self.edge_destinations = []
        self.edge_links = []
        self.edge_seconds = []
        self.terminal_path_cache = OrderedDict()
        self.matrix_cache = OrderedDict()
        pending = deque()

        def register(link_index, speed_mm_per_second):
            key = (link_index, round(float(speed_mm_per_second), 8))
            if key not in self.state_indices:
                self.state_indices[key] = len(self.states)
                self.states_by_link[link_index].append(len(self.states))
                self.states.append(key)
                pending.append(key)
            return self.state_indices[key]

        for link_index in range(len(simulator.LINK_LEN)):
            register(link_index, 0.0)
        while pending:
            link_index, speed_mm_per_second = pending.popleft()
            origin = self.state_indices[(link_index, speed_mm_per_second)]
            for following in simulator.OUT_LINKS[simulator.LINK_B[link_index]]:
                exit_speed = simulator.JUNCTION_SPEED.get((link_index, following),
                    min(simulator.LINK_CAP[link_index], simulator.LINK_CAP[following]))
                if speed_mm_per_second ** 2 > exit_speed ** 2 + 2 * simulator.OHT_DECEL * (simulator.LINK_LEN[link_index] + 1e-6):
                    continue
                profile = get_segment_profile(simulator, speed_mm_per_second, exit_speed,
                    simulator.LINK_CAP[link_index], simulator.LINK_LEN[link_index])
                destination = register(following, profile["v_end"])
                self.edge_origins.append(origin)
                self.edge_destinations.append(destination)
                self.edge_links.append(link_index)
                self.edge_seconds.append(profile["dur"])
            if len(self.states) > 200000:
                raise TravelTimeModelError("속도 상태 수가 검증 범위를 초과함")
        self.edge_origins = np.asarray(self.edge_origins, dtype=int)
        self.edge_destinations = np.asarray(self.edge_destinations, dtype=int)
        self.edge_links = np.asarray(self.edge_links, dtype=int)
        self.edge_seconds = np.asarray(self.edge_seconds)

    def get_target_endpoints(self, target):
        kind, identity = target
        simulator = self.simulator
        if kind == "port":
            port = simulator.PORT_BY_ID[identity]
            return [(self.link_indices_by_id[port["parent_link"]], float(port["rel_dist"]))]
        return [(index, simulator.LINK_LEN[index]) for index in simulator.IN_LINKS[identity]]

    def get_terminal_paths(self, target):
        if target in self.terminal_path_cache:
            self.terminal_path_cache.move_to_end(target)
            return self.terminal_path_cache[target]
        simulator = self.simulator
        paths = []
        pending = [((index,), offset_mm) for index, offset_mm in self.get_target_endpoints(target)]
        seen = set()
        while pending:
            links, end_offset_mm = pending.pop()
            if links in seen:
                continue
            seen.add(links)
            paths.append((links, end_offset_mm))
            # 정지거리 이내의 선행 링크까지 terminal 연결을 확장한다.
            # T가 링크 시작부에 있으면 그 전 링크에서의 감속도 계산한다.
            lengths_mm = sum(simulator.LINK_LEN[index] for index in links[:-1]) + end_offset_mm
            if lengths_mm + 1e-6 >= simulator.LINK_CAP[links[0]] ** 2 / (2 * simulator.OHT_DECEL):
                continue
            for previous in simulator.IN_LINKS[simulator.LINK_A[links[0]]]:
                if previous not in links:
                    pending.append(((previous,) + links, end_offset_mm))
        self.terminal_path_cache[target] = paths
        if len(self.terminal_path_cache) > 1024:
            self.terminal_path_cache.popitem(last=False)
        return paths

    def get_matrix(self, targets, avoided_links=frozenset()):
        simulator = self.simulator
        blocked = frozenset(simulator.BLOCKED_LINKS) | frozenset(avoided_links or ())
        weights = np.asarray(simulator.LINK_PENALTY)
        key = (tuple(targets), blocked, weights.tobytes())
        if key in self.matrix_cache:
            self.matrix_cache.move_to_end(key)
            return self.matrix_cache[key]
        rows = self.edge_origins.tolist()
        columns = self.edge_destinations.tolist()
        values = (self.edge_seconds * weights[self.edge_links]).tolist()
        if blocked:
            retained = [number for number, (first, second) in enumerate(zip(rows, columns))
                        if self.states[first][0] not in blocked and self.states[second][0] not in blocked]
            rows = [rows[number] for number in retained]
            columns = [columns[number] for number in retained]
            values = [values[number] for number in retained]
        terminal_choices = []
        paths_by_link = defaultdict(list)
        for target_number, target in enumerate(targets):
            best = {}
            for links, end_offset_mm in self.get_terminal_paths(target):
                if blocked.intersection(links):
                    continue
                paths_by_link[links[0]].append((target_number, links, end_offset_mm))
                for state_index in self.states_by_link[links[0]]:
                    try:
                        plan = get_travel_plan(simulator, links,
                            start_speed_mm_per_second=self.states[state_index][1], end_offset_mm=end_offset_mm)
                    except TravelTimeModelError:
                        continue
                    if state_index not in best or plan["weighted_seconds"] < best[state_index][0]:
                        best[state_index] = (plan["weighted_seconds"], links)
            for state_index, (seconds, _links) in best.items():
                rows.append(state_index)
                columns.append(len(self.states) + target_number)
                values.append(seconds)
            terminal_choices.append(best)
        size = len(self.states) + len(targets)
        matrix = csr_matrix((values, (rows, columns)), shape=(size, size))
        result = (matrix, terminal_choices, paths_by_link, blocked)
        self.matrix_cache[key] = result
        if len(self.matrix_cache) > 4:
            self.matrix_cache.popitem(last=False)
        return result

    def get_origin_connections(self, origin, paths_by_link, blocked):
        """현재 위치에서 기본 속도 상태로 이어지는 실제 부분 주행 연결."""
        simulator = self.simulator
        current = origin.get("link_index")
        fixed = tuple(origin.get("fixed_links", ()))
        speed = origin["speed_mm_per_second"]
        start_offset_mm = origin.get("offset_mm", 0.0)
        if current is not None:
            initial = [(current, speed, start_offset_mm, (), (), fixed)]
        else:
            initial = [(index, speed, 0.0, (), (), fixed)
                       for index in (fixed[:1] or simulator.OUT_LINKS[origin["node_id"]]) if index not in blocked]
        pending = list(initial)
        seeds = {}
        direct = {}
        visited = {}
        while pending:
            link_index, entry_speed, offset_mm, prefix, prefix_seconds, forced = pending.pop()
            identity = (link_index, round(entry_speed, 8), offset_mm, forced)
            prefix_cost = sum(seconds * simulator.LINK_PENALTY[index] for index, seconds in zip(prefix, prefix_seconds))
            if prefix_cost >= visited.get(identity, math.inf):
                continue
            visited[identity] = prefix_cost
            for target_number, suffix, end_offset_mm in paths_by_link.get(link_index, ()):
                if forced and suffix[:min(len(forced), len(suffix))] != forced[:min(len(forced), len(suffix))]:
                    continue
                links = prefix + suffix
                try:
                    plan = get_travel_plan(simulator, links, start_offset_mm=start_offset_mm,
                        start_speed_mm_per_second=speed, end_offset_mm=end_offset_mm)
                except TravelTimeModelError:
                    continue
                if target_number not in direct or plan["weighted_seconds"] < direct[target_number][0]:
                    direct[target_number] = (plan["weighted_seconds"], links)
            state_index = self.state_indices.get((link_index, round(entry_speed, 8)))
            if not forced and offset_mm <= 1e-8 and state_index is not None:
                if state_index not in seeds or prefix_cost < seeds[state_index][0]:
                    seeds[state_index] = (prefix_cost, prefix)
                continue
            remaining_mm = simulator.LINK_LEN[link_index] - offset_mm
            following_links = forced[1:2] if len(forced) > 1 else simulator.OUT_LINKS[simulator.LINK_B[link_index]]
            for following in following_links:
                if following in blocked:
                    continue
                desired_exit = simulator.JUNCTION_SPEED.get((link_index, following),
                    min(simulator.LINK_CAP[link_index], simulator.LINK_CAP[following]))
                if entry_speed ** 2 > desired_exit ** 2 + 2 * simulator.OHT_DECEL * (remaining_mm + 1e-6):
                    continue
                profile = get_segment_profile(simulator, entry_speed, desired_exit, simulator.LINK_CAP[link_index], remaining_mm)
                pending.append((following, profile["v_end"], 0.0, prefix + (link_index,),
                    prefix_seconds + (profile["dur"],), forced[1:] if forced else ()))
            if len(visited) > 10000:
                raise TravelTimeModelError("실제 출발 속도의 상태 연결 탐색이 검증 범위를 초과함")
        return seeds, direct

    def get_travel_time_matrix(self, origins, targets):
        if not origins or not targets:
            return np.empty((len(origins), len(targets)))
        unique_targets = list(dict.fromkeys(targets))
        matrix, _terminal, paths_by_link, blocked = self.get_matrix(unique_targets)
        connections = [self.get_origin_connections(origin, paths_by_link, blocked) for origin in origins]
        result = np.full((len(origins), len(unique_targets)), math.inf)
        for start in range(0, len(unique_targets), 64):
            selected = list(range(start, min(start + 64, len(unique_targets))))
            distances = dijkstra(matrix.T.tocsr(), directed=True,
                                 indices=[len(self.states) + number for number in selected])
            for origin_number, (seeds, direct) in enumerate(connections):
                if seeds:
                    indices = list(seeds)
                    costs = np.asarray([seeds[index][0] for index in indices])
                    result[origin_number, selected] = np.min(distances[:, indices] + costs, axis=1)
                for target_number in selected:
                    if target_number in direct:
                        result[origin_number, target_number] = min(result[origin_number, target_number], direct[target_number][0])
        for origin_number, origin in enumerate(origins):
            if origin.get("link_index") is None and origin["speed_mm_per_second"] <= 1e-6:
                for target_number, target in enumerate(unique_targets):
                    if target == ("node", origin["node_id"]):
                        result[origin_number, target_number] = 0.0
        return result[:, [unique_targets.index(target) for target in targets]]

    def find_route(self, origin, target, avoided_links=None):
        if target == ("node", origin.get("node_id")) and origin.get("link_index") is None and origin["speed_mm_per_second"] <= 1e-6:
            return []
        matrix, terminal, paths_by_link, blocked = self.get_matrix([target], avoided_links or ())
        seeds, direct = self.get_origin_connections(origin, paths_by_link, blocked)
        distances, predecessors = dijkstra(matrix.T.tocsr(), directed=True,
            indices=len(self.states), return_predecessors=True)
        best = direct.get(0, (math.inf, None))
        for index, (cost, prefix) in seeds.items():
            total = cost + distances[index]
            if total >= best[0] or not math.isfinite(total):
                continue
            links = list(prefix)
            current = index
            while current != len(self.states):
                following = int(predecessors[current])
                if following < 0:
                    raise TravelTimeModelError("유한 거리 상태의 경로 복원 실패")
                if following == len(self.states):
                    links.extend(terminal[0][current][1])
                else:
                    links.append(self.states[current][0])
                current = following
            best = (total, tuple(links))
        return None if best[1] is None else list(best[1])
