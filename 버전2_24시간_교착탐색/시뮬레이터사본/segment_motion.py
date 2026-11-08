"""2026-11-06: 버전2의 선택 가능한 연속 위치 주행 계층.

노드 소속은 마지막 G 노드를 유지한다. 실제 위치는 링크와 mm 진행 거리다.
개별 차량의 가감속 계획을 사건 사이에서 적분한다. 앞차가 계획 직후 최대
감속해도 다음 갱신까지 안전하게 대응할 수 있는 제동 여유를 확보한다.
0.1~1초는 수치 갱신 주기이며 실차 통신 지연 사양이 아니다.
"""
from collections import defaultdict
import importlib.util
import math
from pathlib import Path


class MotionSafetyError(RuntimeError):
    pass


def get_acceleration_limit(speed_mm_per_second, distance_mm, duration_seconds,
                           braking_mm_per_second_squared, terminal_speed_mm_per_second=0.0):
    """한 갱신 동안 이동한 뒤 최대 감속해도 제약 지점 속도를 만족하는 가속도."""
    allowed_distance_mm = distance_mm + terminal_speed_mm_per_second ** 2 / (2 * braking_mm_per_second_squared)
    if allowed_distance_mm < speed_mm_per_second ** 2 / (2 * braking_mm_per_second_squared) - 1e-6:
        raise MotionSafetyError("최대 감속으로도 제약 지점에 도달하기 전에 감속 불가능")
    allowed_distance_mm = max(allowed_distance_mm, speed_mm_per_second ** 2 / (2 * braking_mm_per_second_squared))
    # 갱신 전에 정지하면 속도를 음수로 연장하지 않는다. 정지 후 역주행하는
    # 이차식 해를 쓰면 저속에서 제동거리를 과소평가하게 된다.
    if speed_mm_per_second > 0 and allowed_distance_mm <= speed_mm_per_second * duration_seconds / 2:
        return -speed_mm_per_second * speed_mm_per_second / (2 * max(allowed_distance_mm, 1e-12))
    quadratic = duration_seconds * duration_seconds / (2 * braking_mm_per_second_squared)
    linear = duration_seconds * duration_seconds / 2 + speed_mm_per_second * duration_seconds / braking_mm_per_second_squared
    constant = speed_mm_per_second * duration_seconds + speed_mm_per_second * speed_mm_per_second / (2 * braking_mm_per_second_squared) - allowed_distance_mm
    discriminant = max(0.0, linear * linear - 4 * quadratic * constant)
    calculated = -2 * constant / (linear + math.sqrt(discriminant))
    # 위에서 현재 정지거리의 물리적 가능성을 검사했다. 극히 짧은
    # 경계 통과 시간에서는 뺄셈 반올림 때문에 그 하한보다 작은 해가
    # 나올 수 있으므로, 실제로 가능한 최대 감속 하한으로 제한한다.
    return max(-braking_mm_per_second_squared, calculated)


def get_arrival_seconds(distance_mm, speed_mm_per_second, acceleration_mm_per_second_squared):
    if distance_mm <= 0.0:
        return 0.0
    if abs(acceleration_mm_per_second_squared) < 1e-10:
        return distance_mm / speed_mm_per_second if speed_mm_per_second > 1e-10 else math.inf
    discriminant = speed_mm_per_second ** 2 + 2 * acceleration_mm_per_second_squared * distance_mm
    if discriminant < -1e-6:
        return math.inf
    final_speed_mm_per_second = math.sqrt(max(0.0, discriminant))
    denominator = speed_mm_per_second + final_speed_mm_per_second
    return 2 * distance_mm / denominator if denominator > 1e-10 else math.inf


class SegmentMotion:
    def __init__(self, simulator):
        self.simulator = simulator
        self.link_vehicles = defaultdict(set)
        self.node_vehicles = defaultdict(set)
        self.followers = defaultdict(set)
        self.control_claims = defaultdict(dict)
        self.ring_reservations = defaultdict(set)
        self.port_claims = {}
        self.port_departures = defaultdict(dict)
        self.routes = {}
        self.target_ports = {}
        self.wait_records = {}
        self.finished_wait_count = 0
        self.finished_wait_seconds = 0.0
        self.maximum_finished_wait_seconds = 0.0
        self.active_port = {}
        self.safety_failures = []
        self.minimum_observed_gap_mm = math.inf
        self.maximum_update_seconds = 1.0
        self.follow_update_seconds = 0.1
        self.numerical_margin_mm = 1e-6
        # 지도 Rev7.1은 분기 LINK Param의 ReleaseDistance를 정의하지만
        # 제공 v1.9의 7,785개 LINK Param은 모두 빈 값이다("||||||").
        # 따라서 1,150mm는 회사 도면의 전환기 길이가 아니라 기존 차체
        # 850mm+정지 간격 300mm를 이용한 모델 보호 거리다. 짧은 LINK
        # 길이로 잘라 쓰지 않고 실제 누적 주행거리를 다음 LINK까지 잇는다.
        self.control_nodes = simulator.JUNCTION_NODES | simulator.MERGE_POINT_NODES
        self.port_link_indices = {link["id"]: simulator.EDGE_LINK_IDX[(link["a"], link["b"])]
                                  for link in simulator.graph_data["links"]}
        specification = importlib.util.spec_from_file_location("version_two_travel_time", Path(__file__).with_name("continuous_travel_time.py"))
        travel_module = importlib.util.module_from_spec(specification)
        specification.loader.exec_module(travel_module)
        self.travel_times = travel_module.SegmentTravelTimeCalculator(simulator)
        self.calculate_segment_profile = lambda start_speed, exit_speed, speed_limit, distance_mm: travel_module.get_segment_profile(
            simulator, start_speed, exit_speed, speed_limit, distance_mm)
        self.calculate_route_profile = lambda links, offset_mm=0.0, speed_mm_per_second=0.0, end_offset_mm=None: travel_module.get_travel_plan(
            simulator, links, offset_mm, speed_mm_per_second, end_offset_mm)
        self.route_exit_speeds = {}
        self.last_dispatch_locations = None
        self.merge_waiters = defaultdict(dict)
        self.merge_request_sequence = 0
        self.merge_wait_seconds = defaultdict(lambda: defaultdict(float))

    def fail(self, vehicle, message, **details):
        evidence = {"simulation_second": vehicle.env.now(), "oht_id": vehicle.oht_id,
                    "logical_node": vehicle.node, "link_index": vehicle._motion_link_index,
                    "offset_mm": vehicle._motion_offset_mm, "reason": message, **details}
        evidence["physical_position"] = self.get_position(vehicle)
        evidence["route"] = list(self.routes.get(vehicle, ()))
        evidence["control_claims"] = dict(self.control_claims.get(vehicle, {}))
        other_id = details.get("front_oht_id", details.get("other_oht_id"))
        if other_id is not None and other_id < len(self.simulator.OHT_LIST):
            other = self.simulator.OHT_LIST[other_id]
            evidence["other_position"] = self.get_position(other)
            evidence["other_logical_node"] = other.node
            evidence["other_control_claims"] = dict(self.control_claims.get(other, {}))
        self.safety_failures.append(evidence)
        raise MotionSafetyError(str(evidence))

    def register(self, vehicle):
        vehicle._motion_link_index = None
        vehicle._motion_offset_mm = 0.0
        vehicle._motion_plan = None
        vehicle._motion_transfer_node = None
        vehicle._motion_port_id = None
        vehicle._motion_waited_since_last_node = False
        vehicle._motion_last_stall_reroute_second = -math.inf
        self.node_vehicles[vehicle.node].add(vehicle)
        self.update_rings(vehicle)
        pending = [(0.0, vehicle.node)]
        distances = {vehicle.node: 0.0}
        import heapq
        while pending:
            distance_mm, node_id = heapq.heappop(pending)
            if distances[node_id] != distance_mm:
                continue
            for other in self.node_vehicles.get(node_id, ()):
                if other is not vehicle and distance_mm < self.simulator.FOLLOW_CENTER_GAP_MM:
                    self.fail(vehicle, "초기 안전거리 위반", other_oht_id=other.oht_id, distance_mm=distance_mm)
            if node_id in self.control_nodes and distance_mm < self.simulator.FOLLOW_CENTER_GAP_MM:
                holder = self.simulator.NODE_HOLDER.get(node_id)
                if holder not in (None, vehicle):
                    self.fail(vehicle, "초기 제어점 차체 점유 중복", other_oht_id=holder.oht_id)
                self.simulator.NODE_HOLDER[node_id] = vehicle
                self.control_claims[vehicle][node_id] = distance_mm
            for link_index in self.simulator.OUT_LINKS[node_id] + self.simulator.IN_LINKS[node_id]:
                other_node = (self.simulator.LINK_B[link_index] if self.simulator.LINK_A[link_index] == node_id
                              else self.simulator.LINK_A[link_index])
                candidate = distance_mm + self.simulator.LINK_LEN[link_index]
                if candidate < self.simulator.FOLLOW_CENTER_GAP_MM and candidate < distances.get(other_node, math.inf):
                    distances[other_node] = candidate
                    heapq.heappush(pending, (candidate, other_node))

    def get_position(self, vehicle, now=None):
        simulator = self.simulator
        now = vehicle.env.now() if now is None else now
        plan = vehicle._motion_plan
        elapsed_seconds = min(max(now - plan["start_second"], 0.0), plan["profile"]["dur"]) if plan else 0.0
        moved_mm = vehicle._profile_distance(plan["profile"], elapsed_seconds) if plan else 0.0
        speed_mm_per_second = max(0.0, plan["profile"]["v0"] + plan["profile"]["acceleration_mm_per_second_squared"] * elapsed_seconds) if plan else vehicle._cur_speed
        link_index = vehicle._motion_link_index
        offset_mm = vehicle._motion_offset_mm + moved_mm
        if link_index is None:
            x_mm, y_mm = simulator.NODE_XY[vehicle.node]
        else:
            start = simulator.LINK_A[link_index]
            end = simulator.LINK_B[link_index]
            fraction = offset_mm / simulator.LINK_LEN[link_index]
            first = simulator.NODE_XY[start]
            second = simulator.NODE_XY[end]
            x_mm = first[0] + (second[0] - first[0]) * fraction
            y_mm = first[1] + (second[1] - first[1]) * fraction
        return {"link_index": link_index, "offset_mm": offset_mm, "speed_mm_per_second": speed_mm_per_second,
                "x_mm": x_mm, "y_mm": y_mm, "total_distance_mm": vehicle.total_distance + moved_mm,
                "transfer_node": vehicle._motion_transfer_node if moved_mm <= 1e-7 else None,
                "port_id": vehicle._motion_port_id if moved_mm <= 1e-7 else None}

    def commit(self, vehicle):
        state = self.get_position(vehicle)
        moved_mm = state["offset_mm"] - vehicle._motion_offset_mm
        if vehicle._motion_plan is not None:
            elapsed_seconds = min(vehicle.env.now() - vehicle._motion_plan["start_second"],
                          vehicle._motion_plan["profile"]["dur"])
            if moved_mm > 1e-7:
                vehicle._job_travel += elapsed_seconds
        vehicle._motion_offset_mm = state["offset_mm"]
        vehicle.total_distance += moved_mm
        vehicle._cur_speed = state["speed_mm_per_second"]
        vehicle._motion_plan = None
        if moved_mm > 1e-7:
            vehicle._motion_transfer_node = None
            vehicle._motion_port_id = None
            self.clear_wait(vehicle)
        for node_id, odometer in list(self.control_claims[vehicle].items()):
            if vehicle.total_distance - odometer >= self.simulator.FOLLOW_CENTER_GAP_MM:
                if self.simulator.NODE_HOLDER.get(node_id) is vehicle:
                    self.simulator.NODE_HOLDER[node_id] = None
                del self.control_claims[vehicle][node_id]
        for key, odometer in list(self.port_departures[vehicle].items()):
            if vehicle.total_distance - odometer >= self.simulator.FOLLOW_CENTER_GAP_MM:
                if self.port_claims.get(key) is vehicle:
                    del self.port_claims[key]
                del self.port_departures[vehicle][key]
        self.update_rings(vehicle)
        self.check_vehicle_safety(vehicle)
        self.wake_followers(vehicle)

    def wake_followers(self, vehicle):
        for follower in self.followers.pop(vehicle, set()):
            if follower.ispassive():
                follower.activate()

    def update_rings(self, vehicle):
        simulator = self.simulator
        link_index = vehicle._motion_link_index
        if link_index is None:
            numbers = set(simulator.NODE_RINGS.get(vehicle.node, ()))
        else:
            numbers = (set(simulator.NODE_RINGS.get(simulator.LINK_A[link_index], ()))
                       & set(simulator.NODE_RINGS.get(simulator.LINK_B[link_index], ())))
        for number, members in enumerate(simulator.RING_MEMBERS):
            if number in numbers:
                members.add(vehicle)
                self.ring_reservations[number].discard(vehicle)
            else:
                members.discard(vehicle)
        simulator.HELD_NODES[vehicle] = {vehicle.node}

    def vehicles_at_node(self, node_id):
        return sorted(self.node_vehicles.get(node_id, ()), key=lambda member: member.oht_id)

    def get_origin_location(self, vehicle, for_assignment=False):
        simulator = self.simulator
        if for_assignment and vehicle.state == simulator.OHT_TO_DEST and vehicle.job is not None:
            port = simulator.PORT_BY_ID.get(vehicle.job.to_port)
            if port is not None:
                return {"node_id": simulator.LINK_A[self.port_link_indices[port["parent_link"]]],
                        "link_index": self.port_link_indices[port["parent_link"]],
                        "offset_mm": float(port["rel_dist"]), "speed_mm_per_second": 0.0, "fixed_links": ()}
            return {"node_id": vehicle.job.to_node, "link_index": None,
                    "offset_mm": 0.0, "speed_mm_per_second": 0.0, "fixed_links": ()}
        state = self.get_position(vehicle)
        fixed_links = []
        if state["speed_mm_per_second"] > 1e-5:
            distance_mm = -state["offset_mm"] if state["link_index"] is not None else 0.0
            for link_index in self.routes.get(vehicle, ()):
                fixed_links.append(link_index)
                distance_mm += simulator.LINK_LEN[link_index]
                if distance_mm >= state["speed_mm_per_second"] ** 2 / (2 * simulator.OHT_DECEL) + simulator.FOLLOW_CENTER_GAP_MM:
                    break
        return {"node_id": vehicle.node, "link_index": state["link_index"],
                "offset_mm": state["offset_mm"], "speed_mm_per_second": state["speed_mm_per_second"],
                "fixed_links": tuple(fixed_links)}

    def build_route(self, vehicle, target_node, port_id=None):
        simulator = self.simulator
        port = simulator.PORT_BY_ID.get(port_id) if port_id is not None else None
        target = ("port", port_id) if port is not None else ("node", target_node)
        origin = self.get_origin_location(vehicle)
        links = self.travel_times.find_route(origin, target, vehicle._avoid_links)
        if links is None and vehicle._avoid_links:
            links = self.travel_times.find_route(origin, target)
            vehicle._no_alt_node = vehicle.node
            simulator.REROUTE_COUNTS["no_alt"] += 1
        return links, float(port["rel_dist"]) if port is not None else None, port

    def get_dispatch_travel_times(self, vehicles, jobs):
        origins = [self.get_origin_location(vehicle, for_assignment=True) for vehicle in vehicles]
        targets = [("port", job.from_port) if job.from_port in self.simulator.PORT_BY_ID
                   else ("node", job.from_node) for job in jobs]
        self.last_dispatch_locations = {"simulation_second": vehicles[0].env.now(),
            "origins": [{"oht_id": vehicle.oht_id, **origin} for vehicle, origin in zip(vehicles, origins)],
            "targets": [{"job_id": job.job_id, "target_kind": target[0], "target_id": target[1]}
                        for job, target in zip(jobs, targets)],
            "definition": "빈 차량은 실제 위치·속도, 곧 하차하는 차량은 실제 ToPort T 정차점, 목적지는 FromPort T. 혼잡도 가중치는 기존 LINK_PENALTY"}
        return self.travel_times.get_travel_time_matrix(origins, targets)

    def get_merge_arrival_seconds(self, vehicle, links, merge_index, target_offset, port):
        simulator = self.simulator
        speed_mm_per_second = self.get_position(vehicle)["speed_mm_per_second"]
        elapsed_seconds = 0.0
        for number, link_index in enumerate(links[:merge_index + 1]):
            start_offset_mm = self.get_position(vehicle)["offset_mm"] if number == 0 and vehicle._motion_link_index is not None else 0.0
            distance_mm = simulator.LINK_LEN[link_index] - start_offset_mm
            if number == len(links) - 1 and port is not None and target_offset < simulator.LINK_LEN[link_index]:
                # 이번 경로는 합류 전 T에서 끝난다. 다음 작업의 출발을
                # 가정한 도착시각을 만들지 않고, 물리적 보호 점유만 유지한다.
                return math.inf
            exit_speed = self.route_exit_speeds[vehicle][number]
            profile = self.calculate_segment_profile(speed_mm_per_second, exit_speed, simulator.LINK_CAP[link_index], max(0.0, distance_mm))
            elapsed_seconds += profile["dur"]
            speed_mm_per_second = profile["v_end"]
        return elapsed_seconds

    def enter_link(self, vehicle, link_index):
        if vehicle._motion_link_index == link_index:
            return
        self.node_vehicles[vehicle.node].discard(vehicle)
        vehicle._motion_link_index = link_index
        vehicle._motion_offset_mm = 0.0
        self.link_vehicles[link_index].add(vehicle)
        simulator = self.simulator
        first, second = simulator.NODE_XY[simulator.LINK_A[link_index]], simulator.NODE_XY[simulator.LINK_B[link_index]]
        vehicle._update_heading(*first, *second)

    def set_route(self, vehicle, links, port=None):
        simulator = self.simulator
        self.routes[vehicle] = links
        self.target_ports[vehicle] = port
        if links:
            plan = self.calculate_route_profile(links, 0.0, 0.0, float(port["rel_dist"]) if port is not None else None)
            self.route_exit_speeds[vehicle] = list(plan["boundary_speed_limits"][1:])
        else:
            self.route_exit_speeds[vehicle] = []
        if links:
            vehicle._leg_path = [simulator.LINK_A[links[0]]] + [simulator.LINK_B[index] for index in links]
            _total, vehicle._leg_cum = simulator.path_travel_cumtime(vehicle._leg_path)
            vehicle._leg_idx = 0
        needed_nodes = {simulator.LINK_B[index] for index in links}
        for node_id, waiters in self.merge_waiters.items():
            if node_id not in needed_nodes:
                waiters.pop(vehicle, None)
                self.merge_wait_seconds[vehicle].pop(node_id, None)
        needed_rings = {number for node_id in needed_nodes for number in simulator.NODE_RINGS.get(node_id, ())}
        for node_id, odometer in list(self.control_claims[vehicle].items()):
            if node_id not in needed_nodes and odometer - vehicle.total_distance > simulator.FOLLOW_CENTER_GAP_MM + 1e-6:
                if simulator.NODE_HOLDER.get(node_id) is vehicle:
                    simulator.NODE_HOLDER[node_id] = None
                del self.control_claims[vehicle][node_id]
        for number, members in self.ring_reservations.items():
            if number not in needed_rings:
                members.discard(vehicle)
        for merge_node in list(vehicle._merge_admits):
            if merge_node not in needed_nodes:
                simulator.merge_on_enter(vehicle, merge_node)
        if vehicle._merge_wait is not None and vehicle._merge_wait[0] not in needed_nodes:
            simulator.merge_cancel_reservation(vehicle, *vehicle._merge_wait)
            vehicle._merge_wait = None

    def select_merge_waiter(self, node_id, now):
        """제동 여유 안에 접근한 각 진입로 선두의 요청 순서와 기존 우선권 적용."""
        waiters = self.merge_waiters[node_id]
        ordered = sorted(waiters, key=lambda member: waiters[member][1])
        if not ordered:
            return None
        # OCS owns admission. A separate FCFS choice must not hold an admitted
        # vehicle behind a requester whose OCS permission depends on that
        # vehicle clearing the merge. Physical gates are still checked below.
        if self.simulator.MERGE_CONTROL_MODE == "OCS":
            admitted = set().union(*self.simulator.MERGE_ADMITTED.get(node_id, {}).values())
            for member in ordered:
                if member in admitted:
                    return member
        held_link = (self.simulator.merge_priority_link(node_id, now)
                     if self.simulator.MERGE_CONTROL_MODE == "ADAPTIVE" else None)
        return next((member for member in ordered if waiters[member][2] == held_link), ordered[0])

    def request_merge_order(self, vehicle, node_id, incoming_link, holder):
        waiters = self.merge_waiters[node_id]
        if vehicle not in waiters:
            self.merge_request_sequence += 1
            waiters[vehicle] = [vehicle.env.now(), self.merge_request_sequence, incoming_link, holder is not None]
        else:
            waiters[vehicle][2] = incoming_link
        selected = self.select_merge_waiter(node_id, vehicle.env.now())
        if selected is not vehicle:
            waiters[vehicle][3] = True
        return selected

    def record_merge_grant(self, vehicle, node_id):
        waiters = self.merge_waiters[node_id]
        entry = waiters.get(vehicle)
        if entry is not None and entry[3]:
            ordered = sorted(waiters, key=lambda member: waiters[member][1])
            priority = (self.simulator.MERGE_CONTROL_MODE == "ADAPTIVE" and
                        self.simulator.merge_priority_link(node_id, vehicle.env.now()) == entry[2])
            self.simulator.MERGE_STATS["priority_grants" if priority else "fcfs_grants"] += 1
            if priority and ordered[0] is not vehicle:
                self.simulator.MERGE_STATS["overtakes"] += 1
        waiters.pop(vehicle, None)

    def find_front(self, vehicle, links, maximum_distance_mm=10000.0, extend_after_route=True):
        simulator = self.simulator
        # 관측 컴포넌트도 호출하므로 마지막 적분값이 아닌 현재 시각의
        # 실제 위치를 기준으로 앞뒤 순서를 정한다.
        cumulative_mm = -self.get_position(vehicle)["offset_mm"] if vehicle._motion_link_index is not None else 0.0
        closest = (math.inf, None)
        for route_index, link_index in enumerate(links):
            for other in self.link_vehicles.get(link_index, ()):
                if other is vehicle:
                    continue
                distance_mm = cumulative_mm + self.get_position(other)["offset_mm"]
                if 0 <= distance_mm < closest[0]:
                    closest = (distance_mm, other)
            cumulative_mm += simulator.LINK_LEN[link_index]
            for other in self.node_vehicles.get(simulator.LINK_B[link_index], ()):
                if other is not vehicle and cumulative_mm < closest[0]:
                    closest = (cumulative_mm, other)
            if closest[1] is not None or cumulative_mm > maximum_distance_mm:
                break
        else:
            # G에서 주차/비켜주기를 끝내더라도 바로 다음 링크의 차와
            # 1,150mm를 유지해야 한다. 목적 노드가 탐색의 물리적 끝은 아니다.
            if links and extend_after_route:
                pending = [(simulator.LINK_B[links[-1]], cumulative_mm)]
                checked = {}
                endpoint_distance_mm = cumulative_mm
                while pending:
                    node_id, distance_mm = pending.pop()
                    if distance_mm >= checked.get(node_id, math.inf):
                        continue
                    checked[node_id] = distance_mm
                    for link_index in simulator.OUT_LINKS[node_id]:
                        for other in self.link_vehicles.get(link_index, ()):
                            if other is vehicle:
                                continue
                            front_distance_mm = distance_mm + self.get_position(other)["offset_mm"]
                            if 0 <= front_distance_mm < closest[0]:
                                closest = (front_distance_mm, other)
                        next_distance_mm = distance_mm + simulator.LINK_LEN[link_index]
                        next_node = simulator.LINK_B[link_index]
                        for other in self.node_vehicles.get(next_node, ()):
                            if other is not vehicle and next_distance_mm < closest[0]:
                                closest = (next_distance_mm, other)
                        if next_distance_mm - endpoint_distance_mm < simulator.FOLLOW_CENTER_GAP_MM:
                            pending.append((next_node, next_distance_mm))
        return closest

    def clear_wait(self, vehicle):
        previous = self.wait_records.pop(vehicle, None)
        if previous is not None:
            elapsed_seconds = vehicle.env.now() - previous["since_second"]
            self.finished_wait_count += 1
            self.finished_wait_seconds += elapsed_seconds
            self.maximum_finished_wait_seconds = max(self.maximum_finished_wait_seconds, elapsed_seconds)
            simulator = self.simulator
            if elapsed_seconds >= simulator.STALL_THRESHOLD_SECONDS:
                state = self.get_position(vehicle)
                simulator.STALL_EVENTS.append({"oht": vehicle.oht_id, "node": vehicle.node,
                    "t_start": previous["since_second"], "duration": elapsed_seconds,
                    "lock_wait": previous["lock_seconds"], "follow_wait": previous["follow_seconds"],
                    "link_index": state["link_index"], "link_offset_mm": state["offset_mm"],
                    "position_x_millimeters": state["x_mm"], "position_y_millimeters": state["y_mm"],
                    "blocking_details": previous["detail"], "occupancy_mode": "segment"})
        vehicle._stall_next = None
        vehicle._stall_start = None
        vehicle._stuck_since = None
        vehicle._ring_waiting = False

    def set_wait(self, vehicle, cause, blockers, detail, next_node):
        now = vehicle.env.now()
        previous = self.wait_records.get(vehicle)
        self.wait_records[vehicle] = {"cause": cause, "blocking_vehicles": list(blockers),
                                     "detail": detail, "since_second": previous["since_second"] if previous else now,
                                     "accounted_until_second": previous["accounted_until_second"] if previous else now,
                                     "lock_seconds": previous["lock_seconds"] if previous else 0.0,
                                     "follow_seconds": previous["follow_seconds"] if previous else 0.0}
        if previous is None and cause != "follow_distance":
            self.simulator.WAIT_COUNTS[next_node] += 1
        vehicle._stall_next = next_node
        vehicle._stall_start = self.wait_records[vehicle]["since_second"]
        vehicle._stuck_since = vehicle._stall_start
        vehicle._ring_waiting = cause == "ring_capacity"
        vehicle._motion_waited_since_last_node = True

    def request_idle_yield(self, other):
        simulator = self.simulator
        if other.job is None and other.state == simulator.OHT_IDLE and not other._yield_request:
            other._yield_request = True
            simulator.IDLE_STATS["yield_requests"] += 1
            if other.ispassive():
                other.activate()

    def update_constraints(self, vehicle, links, target_offset, port):
        simulator = self.simulator
        constraints = []
        distance_mm = -vehicle._motion_offset_mm if vehicle._motion_link_index is not None else 0.0
        maximum_lookahead_mm = (simulator.OHT_MAX_SPEED ** 2 / (2 * simulator.OHT_DECEL)
                                + simulator.OHT_MAX_SPEED + simulator.FOLLOW_CENTER_GAP_MM)
        predicted_speed_mm_per_second = min(simulator.OHT_MAX_SPEED,
            vehicle._cur_speed + simulator.OHT_ACCEL * self.maximum_update_seconds)
        reservation_distance_mm = (predicted_speed_mm_per_second ** 2 / (2 * simulator.OHT_DECEL)
                                   + predicted_speed_mm_per_second * self.maximum_update_seconds
                                   + simulator.FOLLOW_CENTER_GAP_MM)
        reason = None
        has_blocked_gate = False
        for number, link_index in enumerate(links):
            remaining_start_mm = max(distance_mm, 0.0)
            constraints.append((remaining_start_mm, simulator.LINK_CAP[link_index]))
            if link_index in simulator.BLOCKED_LINKS and link_index != vehicle._motion_link_index:
                constraints.append((max(0.0, distance_mm), 0.0))
                if reason is None:
                    reason = ("blocked_link", [], {"link_index": link_index})
            end_node = simulator.LINK_B[link_index]
            distance_mm += simulator.LINK_LEN[link_index]
            node_distance_mm = distance_mm
            front_distance_mm, front = self.find_front(vehicle, links[:number + 1], extend_after_route=False)
            is_front_of_gate = front is None or front_distance_mm > node_distance_mm + 1e-6
            if number + 1 < len(links):
                constraints.append((node_distance_mm, simulator.JUNCTION_SPEED.get(
                    (link_index, links[number + 1]), simulator.LINK_CAP[links[number + 1]])))
            is_goal = number == len(links) - 1
            if is_goal and port is not None:
                distance_mm += target_offset - simulator.LINK_LEN[link_index]
            if is_goal:
                constraints.append((distance_mm, 0.0))
            if simulator.ENABLE_RING_CONTROL and end_node not in self.control_nodes:
                if self.is_ring_blocked(vehicle, end_node):
                    constraints.append((max(0.0, node_distance_mm - simulator.FOLLOW_CENTER_GAP_MM), 0.0))
                    members = set().union(*(simulator.RING_MEMBERS[ring_number] | self.ring_reservations[ring_number]
                                           for ring_number in simulator.NODE_RINGS.get(end_node, ())))
                    if reason is None:
                        reason = ("ring_capacity", list(members - {vehicle}), {"node": end_node})
                    has_blocked_gate = True
                elif node_distance_mm <= reservation_distance_mm and is_front_of_gate and not has_blocked_gate:
                    for ring_number in simulator.NODE_RINGS.get(end_node, ()):
                        if vehicle not in simulator.RING_MEMBERS[ring_number]:
                            self.ring_reservations[ring_number].add(vehicle)
            if end_node in self.control_nodes and not (is_goal and port is not None
                    and target_offset + simulator.FOLLOW_CENTER_GAP_MM < simulator.LINK_LEN[link_index]):
                holder = simulator.NODE_HOLDER.get(end_node)
                gate_reason = None
                selected_waiter = vehicle
                if end_node in simulator.MERGE_POINT_NODES and holder is not vehicle:
                    if (node_distance_mm <= reservation_distance_mm and is_front_of_gate
                            and not has_blocked_gate and not self.is_ring_blocked(vehicle, end_node)):
                        selected_waiter = self.request_merge_order(vehicle, end_node, link_index, holder)
                    else:
                        self.merge_waiters[end_node].pop(vehicle, None)
                if holder is vehicle and not is_front_of_gate and self.control_claims[vehicle].get(end_node, 0) > vehicle.total_distance:
                    simulator.NODE_HOLDER[end_node] = None
                    del self.control_claims[vehicle][end_node]
                    holder = None
                if holder is not None and holder is not vehicle:
                    gate_reason = ("node_lock", [holder], {"node": end_node})
                elif not is_front_of_gate:
                    gate_reason = ("follow_distance", [front], {"node": end_node, "distance_mm": front_distance_mm})
                elif has_blocked_gate:
                    gate_reason = ("upstream_gate", [], {"node": end_node})
                elif simulator.ENABLE_RING_CONTROL and self.is_ring_blocked(vehicle, end_node):
                    members = set().union(*(simulator.RING_MEMBERS[number] for number in simulator.NODE_RINGS.get(end_node, ())))
                    gate_reason = ("ring_capacity", list(members - {vehicle}), {"node": end_node})
                elif selected_waiter is not vehicle:
                    gate_reason = ("merge_order", [selected_waiter], {"merge_node": end_node,
                                   "mode": simulator.MERGE_CONTROL_MODE})
                elif (end_node in simulator.MERGE_POINT_NODES and simulator.MERGE_CONTROL_MODE == "OCS"
                        and end_node not in vehicle._merge_admits and not
                        (is_goal and port is not None and target_offset < simulator.LINK_LEN[link_index])):
                    chain = simulator.MERGE_CHAIN_OF_LINK.get(link_index)
                    side = chain[1] if chain is not None and chain[0] == end_node else None
                    if side is not None:
                        arrival_seconds = self.get_merge_arrival_seconds(vehicle, links, number, target_offset, port)
                        if simulator.merge_request(vehicle, end_node, side, arrival_seconds, vehicle.env.now()):
                            vehicle._merge_wait = None
                        else:
                            vehicle._merge_wait = (end_node, side)
                            admitted = simulator.MERGE_ADMITTED.get(end_node, {})
                            blockers = [other for members in admitted.values() for other in members if other is not vehicle]
                            if self.is_outgoing_full(end_node):
                                blockers.extend(other for other in self.get_outgoing_vehicles(end_node) if other is not vehicle)
                            if vehicle._merge_deny_ring:
                                blockers.extend(other for ring_number in simulator.NODE_RINGS.get(end_node, ())
                                                for other in simulator.RING_MEMBERS[ring_number] if other is not vehicle)
                            gate_reason = ("merge_permission", blockers, {"merge_node": end_node, "side": side,
                                "outgoing_segment_full": self.is_outgoing_full(end_node)})
                if gate_reason is not None:
                    constraints.append((max(0.0, node_distance_mm - simulator.FOLLOW_CENTER_GAP_MM), 0.0))
                    # Keep the first blocking gate on the forward route.
                    # A downstream 'upstream_gate' is only a consequence and
                    # must not erase its blocker IDs or ring-wait semantics.
                    if reason is None:
                        reason = gate_reason
                    has_blocked_gate = True
                elif distance_mm <= reservation_distance_mm:
                    if end_node in simulator.MERGE_POINT_NODES and holder is not vehicle:
                        self.record_merge_grant(vehicle, end_node)
                    simulator.NODE_HOLDER[end_node] = vehicle
                    self.control_claims[vehicle].setdefault(end_node, vehicle.total_distance + node_distance_mm)
                    for ring_number in simulator.NODE_RINGS.get(end_node, ()):
                        if vehicle not in simulator.RING_MEMBERS[ring_number]:
                            self.ring_reservations[ring_number].add(vehicle)
            if distance_mm > maximum_lookahead_mm:
                break
        if port is not None and distance_mm <= maximum_lookahead_mm:
            key = (port["parent_link"], float(port["rel_dist"]))
            holder = self.port_claims.get(key)
            if holder not in (None, vehicle):
                # 작업 뒤 출발 중인 앞차는 실제 위치로 따라간다. 예전 T
                # 좌표 앞 1,150mm에 고정 장벽을 남기면 안전하게 접근 중인
                # 뒤차 뒤에 정지선이 갑자기 생길 수 있다.
                constraints.append((max(0.0, distance_mm), 0.0))
                if reason is None:
                    reason = ("port_service", [holder], {"port_id": port["port_id"], "transfer_node": port["transfer_node"]})
            # 접근 예약은 실제 정차 자리 점유가 아니다. 선두가 T에서 정지한
            # 순간에만 점유한다. 뒤차 접근은 실제 앞차 위치로 제어한다.
        # 짧은 링크에서는 목적 G/T가 다음 합류점의 1,150mm 보호 구역
        # 안에 있을 수 있다. 목적지 뒤의 그 제어점도 함께 보호해야 다른
        # 갈래 차량이 합류하며 정차 차량 앞으로 갑자기 끼어들지 않는다.
        endpoint_distance_mm = sum(simulator.LINK_LEN[index] for index in links) - vehicle._motion_offset_mm
        if port is not None:
            endpoint_distance_mm += target_offset - simulator.LINK_LEN[links[-1]]
        pending = [(simulator.LINK_B[links[-1]],
                    0.0 if port is None else simulator.LINK_LEN[links[-1]] - target_offset,
                    list(links))]
        checked = {}
        while pending:
            node_id, after_endpoint_mm, extended_links = pending.pop()
            if (after_endpoint_mm >= simulator.FOLLOW_CENTER_GAP_MM
                    or after_endpoint_mm >= checked.get(node_id, math.inf)):
                continue
            checked[node_id] = after_endpoint_mm
            if node_id in self.control_nodes and node_id not in {simulator.LINK_B[index] for index in links}:
                node_distance_mm = endpoint_distance_mm + after_endpoint_mm
                holder = simulator.NODE_HOLDER.get(node_id)
                front_distance_mm, front = self.find_front(vehicle, extended_links, extend_after_route=False)
                blockers = ([holder] if holder not in (None, vehicle) else
                            [front] if front is not None and front_distance_mm <= node_distance_mm else [])
                if blockers:
                    constraints.append((max(0.0, node_distance_mm - simulator.FOLLOW_CENTER_GAP_MM), 0.0))
                    if reason is None:
                        reason = ("node_lock", blockers, {"node": node_id, "endpoint_protection": True})
                elif node_distance_mm <= reservation_distance_mm and not has_blocked_gate:
                    simulator.NODE_HOLDER[node_id] = vehicle
                    self.control_claims[vehicle].setdefault(node_id, vehicle.total_distance + node_distance_mm)
            for link_index in simulator.OUT_LINKS[node_id]:
                pending.append((simulator.LINK_B[link_index], after_endpoint_mm + simulator.LINK_LEN[link_index],
                                extended_links + [link_index]))
        return constraints, reason

    def goto(self, vehicle, target_node, port_id=None):
        simulator = self.simulator
        self.commit(vehicle)
        links, target_offset, port = self.build_route(vehicle, target_node, port_id)
        while links is None:
            simulator.REROUTE_COUNTS["fail"] += 1
            yield vehicle.hold(simulator.PATH_RETRY_COOLDOWN)
            links, target_offset, port = self.build_route(vehicle, target_node, port_id)
        vehicle._replan = False
        vehicle._avoid_links = None
        self.set_route(vehicle, links, port)
        is_aborting = False
        if links:
            vehicle._leg_path = [simulator.LINK_A[links[0]]] + [simulator.LINK_B[index] for index in links]
            _total, vehicle._leg_cum = simulator.path_travel_cumtime(vehicle._leg_path)
            vehicle._leg_idx = 0
        while links:
            self.commit(vehicle)
            if vehicle._abort_leg and not is_aborting:
                if vehicle._cur_speed <= 1e-5:
                    self.clear_wait(vehicle)
                    self.routes[vehicle] = []
                    return False
                stopping_links = []
                available_distance_mm = -vehicle._motion_offset_mm if vehicle._motion_link_index is not None else 0.0
                for link_index in links:
                    stopping_links.append(link_index)
                    available_distance_mm += simulator.LINK_LEN[link_index]
                    if available_distance_mm >= vehicle._cur_speed ** 2 / (2 * simulator.OHT_DECEL) + 1e-6:
                        break
                links, target_offset, port = stopping_links, None, None
                is_aborting = True
                vehicle._replan = False
                self.set_route(vehicle, links)
            approach_node = (simulator.LINK_B[vehicle._motion_link_index]
                             if vehicle._motion_link_index is not None else None)
            if (not is_aborting and simulator.ENABLE_JUNCTION_REROUTE and approach_node in simulator.JUNCTION_NODES and len(links) > 1
                    and vehicle._junction_reroute_node != approach_node
                    and vehicle.env.now() - vehicle._last_reroute_t >= simulator.JUNCTION_REROUTE_MIN_GAP):
                vehicle._junction_reroute_node = approach_node
                vehicle._last_reroute_t = vehicle.env.now()
                vehicle._replan = True
                simulator.REROUTE_COUNTS["junction"] += 1
            if not is_aborting and vehicle._replan and vehicle._motion_link_index is not None:
                replacement, replacement_offset, replacement_port = self.build_route(vehicle, target_node, port_id)
                if replacement is not None:
                    links, target_offset, port = replacement, replacement_offset, replacement_port
                    vehicle._replan = False
                    vehicle._avoid_links = None
                    self.set_route(vehicle, links, port)
            current = links[0]
            if vehicle._motion_link_index is None:
                if vehicle._abort_leg and not is_aborting:
                    self.clear_wait(vehicle)
                    return False
                if (not is_aborting and (vehicle._replan or (simulator.ENABLE_JUNCTION_REROUTE and vehicle.node in simulator.JUNCTION_NODES
                        and vehicle._junction_reroute_node != vehicle.node
                        and vehicle.env.now() - vehicle._last_reroute_t >= simulator.JUNCTION_REROUTE_MIN_GAP))):
                    if not vehicle._replan:
                        simulator.REROUTE_COUNTS["junction"] += 1
                    vehicle._junction_reroute_node = vehicle.node
                    vehicle._last_reroute_t = vehicle.env.now()
                    vehicle._replan = False
                    links, target_offset, port = self.build_route(vehicle, target_node, port_id)
                    while links is None:
                        yield vehicle.hold(simulator.PATH_RETRY_COOLDOWN)
                        simulator.REROUTE_COUNTS["fail"] += 1
                        links, target_offset, port = self.build_route(vehicle, target_node, port_id)
                    vehicle._avoid_links = None
                    self.set_route(vehicle, links, port)
                    if not links:
                        break
                    current = links[0]
                self.enter_link(vehicle, current)
            final_offset = target_offset if len(links) == 1 and port is not None else simulator.LINK_LEN[current]
            remaining_mm = final_offset - vehicle._motion_offset_mm
            boundary_tolerance_mm = max(1e-9, 2*math.ulp(vehicle.env.now())*max(vehicle._cur_speed,1.0))
            if ((remaining_mm <= boundary_tolerance_mm and (len(links) > 1 or vehicle._cur_speed <= 1e-4))
                    or (len(links) == 1 and remaining_mm <= 1e-6 and vehicle._cur_speed <= 1e-4)):
                vehicle.total_distance += remaining_mm
                vehicle._motion_offset_mm = final_offset
                if len(links) == 1 and port is not None:
                    if vehicle._cur_speed > 1e-4:
                        self.fail(vehicle, "이적재 위치에서 정지하지 못함", speed_mm_per_second=vehicle._cur_speed)
                    vehicle._motion_transfer_node = port["transfer_node"]
                    vehicle._motion_port_id = port["port_id"]
                    key = (port["parent_link"], float(port["rel_dist"]))
                    self.port_claims[key] = vehicle
                    self.port_departures[vehicle][key] = vehicle.total_distance
                    self.active_port[vehicle] = port
                    self.clear_wait(vehicle)
                    self.routes[vehicle] = []
                    return True
                self.link_vehicles[current].discard(vehicle)
                vehicle._motion_link_index = None
                vehicle._motion_offset_mm = 0.0
                vehicle.node = simulator.LINK_B[current]
                vehicle._arrival_link = current
                self.node_vehicles[vehicle.node].add(vehicle)
                simulator.LINK_FLOW[current] += 1
                if vehicle.node in vehicle._merge_admits:
                    simulator.merge_on_enter(vehicle, vehicle.node)
                if vehicle.node in simulator.MERGE_POINT_NODES:
                    simulator.grant_merge_priority(vehicle.node, current, vehicle.env.now())
                    waiting_seconds = self.merge_wait_seconds[vehicle].pop(vehicle.node, 0.0)
                    statistics = simulator.merge_node_stat(vehicle.node)
                    statistics["passes"] += 1
                    statistics["node_wait"] += waiting_seconds
                    if simulator.RECORD_MERGE_PASS_LOG:
                        simulator.MERGE_PASS_LOG.append((vehicle.env.now(), vehicle.node, vehicle.oht_id,
                                                        simulator.LINK_A[current], waiting_seconds, vehicle.job is not None))
                vehicle._leg_idx += 1
                if not vehicle._motion_waited_since_last_node:
                    vehicle._first_stall_t = None
                    vehicle._stall_avoid_links.clear()
                vehicle._motion_waited_since_last_node = False
                links.pop(0)
                self.route_exit_speeds[vehicle].pop(0)
                self.update_rings(vehicle)
                continue
            constraints, gate_reason = self.update_constraints(vehicle, links, target_offset, port)
            front_distance_mm, front = self.find_front(vehicle, links)
            duration_seconds = self.follow_update_seconds if front is not None else self.maximum_update_seconds
            speed_mm_per_second = vehicle._cur_speed
            free_profile = self.calculate_segment_profile(speed_mm_per_second,
                self.route_exit_speeds[vehicle][0], simulator.LINK_CAP[current], max(0.0, remaining_mm))
            phase_time_resolution = 2*math.ulp(vehicle.env.now())
            if free_profile["t_acc"] > phase_time_resolution:
                acceleration_mm_per_second_squared = simulator.OHT_ACCEL
                duration_seconds = min(duration_seconds, free_profile["t_acc"])
            elif free_profile["t_cruise"] > phase_time_resolution:
                acceleration_mm_per_second_squared = 0.0
                duration_seconds = min(duration_seconds, free_profile["t_cruise"])
            else:
                acceleration_mm_per_second_squared = -simulator.OHT_DECEL
                duration_seconds = min(duration_seconds, max(free_profile["t_dec"], 1e-9))
            reason = gate_reason
            if front is not None:
                self.minimum_observed_gap_mm = min(self.minimum_observed_gap_mm, front_distance_mm)
                if front_distance_mm < simulator.FOLLOW_CENTER_GAP_MM - 1e-6:
                    self.fail(vehicle, "차간거리 위반", front_oht_id=front.oht_id, distance_mm=front_distance_mm)
                front_speed_mm_per_second = self.get_position(front)["speed_mm_per_second"]
                available_mm = front_distance_mm - simulator.FOLLOW_CENTER_GAP_MM
                try:
                    acceleration_mm_per_second_squared = min(acceleration_mm_per_second_squared, get_acceleration_limit(speed_mm_per_second, max(0.0, available_mm), duration_seconds,
                                        simulator.OHT_DECEL, front_speed_mm_per_second))
                except MotionSafetyError:
                    self.fail(vehicle, "앞차에 대한 제동 여유 부족", front_oht_id=front.oht_id, distance_mm=front_distance_mm)
                if available_mm < simulator.OHT_MAX_SPEED ** 2 / (2 * simulator.OHT_DECEL):
                    gate_distance_mm = min((distance for distance, terminal_speed_mm_per_second in constraints if terminal_speed_mm_per_second == 0), default=math.inf)
                    if gate_reason is None or available_mm < gate_distance_mm - 1e-6:
                        reason = ("follow_distance", [front], {"distance_mm": front_distance_mm,
                                  "transfer_node": self.get_position(front)["transfer_node"], "port_id": self.get_position(front)["port_id"]})
            for constraint_distance_mm, terminal_speed_mm_per_second in constraints:
                if constraint_distance_mm <= 1e-6 and terminal_speed_mm_per_second > 0:
                    acceleration_mm_per_second_squared = min(acceleration_mm_per_second_squared, (terminal_speed_mm_per_second - speed_mm_per_second) / duration_seconds)
                    continue
                try:
                    acceleration_mm_per_second_squared = min(acceleration_mm_per_second_squared, get_acceleration_limit(speed_mm_per_second, max(0.0, constraint_distance_mm),
                                        duration_seconds, simulator.OHT_DECEL, terminal_speed_mm_per_second))
                except MotionSafetyError:
                    self.fail(vehicle, "고정 제약에 대한 제동 여유 부족", constraint_distance_mm=constraint_distance_mm,
                              speed_mm_per_second=speed_mm_per_second, terminal_speed_mm_per_second=terminal_speed_mm_per_second, gate_reason=str(gate_reason))
            if acceleration_mm_per_second_squared < -simulator.OHT_DECEL - 1e-4:
                self.fail(vehicle, "감속 한계 초과", acceleration_mm_per_second_squared=acceleration_mm_per_second_squared, speed_mm_per_second=speed_mm_per_second,
                          reason_detail=str(reason), constraints=constraints)
            acceleration_mm_per_second_squared = max(-simulator.OHT_DECEL, acceleration_mm_per_second_squared)
            if speed_mm_per_second <= 1e-6 and acceleration_mm_per_second_squared <= 1e-5:
                cause, blockers, detail = reason or ("route_clearance", [], {})
                self.set_wait(vehicle, cause, blockers, detail, simulator.LINK_B[current])
                for other in blockers:
                    self.request_idle_yield(other)
                started = vehicle.env.now()
                if cause == "follow_distance" and blockers and gate_reason is None:
                    self.followers[blockers[0]].add(vehicle)
                    yield vehicle.passivate()
                else:
                    yield vehicle.hold(min(simulator.MERGE_REQUEST_POLL, simulator.RING_WAIT_POLL))
                elapsed_seconds = vehicle.env.now() - started
                # 합류 접근 구간에서 실제 정지한 시간만 적산한다. 요청을
                # 미리 등록한 뒤 계속 주행한 시간은 합류 대기로 세지 않는다.
                distance_to_merge_mm = -vehicle._motion_offset_mm
                for approach_index, approach_link in enumerate(links):
                    distance_to_merge_mm += simulator.LINK_LEN[approach_link]
                    if approach_index == len(links) - 1 and port is not None and target_offset < simulator.LINK_LEN[approach_link]:
                        break
                    merge_node = simulator.LINK_B[approach_link]
                    if distance_to_merge_mm > simulator.OHT_MAX_SPEED ** 2 / (2 * simulator.OHT_DECEL) + simulator.OHT_MAX_SPEED + simulator.FOLLOW_CENTER_GAP_MM:
                        break
                    if merge_node in simulator.MERGE_POINT_NODES:
                        self.merge_wait_seconds[vehicle][merge_node] += elapsed_seconds
                        break
                    if merge_node in simulator.JUNCTION_NODES:
                        break
                self.wait_records[vehicle]["accounted_until_second"] = vehicle.env.now()
                vehicle._job_wait += elapsed_seconds
                vehicle.total_stall_time += elapsed_seconds
                if cause == "follow_distance":
                    vehicle.total_follow_stall_time += elapsed_seconds
                    self.wait_records[vehicle]["follow_seconds"] += elapsed_seconds
                else:
                    vehicle.total_lock_stall_time += elapsed_seconds
                    self.wait_records[vehicle]["lock_seconds"] += elapsed_seconds
                simulator.LINK_STALL[current] += elapsed_seconds
                region_node = simulator.LINK_A[current] if vehicle._motion_offset_mm < simulator.LINK_LEN[current] / 2 else simulator.LINK_B[current]
                simulator._accumulate_region_stall(region_node, elapsed_seconds,
                    0.0 if cause == "follow_distance" else elapsed_seconds,
                    elapsed_seconds if cause == "follow_distance" else 0.0)
                if cause == "merge_permission":
                    simulator.MERGE_OCS_STATS["wait_time"] += elapsed_seconds
                    simulator.merge_node_stat(detail["merge_node"])["reserve_wait"] += elapsed_seconds
                continue
            self.clear_wait(vehicle)
            arrival_seconds = get_arrival_seconds(remaining_mm, speed_mm_per_second, acceleration_mm_per_second_squared)
            duration_seconds = min(duration_seconds, arrival_seconds)
            # 위치 오차 1e-6mm 이내에서 잔여 속도가 남으면 속도를 덮어쓰지
            # 않고 실제 마지막 감속 시간을 적분한다(최대 위치 오차도 1e-6mm).
            if remaining_mm <= 1e-6 and speed_mm_per_second <= 0.1:
                acceleration_mm_per_second_squared = -simulator.OHT_DECEL
                duration_seconds = speed_mm_per_second / simulator.OHT_DECEL
            if acceleration_mm_per_second_squared < 0 and speed_mm_per_second > 0:
                duration_seconds = min(duration_seconds, speed_mm_per_second / -acceleration_mm_per_second_squared)
            if duration_seconds <= 0 or vehicle.env.now() + duration_seconds == vehicle.env.now():
                self.fail(vehicle, "시간 진행 없는 주행 계획",duration_seconds=duration_seconds,
                          remaining_mm=remaining_mm,free_profile=free_profile,
                          acceleration_mm_per_second_squared=acceleration_mm_per_second_squared)
            final_speed_mm_per_second = max(0.0, speed_mm_per_second + acceleration_mm_per_second_squared * duration_seconds)
            profile = {"v0": speed_mm_per_second, "v_end": final_speed_mm_per_second, "v_peak": max(speed_mm_per_second, final_speed_mm_per_second),
                       "t_acc": duration_seconds if acceleration_mm_per_second_squared > 0 else 0.0,
                       "t_cruise": duration_seconds if acceleration_mm_per_second_squared == 0 else 0.0,
                       "t_dec": duration_seconds if acceleration_mm_per_second_squared < 0 else 0.0,
                       "dur": duration_seconds, "acceleration_mm_per_second_squared": acceleration_mm_per_second_squared}
            vehicle._motion_plan = {"start_second": vehicle.env.now(), "profile": profile}
            vehicle._seg_profile = profile
            vehicle._seg_start_t = vehicle.env.now()
            self.wake_followers(vehicle)
            yield vehicle.hold(duration_seconds)
        self.clear_wait(vehicle)
        self.routes[vehicle] = []
        return not is_aborting

    def yield_step(self, vehicle):
        simulator = self.simulator
        current = vehicle._motion_link_index
        origin = simulator.LINK_B[current] if current is not None else vehicle.node
        choices = []
        for link_index in simulator.OUT_LINKS.get(origin, ()):
            if link_index in simulator.BLOCKED_LINKS:
                continue
            links = ([current] if current is not None else []) + [link_index]
            front_distance_mm, front = self.find_front(vehicle, links)
            goal_distance_mm = sum(simulator.LINK_LEN[index] for index in links) - (
                vehicle._motion_offset_mm if current is not None else 0.0)
            if front is not None and front_distance_mm < goal_distance_mm + simulator.FOLLOW_CENTER_GAP_MM:
                self.request_idle_yield(front)
                continue
            end_node = simulator.LINK_B[link_index]
            if any(simulator.RINGS[number].get("hard") and self.is_ring_blocked(vehicle, end_node)
                   for number in simulator.NODE_RINGS.get(end_node, ())):
                continue
            target = vehicle._idle_target
            preference_mm = (math.dist(simulator.NODE_XY[end_node], simulator.NODE_XY[target])
                             if target in simulator.NODE_XY else 0.0)
            choices.append((self.is_ring_blocked(vehicle, end_node), preference_mm, end_node))
        if not choices:
            yield vehicle.hold(simulator.YIELD_RETRY_INTERVAL)
            return
        vehicle._yield_request = False
        vehicle._abort_leg = False
        vehicle._replan = False
        simulator.IDLE_STATS["yield_moves"] += 1
        if vehicle.node == vehicle._idle_target:
            simulator.release_standby_point(vehicle)
        vehicle._ring_exempt = True
        try:
            yield from self.goto(vehicle, min(choices)[2])
        finally:
            vehicle._ring_exempt = False

    def is_outgoing_full(self, merge_node):
        simulator = self.simulator
        exits = simulator.OUT_LINKS.get(merge_node, ())
        for first_link in exits:
            distance_mm = 0.0
            link_index = first_link
            closest = math.inf
            seen = set()
            while link_index not in seen and distance_mm < 2 * simulator.FOLLOW_CENTER_GAP_MM:
                seen.add(link_index)
                for member in self.link_vehicles.get(link_index, ()):
                    state = self.get_position(member)
                    if not simulator.MERGE_OUT_FULL_STOPPED_ONLY or state["speed_mm_per_second"] <= 1e-6:
                        closest = min(closest, distance_mm + state["offset_mm"])
                distance_mm += simulator.LINK_LEN[link_index]
                end_node = simulator.LINK_B[link_index]
                for member in self.node_vehicles.get(end_node, ()):
                    if not simulator.MERGE_OUT_FULL_STOPPED_ONLY or self.get_position(member)["speed_mm_per_second"] <= 1e-6:
                        closest = min(closest, distance_mm)
                following = simulator.OUT_LINKS[end_node]
                if len(following) != 1:
                    break
                link_index = following[0]
            # 합류점을 벗어날 1,150mm와 그 앞차까지의 1,150mm가 필요하다.
            if closest >= 2 * simulator.FOLLOW_CENTER_GAP_MM:
                return False
        return bool(exits)

    def get_outgoing_vehicles(self, merge_node):
        """합류 출구의 실제 링크 차량. 논리적 G 노드 소속으로 찾지 않는다."""
        simulator = self.simulator
        members = set()
        pending = [(index, 0.0) for index in simulator.OUT_LINKS.get(merge_node, ())]
        seen = set()
        while pending:
            link_index, distance_mm = pending.pop()
            if link_index in seen or distance_mm >= 2 * simulator.FOLLOW_CENTER_GAP_MM:
                continue
            seen.add(link_index)
            members.update(member for member in self.link_vehicles.get(link_index, ())
                           if distance_mm + self.get_position(member)["offset_mm"] < 2 * simulator.FOLLOW_CENTER_GAP_MM)
            distance_mm += simulator.LINK_LEN[link_index]
            end_node = simulator.LINK_B[link_index]
            if distance_mm < 2 * simulator.FOLLOW_CENTER_GAP_MM:
                members.update(self.node_vehicles.get(end_node, ()))
                if len(simulator.OUT_LINKS[end_node]) == 1:
                    pending.append((simulator.OUT_LINKS[end_node][0], distance_mm))
        return sorted(members, key=lambda member: member.oht_id)

    def region_nodes(self, vehicle):
        state = self.get_position(vehicle)
        if state["link_index"] is None:
            return {vehicle.node}
        index = state["link_index"]
        return {self.simulator.LINK_A[index], self.simulator.LINK_B[index]}

    def get_physical_movement_options(self, vehicle):
        """모든 가까운 분기를 검사한다. 순환·조합 폭증 시 미완전으로 반환한다."""
        simulator = self.simulator
        current = vehicle._motion_link_index
        initial = [current] if current is not None else []
        origin = simulator.LINK_B[current] if current is not None else vehicle.node
        pending = [(initial, origin)]
        options = []
        while pending:
            path, node_id = pending.pop()
            if len(path) > 64 or len(options) + len(pending) > 128:
                return [], False
            distance_mm, front = self.find_front(vehicle, path, extend_after_route=False) if path else (math.inf, None)
            path_length_mm = sum(simulator.LINK_LEN[index] for index in path) - (
                self.get_position(vehicle)["offset_mm"] if current is not None else 0)
            if front is not None and distance_mm <= simulator.FOLLOW_CENTER_GAP_MM + self.numerical_margin_mm:
                options.append({"blocking_vehicle_id": front.oht_id, "distance_mm": distance_mm,
                                "link_indices": path})
            elif path_length_mm >= simulator.FOLLOW_CENTER_GAP_MM or not simulator.OUT_LINKS[node_id]:
                options.append({"blocking_vehicle_id": None, "distance_mm": None, "link_indices": path})
            else:
                for link_index in simulator.OUT_LINKS[node_id]:
                    pending.append((path + [link_index], simulator.LINK_B[link_index]))
        return options, True

    def check_safety(self):
        simulator = self.simulator
        for link_index, members in self.link_vehicles.items():
            active = [member for member in members if self.get_position(member)["offset_mm"] < simulator.LINK_LEN[link_index] - 1e-6]
            if len(active) > math.ceil(simulator.LINK_LEN[link_index] / simulator.FOLLOW_CENTER_GAP_MM):
                self.fail(active[0], "링크 내 차량 중심 수용 상한 초과")
        for vehicle in simulator.OHT_LIST:
            self.check_vehicle_safety(vehicle)

    def check_vehicle_safety(self, vehicle):
        simulator = self.simulator
        state = self.get_position(vehicle)
        index = state["link_index"]
        occupied_control_nodes = []
        if index is None and vehicle.node in self.control_nodes:
            occupied_control_nodes.append(vehicle.node)
        elif index is not None:
            if simulator.LINK_A[index] in self.control_nodes and state["offset_mm"] < simulator.OHT_LENGTH_MM / 2:
                occupied_control_nodes.append(simulator.LINK_A[index])
            if simulator.LINK_B[index] in self.control_nodes and simulator.LINK_LEN[index] - state["offset_mm"] < simulator.OHT_LENGTH_MM / 2:
                occupied_control_nodes.append(simulator.LINK_B[index])
        for node_id in occupied_control_nodes:
            if simulator.NODE_HOLDER.get(node_id) is not vehicle:
                self.fail(vehicle, "제어점 실제 차체와 점유 자원 불일치", controlled_node=node_id)
        if state["port_id"] is not None:
            port = simulator.PORT_BY_ID[state["port_id"]]
            key = (port["parent_link"], float(port["rel_dist"]))
            if self.port_claims.get(key) is not vehicle:
                self.fail(vehicle, "이적재 실제 정차 자리 중복 점유", port_id=state["port_id"])
        links = self.routes.get(vehicle, [])
        if not links and vehicle._motion_link_index is not None:
            links = [vehicle._motion_link_index]
        if links:
            distance_mm, front = self.find_front(vehicle, links)
            if front is not None and distance_mm < simulator.FOLLOW_CENTER_GAP_MM - 1e-6:
                self.fail(vehicle, "관측 시점의 차간거리 위반", front_oht_id=front.oht_id, distance_mm=distance_mm)

    def get_wait_summary(self, now):
        current_durations = [now - record["since_second"] for record in self.wait_records.values()]
        count = self.finished_wait_count + len(current_durations)
        return {"observed_wait_period_count": count,
            "mean_wait_period_seconds": (self.finished_wait_seconds + sum(current_durations)) / count if count else 0.0,
            "maximum_wait_period_seconds": max([self.maximum_finished_wait_seconds] + current_durations),
            "is_current_wait_right_censored": bool(current_durations)}

    def is_ring_blocked(self, vehicle, node_id):
        simulator = self.simulator
        for number in simulator.NODE_RINGS.get(node_id, ()):
            if vehicle in simulator.RING_MEMBERS[number] or vehicle in self.ring_reservations[number]:
                continue
            ring = simulator.RINGS[number]
            if len(simulator.RING_MEMBERS[number] | self.ring_reservations[number]) < ring["cap"]:
                continue
            if not ring.get("hard") and (vehicle._ring_exempt or (vehicle._stuck_since is not None
                    and vehicle.env.now() - vehicle._stuck_since >= simulator.RING_WAIT_OVERRIDE_S)):
                continue
            if not ring.get("hard") and any(vehicle in members and node_id not in simulator.RINGS[other_number]["nodes"]
                    and len(members) >= simulator.RINGS[other_number]["cap"]
                    for other_number, members in enumerate(simulator.RING_MEMBERS)):
                continue
            return True
        return False

    def estimate_remaining_seconds(self, vehicle):
        links = self.routes.get(vehicle, [])
        if not links:
            return 0.0
        state = self.get_position(vehicle)
        port = self.target_ports.get(vehicle)
        return self.calculate_route_profile(links, state["offset_mm"] if state["link_index"] is not None else 0.0,
            state["speed_mm_per_second"], float(port["rel_dist"]) if port is not None else None)["duration_seconds"]

    def count_straight_behind(self, vehicle):
        seen = {vehicle}
        pending = [vehicle]
        count = 0
        while pending and len(seen) < 64:
            leader = pending.pop()
            for follower, record in self.wait_records.items():
                if follower not in seen and leader in record["blocking_vehicles"]:
                    seen.add(follower)
                    pending.append(follower)
                    if follower._merge_wait != vehicle._merge_wait:
                        count += 1
        return count

    def check_stalls(self, now):
        simulator = self.simulator
        handled = set()
        counts = {"fired": 0, "relayed": 0, "forced": 0}
        for vehicle, record in list(self.wait_records.items()):
            if (now - record["since_second"] < simulator.JUNCTION_STALL_REROUTE
                    or vehicle._replan or vehicle._ring_waiting):
                continue
            target = self.find_queue_head(vehicle)
            if (target is None or target in handled or target._replan or target._abort_leg or target._ring_waiting):
                continue
            if now - target._motion_last_stall_reroute_second < simulator.JUNCTION_STALL_REROUTE:
                continue
            handled.add(target)
            current = target._motion_link_index
            links = self.routes.get(target, [])
            blocked_link = links[1] if current is not None and len(links) > 1 else (links[0] if links else None)
            if blocked_link is None:
                continue
            if target._first_stall_t is None or record["since_second"] < target._first_stall_t:
                target._first_stall_t = record["since_second"]
            target._stall_avoid_links.add(blocked_link)
            is_forced = now - target._first_stall_t >= simulator.FORCE_DETOUR_SECONDS
            target._avoid_links = set(target._stall_avoid_links) if is_forced else {blocked_link}
            target._replan = True
            target._motion_last_stall_reroute_second = now
            target._last_reroute_t = now
            counts["fired"] += 1
            counts["relayed"] += target is not vehicle
            counts["forced"] += is_forced
            simulator.REROUTE_COUNTS["stall"] += 1
            if target.ispassive():
                target.activate()
        return counts

    def find_queue_head(self, vehicle):
        simulator = self.simulator
        pending = [vehicle]
        seen = set()
        while pending and len(seen) < 64:
            candidate = pending.pop()
            if candidate in seen or candidate not in self.wait_records:
                continue
            seen.add(candidate)
            current = candidate._motion_link_index
            decision_node = simulator.LINK_B[current] if current is not None else candidate.node
            if decision_node in simulator.JUNCTION_NODES:
                return candidate
            pending.extend(self.wait_records[candidate]["blocking_vehicles"])
        return None
