# -*- coding: utf-8 -*-
"""데드락 탐지기.

원본 시뮬레이터에는 데드락 탐지 기능이 없다. 있는 것은 아래 한 줄뿐이다.

    gridlock = (n_jobs == 0 and num_oht >= 20)

이것은 "완료 건수가 0이냐"를 보는 처리량 지표라서, 데드락인지 단순 정체인지
구분하지 못하고 어디서 누가 걸렸는지도 알 수 없다. 그래서 따로 만든다.

판정 구조는 교수님 시뮬레이터(oht_deadlock_simulator.html)의
detectDeadlock() 과 같다.

    1) 막혀 있는 차마다 "나를 막는 차"를 하나 정한다
    2) 그 대기 그래프에서 닫힌 순환을 찾는다
    3) 순환 구성원 전원이 일정 시간 이상 멈춰 있으면 교착으로 본다

다만 두 모델의 차이 때문에 그대로는 쓸 수 없다.

    교수님 모델은 "앞차의 차체가 막는다"는 대기 관계가 있다. 차체는 반납할
    수 없으므로 그런 순환은 영원히 풀리지 않는다. 그래서 15초면 확인으로
    충분하다.

    우리 모델에는 차체 개념이 없어 모든 대기가 노드 락이고, 락은 반납될 수
    있어 순환이 풀린다. 실제로 7조건 5시간 실험에서 순환이 406초까지
    지속됐다가 풀린 사례가 있다. 따라서 임계를 미리 정하지 않고
    순환의 생애(시작~종료)를 전부 기록한 뒤, 그 분포를 보고 판단한다.

환경변수 (약어를 쓰지 않는다)

    OHT_COUNT                   OHT 대수                  기본 400
    SPAWN_INTERVAL_SECONDS      로트 투입 간격(초)         기본 0.45 (= 8,000건/시간)
    SIMULATION_HOURS            시뮬레이션 시간 상한(시간)  기본 6
    CHECK_INTERVAL_SECONDS      점검 주기(시뮬 초)         기본 1
    APPLY_FIXES                 1이면 수정 적용, 0이면 원문 기본 1
    ENABLE_RING_CONTROL         베이 고리 진입 제어 0/1    기본 유지
    RING_CAPACITY_MARGIN        고리 정원 = 노드수//2 - 이 값
    STOP_AFTER_SECONDS          순환이 이 시간 이상 지속되면 교착으로 보고 종료  기본 600
    STOP_WHEN_ALL_LOTS_DONE     로트를 전부 투입하고 전부 완료하면 종료  기본 1
    PENDING_JOB_LIMIT           대기열 상한(건). 넘으면 투입을 건너뛴다. 0이면 무제한
    CASE_DIRECTORY              시드 케이스 폴더. 생략하면 현재 폴더의 파일을 쓴다
    RESULT_JSON_PATH            결과 저장 경로
"""
import os
import sys
import json
import time

sys.argv = ["agv_fab_sim.py", "--analyze"]        # pygame 창 없이 불러오기
SCRIPT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIRECTORY)

# [시드 케이스] 본체는 데이터 파일을 파일명만으로 연다(= 현재 작업 폴더 기준).
# 그래서 케이스 폴더로 옮겨간 뒤에 불러오면 그 폴더의 시작위치/로트 스케줄을
# 쓰게 된다. 폴더마다 독립이라 여러 시드를 동시에 돌려도 서로 간섭하지 않는다.
# 결과 경로는 폴더를 옮기기 전에 절대경로로 바꿔둔다.
CASE_DIRECTORY = os.environ.get("CASE_DIRECTORY", "")
if os.environ.get("RESULT_JSON_PATH"):
    os.environ["RESULT_JSON_PATH"] = os.path.abspath(os.environ["RESULT_JSON_PATH"])
if CASE_DIRECTORY:
    CASE_DIRECTORY = os.path.abspath(CASE_DIRECTORY)
    os.chdir(CASE_DIRECTORY)

# 케이스 명세(어떤 시드로 만든 폴더인지)를 읽어둔다. 결과만 보고도
# "어느 시드에서 난 데드락인지" 추적할 수 있어야 한다.
CASE_INFO = {}
if os.path.exists("case_info.json"):
    with open("case_info.json", encoding="utf-8") as _handle:
        CASE_INFO = json.load(_handle)

import agv_fab_sim as simulator
import salabim as sim


def _environment_value(name, cast, default):
    raw = os.environ.get(name)
    return default if raw is None else cast(raw)


OHT_COUNT              = _environment_value("OHT_COUNT", int, 400)
SPAWN_INTERVAL_SECONDS = _environment_value("SPAWN_INTERVAL_SECONDS", float, 0.45)
SIMULATION_HOURS       = _environment_value("SIMULATION_HOURS", float, 6.0)
CHECK_INTERVAL_SECONDS = _environment_value("CHECK_INTERVAL_SECONDS", float, 1.0)
APPLY_FIXES            = _environment_value("APPLY_FIXES", lambda v: v != "0", True)
STOP_AFTER_SECONDS     = _environment_value("STOP_AFTER_SECONDS", float, 600.0)
STOP_WHEN_ALL_LOTS_DONE = _environment_value(
    "STOP_WHEN_ALL_LOTS_DONE", lambda v: v != "0", True)
RESULT_JSON_PATH       = os.environ.get("RESULT_JSON_PATH", "")

# 보고용 구간. 교수님 기준(15초)을 포함시켜 같은 조건으로 비교할 수 있게 한다.
REPORT_THRESHOLDS_SECONDS = (15.0, 60.0, 300.0, 600.0)

simulator.FOLLOW_DISTANCE_CHECK_BEFORE_OCCUPANCY = APPLY_FIXES
simulator.RELEASE_PREVIOUS_NODE_AT_SAFE_DISTANCE = APPLY_FIXES
simulator.DECELERATE_BEFORE_BLOCKED_NODE         = APPLY_FIXES
if not APPLY_FIXES:
    # 원문은 차간 최소거리에 차체 길이를 더하지 않는다.
    simulator.MIN_FOLLOW_DISTANCE = simulator.OHT_STOP_GAP_MM
if "ENABLE_RING_CONTROL" in os.environ:
    simulator.ENABLE_RING_CONTROL = bool(int(os.environ["ENABLE_RING_CONTROL"]))
if "RING_CAPACITY_MARGIN" in os.environ:
    simulator.RING_CAP_MARGIN = int(os.environ["RING_CAPACITY_MARGIN"])


def find_blocking_vehicle(vehicle, next_node):
    """vehicle 을 실제로 막고 있는 차량. 없으면 None.

    (가) 들어가려는 노드를 다른 차가 쥐고 있으면 그 차.
    (나) 그 노드는 비었는데 차간거리 때문에 못 가는 경우. 차간거리로 막힌
         차량은 잡았던 노드 락을 도로 반납하므로, 들어가려는 노드만 보면
         비어 있어서 "안 막힌 것"으로 오판한다. 진짜로 막고 있는 것은 기준
         거리 안쪽 노드를 쥔 차다. 본체 _path_clear_ahead() 와 같은 방식으로
         훑는다(거리를 더하기 전에 기준 초과 여부를 먼저 판정).
    """
    holder = simulator.NODE_HOLDER.get(next_node)
    if holder is not None and holder is not vehicle:
        return holder

    path = getattr(vehicle, "_leg_path", None)
    if not path:
        return None
    index = None
    search_from = max(0, (getattr(vehicle, "_leg_idx", 0) or 0) - 1)
    for position in range(search_from, len(path)):
        if path[position] == next_node:
            index = position
            break
    if index is None:
        return None

    accumulated_mm = 0.0
    position = index
    while position < len(path) - 1 and accumulated_mm < simulator.MIN_FOLLOW_DISTANCE:
        here, ahead = path[position], path[position + 1]
        next_accumulated = accumulated_mm + simulator.edge_length(here, ahead)
        if (simulator.FOLLOW_DISTANCE_CHECK_BEFORE_OCCUPANCY
                and next_accumulated > simulator.MIN_FOLLOW_DISTANCE):
            break
        accumulated_mm = next_accumulated
        holder = simulator.NODE_HOLDER.get(ahead)
        if holder is not None and holder is not vehicle:
            return holder
        position += 1
    return None


def build_wait_graph():
    """{막힌 차 -> 나를 막는 차} 와 {막힌 차 -> 멈춘 시각} 을 만든다."""
    wait_for, stopped_since = {}, {}
    for vehicle in simulator.OHT_LIST:
        next_node = getattr(vehicle, "_stall_next", None)
        stall_start = getattr(vehicle, "_stall_start", None)
        if next_node is None or stall_start is None:
            continue                       # 한 칸이라도 움직이면 _stall_start 가 None 이 된다
        blocker = find_blocking_vehicle(vehicle, next_node)
        if blocker is None:
            continue
        wait_for[id(vehicle)] = id(blocker)
        stopped_since[id(vehicle)] = stall_start
    return wait_for, stopped_since


def find_cycles(wait_for):
    """대기 그래프에서 닫힌 순환을 전부 찾는다.

    대기 대상이 차량당 정확히 하나라 사슬을 따라가다 이미 지난 차로
    돌아오면 그 구간이 순환이다."""
    cycles, visit_state = [], {}
    for start in list(wait_for):
        if visit_state.get(start):
            continue
        chain, current = [], start
        while current is not None and current in wait_for and not visit_state.get(current):
            visit_state[current] = 1
            chain.append(current)
            current = wait_for[current]
        if current is not None and visit_state.get(current) == 1 and current in chain:
            cycles.append(chain[chain.index(current):])
        for member in chain:
            visit_state[member] = 2
    return cycles


PENDING_JOB_LIMIT = _environment_value("PENDING_JOB_LIMIT", int, 2000)

# [투입 제어] 원본 Manager 는 SPAWN_INTERVAL 마다 조건 없이 로트를 꺼낸다.
# 그래서 두 가지 문제가 있다.
#
#   1) 로트 재고가 바닥나면 next_lot_entry() 가 ValueError 를 던지고
#      실행이 통째로 죽는다. 결과 파일도 안 남는다.
#   2) 투입(8,000건/시간)이 처리 능력(약 950 로트/시간)보다 훨씬 많아서,
#      그냥 두면 대기열에 수십만 건이 쌓인다. 그 숫자는 실험에 아무
#      정보도 주지 않으면서 메모리만 먹는다.
#
# 그래서 대기열이 가득 차면 그 회차 투입을 건너뛴다. 교수님 시뮬레이터도
# 같은 방식이다(jobs.length > 800 이면 생성 건너뜀). 포화 상태에서는
# 투입이 곧 완료 속도에 맞춰지므로 실질적인 물동량은 달라지지 않는다.
#
# 본체(agv_fab_sim.py)는 건드리지 않는다. 모델 수정이 아니라 실험 조건이라
# 실험 스크립트 쪽에 두는 것이 맞다.
INJECTION_SKIPPED = {"backlog_full": 0, "schedule_empty": 0}
_original_create_job = simulator.Manager.create_job


def _create_job_with_backlog_limit(self):
    if simulator._JOB_SCHEDULE_IDX[0] >= len(simulator.JOB_SCHEDULE):
        INJECTION_SKIPPED["schedule_empty"] += 1
        return None
    if 0 < PENDING_JOB_LIMIT <= len(simulator.PENDING_JOBS):
        INJECTION_SKIPPED["backlog_full"] += 1
        return None
    return _original_create_job(self)


simulator.Manager.create_job = _create_job_with_backlog_limit


def measure_utilization(vehicles, elapsed_seconds):
    """OHT 부하율. 본체 run_headless_sim() 과 같은 식으로 계산한다.

    가동률  작업을 배정받고 있던 시간의 비율. 대기 시간도 포함되므로
            정체가 심해질수록 올라간다(원본 주석이 이 역설을 인정한다).
            교수님이 말씀하신 "부하율 80 85 90 95" 가 이 값이다.
    실동률  그중 실제로 주행/가공/싣고내리기에 쓴 시간만.
    대기율  그중 노드락·설비 대기로 멈춰 있던 시간.

    완료된 작업(JOB_STATS)과 종료 시점에 아직 진행 중이던 작업을 함께
    더한다. 진행 중인 것을 빼면, 혼잡이 심해 완료가 0에 가까울 때
    "다들 꽉 막혀 바쁜" 상태가 오히려 0%로 계산되는 문제가 생긴다.
    """
    if not vehicles or elapsed_seconds <= 0:
        return 0.0, 0.0, 0.0
    denominator = len(vehicles) * elapsed_seconds
    in_flight = [v for v in vehicles
                 if v.job is not None and v.job_start_t is not None]

    busy = (sum(v.total_busy for v in vehicles)
            + sum(elapsed_seconds - v.job_start_t for v in in_flight))
    active = (sum(j["travel"] + j["process"] + j["loadunload"]
                  for j in simulator.JOB_STATS)
              + sum(v._job_travel + v._job_process + v._job_loadunload
                    for v in in_flight))
    waiting = (sum(j["wait"] for j in simulator.JOB_STATS)
               + sum(v._job_wait for v in in_flight))
    return busy / denominator, active / denominator, waiting / denominator


def all_lots_finished():
    """투입할 로트를 다 꺼냈고, 진행 중인 작업도 전부 끝났는가.

    투입이 끝난 것만으로는 종료하지 않는다. 마지막에 투입된 로트가 아직
    운반 중일 수 있기 때문이다. 스케줄을 다 소진하고 JOB_POOL(아직 끝나지
    않은 작업)과 PENDING_JOBS(미할당 작업)가 모두 비었을 때가 진짜 완주다.
    """
    schedule_used = simulator._JOB_SCHEDULE_IDX[0] >= len(simulator.JOB_SCHEDULE)
    nothing_in_flight = not simulator.JOB_POOL and not simulator.PENDING_JOBS
    return schedule_used and nothing_in_flight


class DeadlockWatcher(sim.Component):
    """시뮬 CHECK_INTERVAL_SECONDS 마다 상태를 읽는다. 읽기만 한다."""

    def setup(self):
        self.open_cycles = {}        # frozenset(멤버) -> 기록
        self.closed_cycles = []      # 풀린 순환의 생애 기록
        self.peak_waiting = 0
        self.peak_cycle_size = 0
        self.stopped_early = False
        self.finished_all_lots = False

    def process(self):
        vehicle_by_id = {id(v): v for v in simulator.OHT_LIST}
        while True:
            yield self.hold(CHECK_INTERVAL_SECONDS)
            now = self.env.now()

            if STOP_WHEN_ALL_LOTS_DONE and all_lots_finished():
                print("  >>> 로트 %d건을 전부 투입하고 전부 완료했다. 정상 완주. (t=%.0f초)"
                      % (len(simulator.JOB_SCHEDULE), now), flush=True)
                self.finished_all_lots = True
                self._close_all(now, "run_ended")
                self.env.main().activate()
                return

            wait_for, stopped_since = build_wait_graph()
            self.peak_waiting = max(self.peak_waiting, len(wait_for))

            alive_keys = set()
            for cycle in find_cycles(wait_for):
                if any(member not in stopped_since for member in cycle):
                    continue
                key = frozenset(cycle)
                alive_keys.add(key)
                # 전원이 멈춰 있던 시간 = 가장 늦게 멈춘 차 기준
                held_seconds = now - max(stopped_since[m] for m in cycle)
                self.peak_cycle_size = max(self.peak_cycle_size, len(cycle))

                record = self.open_cycles.get(key)
                if record is None:
                    record = self._new_record(now, cycle, wait_for, vehicle_by_id)
                    self.open_cycles[key] = record
                record["last_seen_second"] = round(now, 1)
                record["held_seconds"] = round(held_seconds, 1)

                if held_seconds >= STOP_AFTER_SECONDS:
                    print("  >>> %.0f초 이상 지속. 교착으로 보고 실행을 멈춘다. "
                          "(t=%.0f초, 순환 %d대)" % (STOP_AFTER_SECONDS, now, len(cycle)),
                          flush=True)
                    self.stopped_early = True
                    self._close_all(now, "held_over_limit")
                    self.env.main().activate()
                    return

            # 이번 점검에서 사라진 순환은 풀린 것으로 보고 종료 시각을 적는다
            for key in [k for k in self.open_cycles if k not in alive_keys]:
                record = self.open_cycles.pop(key)
                record["ended_second"] = round(now, 1)
                record["ended_reason"] = "resolved"      # 스스로 풀렸다
                self.closed_cycles.append(record)

    def _new_record(self, now, cycle, wait_for, vehicle_by_id):
        members = [vehicle_by_id[m] for m in cycle]
        nodes = [m.node for m in members]
        inside = set(cycle)
        affected = 0
        for key in wait_for:
            if key in inside:
                continue
            current, steps = key, 0
            while current is not None and current not in inside and steps < 600:
                current = wait_for.get(current)
                steps += 1
            if current in inside:
                affected += 1
        print("  [순환] t=%.0fs  %d대  영향 %d대  베이 %s  중앙통로 %s"
              % (now, len(cycle), affected,
                 sorted({simulator.NODE_TO_BAY_ID.get(n) for n in nodes} - {None}) or "-",
                 sorted({simulator.NODE_TO_CORRIDOR_ID.get(n) for n in nodes} - {None}) or "-"),
              flush=True)
        return {
            "started_second": round(now, 1),
            "ended_second": None,
            "ended_reason": None,
            "held_seconds": 0.0,
            "cycle_size": len(cycle),
            "affected_vehicles": affected,
            "oht_ids": [m.oht_id for m in members],
            "nodes": nodes,
            "bay_ids": sorted({simulator.NODE_TO_BAY_ID.get(n) for n in nodes} - {None}),
            "corridor_ids": sorted({simulator.NODE_TO_CORRIDOR_ID.get(n) for n in nodes} - {None}),
            "merge_ids": sorted({m for n in nodes for m in simulator.NODE_TO_MERGE_ID.get(n, [])}),
            "detected_ring_ids": sorted({r for n in nodes for r in simulator.NODE_RINGS.get(n, ())}),
        }

    def _close_all(self, now, reason):
        """아직 열려 있는 순환을 닫는다.

        reason 은 세 가지다.
          resolved          스스로 풀렸다
          held_over_limit   한계 시간을 넘겨 우리가 실행을 멈췄다 (교착으로 본다)
          run_ended         시뮬레이션 시간이 끝나 더 못 봤다 (풀렸을지 모른다)
        """
        for key in list(self.open_cycles):
            record = self.open_cycles.pop(key)
            record["ended_second"] = round(now, 1)
            record["ended_reason"] = reason
            self.closed_cycles.append(record)

    def all_records(self, now):
        # 시뮬이 끝나는 시점에 남아 있던 순환은 "풀렸는지 모른다"로 적는다.
        # 더 돌렸으면 풀렸을 수도 있으므로 교착으로 단정하지 않는다.
        self._close_all(now, "run_ended")
        return sorted(self.closed_cycles, key=lambda r: r["started_second"])


def main():
    simulator.reset_sim_state()
    simulator.SPAWN_INTERVAL = SPAWN_INTERVAL_SECONDS
    sim.yieldless(False)
    environment = sim.Environment(trace=False)

    vehicles = [simulator.OHT(park_node=node)
                for node in simulator.get_park_nodes(OHT_COUNT)]
    simulator.manager = simulator.Manager()
    simulator.dispatcher = simulator.Dispatcher()
    simulator.StallWatchdog()
    simulator.CongestionMonitor()
    simulator.IdleDistributor()
    watcher = DeadlockWatcher()

    if CASE_INFO:
        print("케이스 %s (시작위치 시드 %s / 로트 시드 %s)"
              % (CASE_INFO.get("case_name"), CASE_INFO.get("park_position_seed"),
                 CASE_INFO.get("lot_schedule_seed")))
    print("대수 %d / 투입 %.2f초(= %.0f건/시간) / 수정 %s / 차간 %.0fmm / 고리제어 %s"
          % (OHT_COUNT, SPAWN_INTERVAL_SECONDS, 3600 / SPAWN_INTERVAL_SECONDS,
             "적용" if APPLY_FIXES else "원문",
             simulator.MIN_FOLLOW_DISTANCE,
             "켬" if simulator.ENABLE_RING_CONTROL else "끔"), flush=True)

    started_at = time.time()
    environment.run(till=SIMULATION_HOURS * 3600.0)
    wall_seconds = time.time() - started_at
    simulated_seconds = environment.now()

    utilization, active, stalled = measure_utilization(vehicles, simulated_seconds)
    records = watcher.all_records(simulated_seconds)
    lifetimes = [r["held_seconds"] for r in records]
    counts = {("over_%ds" % int(t)): sum(1 for v in lifetimes if v >= t)
              for t in REPORT_THRESHOLDS_SECONDS}
    held_over_limit = [r for r in records if r["ended_reason"] == "held_over_limit"]
    open_at_end     = [r for r in records if r["ended_reason"] == "run_ended"]

    result = {
        "oht_count": OHT_COUNT,
        "spawn_interval_seconds": SPAWN_INTERVAL_SECONDS,
        "lots_per_hour": round(3600 / SPAWN_INTERVAL_SECONDS),
        "simulation_hours": SIMULATION_HOURS,
        "apply_fixes": APPLY_FIXES,
        "min_follow_distance_mm": simulator.MIN_FOLLOW_DISTANCE,
        "ring_control_enabled": simulator.ENABLE_RING_CONTROL,
        "ring_capacity_margin": simulator.RING_CAP_MARGIN,
        "stop_reason": ("deadlock_held_over_limit" if watcher.stopped_early
                        else "all_lots_completed" if watcher.finished_all_lots
                        else "time_limit_reached"),
        "stopped_early": watcher.stopped_early,
        "lots_injected": simulator._JOB_SCHEDULE_IDX[0],
        "lots_in_schedule": len(simulator.JOB_SCHEDULE),
        "jobs_still_in_flight": len(simulator.JOB_POOL),
        "simulated_seconds": round(simulated_seconds, 1),
        "wall_seconds": round(wall_seconds, 1),
        "realtime_factor": round(simulated_seconds / wall_seconds, 1) if wall_seconds else None,
        "completed_transfers": len(simulator.JOB_STATS),   # 반송 1건 = 한 포트에서 싣고 다른 포트에 내려놓기
        "completed_lots": len(simulator.LOT_STATS),        # 로트 1건 = STK 출발~가공~STK 복귀 전부 완료
        "pending_job_limit": PENDING_JOB_LIMIT,
        "pending_jobs_at_end": len(simulator.PENDING_JOBS),
        "injection_skipped_backlog_full": INJECTION_SKIPPED["backlog_full"],
        "injection_skipped_schedule_empty": INJECTION_SKIPPED["schedule_empty"],
        "case_directory": CASE_DIRECTORY or os.getcwd(),
        "case_info": CASE_INFO,
        "utilization_percent": round(utilization * 100, 1),
        "active_percent": round(active * 100, 1),
        "stall_percent": round(stalled * 100, 1),
        "peak_waiting_vehicles": watcher.peak_waiting,
        "peak_cycle_size": watcher.peak_cycle_size,
        "cycle_count": len(records),
        "longest_cycle_seconds": round(max(lifetimes), 1) if lifetimes else 0.0,
        "cycles_over_threshold": counts,
        "held_over_limit_count": len(held_over_limit),
        "open_at_run_end_count": len(open_at_end),
        "stop_after_seconds": STOP_AFTER_SECONDS,
        "cycles": records,
    }

    print("")
    print("종료 사유: %s" % {
        "deadlock_held_over_limit": "순환이 한계시간을 넘겨 교착으로 판정",
        "all_lots_completed": "로트 전부 완료 (정상 완주)",
        "time_limit_reached": "시뮬 시간 상한 도달",
    }[result["stop_reason"]])
    print("로트 투입 %d / %d건, 아직 진행 중인 작업 %d건"
          % (result["lots_injected"], result["lots_in_schedule"],
             result["jobs_still_in_flight"]))
    print("시뮬 %.0f초를 실제 %.0f초에 실행 (%.1f배속)"
          % (simulated_seconds, wall_seconds, result["realtime_factor"] or 0))
    print("반송 완료 %d건 / 로트 완료 %d건 / 대기 차량 최대 %d대 / 순환 최대 %d대"
          % (result["completed_transfers"], result["completed_lots"],
             watcher.peak_waiting, watcher.peak_cycle_size))
    print("대기열 상한 %d건 / 종료 시점 대기 %d건 / 투입 건너뜀 %d회(상한) %d회(재고소진)"
          % (PENDING_JOB_LIMIT, result["pending_jobs_at_end"],
             INJECTION_SKIPPED["backlog_full"], INJECTION_SKIPPED["schedule_empty"]))
    print("부하율(가동률) %.1f%%  /  실동률 %.1f%%  /  대기 %.1f%%"
          % (result["utilization_percent"], result["active_percent"],
             result["stall_percent"]))
    print("순환 %d건 발생, 최장 %.0f초" % (len(records), result["longest_cycle_seconds"]))
    for threshold in REPORT_THRESHOLDS_SECONDS:
        print("   %4.0f초 이상 지속: %d건" % (threshold, counts["over_%ds" % int(threshold)]))
    print("한계시간(%.0f초) 초과로 교착 판정: %d건" % (STOP_AFTER_SECONDS, len(held_over_limit)))
    print("시뮬 종료 시점에 열려 있던 순환(판정 보류): %d건" % len(open_at_end))
    if RESULT_JSON_PATH:
        with open(RESULT_JSON_PATH, "w", encoding="utf-8") as handle:
            json.dump(result, handle, ensure_ascii=False, indent=1)
        print("결과 저장: %s" % RESULT_JSON_PATH)


if __name__ == "__main__":
    main()
