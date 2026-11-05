"""제공 버전 2의 기본 제어를 유지하는 단일 헤드리스 실험 실행."""

import argparse
import importlib.metadata
import json
import math
import os
from pathlib import Path
import platform
import resource
import signal
import sys
import time
import traceback

from experiment_support import (
    current_timestamp, file_sha256, json_default, load_simulator, simulation_signature,
    verify_provided_files, write_json,
)
from integrated_deadlock_observer import IntegratedDeadlockRecorder, PROTOCOL_VERSION


def create_observer_component(simulator, recorder):
    class ReadOnlyIntegratedObserver(simulator.sim.Component):
        def process(self):
            while True:
                recorder.observe(self.env.now())
                yield self.hold(recorder.observation_interval_seconds)
    return ReadOnlyIntegratedObserver()


def run_with_observation(simulator, num_oht, sim_duration, spawn_interval, seed, recorder):
    """run_headless_sim의 초기화 순서를 그대로 사용하고 관측 컴포넌트만 마지막에 추가.

    이 함수와 제공 run_headless_sim의 최종 제어 상태·반송 기록 일치를 별도
    프로세스에서 검증해야 본실험 실행이 허용된다. 제공 소스 변경 없음.
    """
    if seed is not None:
        simulator.random.seed(seed)
    simulator.reset_sim_state()
    prev_spawn_interval = simulator.SPAWN_INTERVAL
    simulator.SPAWN_INTERVAL = spawn_interval
    simulator.sim.yieldless(False)
    env = simulator.sim.Environment(trace=False)
    park_nodes = simulator.get_park_nodes(num_oht)
    ohts = [simulator.OHT(park_node=node) for node in park_nodes]
    simulator.manager = simulator.Manager()
    simulator.dispatcher = simulator.Dispatcher()
    simulator.StallWatchdog()
    simulator.CongestionMonitor()
    simulator.IdleDistributor()
    if simulator.PROGRESS_INTERVAL:
        simulator.ProgressReporter(interval=simulator.PROGRESS_INTERVAL, total=sim_duration)
    observer = create_observer_component(simulator, recorder)
    try:
        env.run(till=sim_duration)
        recorder.observe(env.now(), force_snapshot=True)
    finally:
        simulator.SPAWN_INTERVAL = prev_spawn_interval
    return env.now()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case-directory", type=Path, required=True)
    parser.add_argument("--output-directory", type=Path, required=True)
    parser.add_argument("--num-oht", dest="num_oht", type=int, default=1000)
    parser.add_argument("--sim-duration", dest="sim_duration", type=float, default=86400)
    parser.add_argument("--spawn-interval", dest="spawn_interval", type=float, default=0.5)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--occupancy-mode", choices=("node", "segment"), default="node")
    parser.add_argument("--merge-mode", choices=("OCS", "ADAPTIVE", "FCFS"), default=None)
    parser.add_argument("--without-observation", action="store_true")
    parser.add_argument("--risk-hold-seconds", type=float, default=30)
    parser.add_argument("--long-wait-seconds", type=float, default=60)
    arguments = parser.parse_args()
    case_directory = arguments.case_directory.resolve()
    output_directory = arguments.output_directory.resolve()
    if output_directory.exists():
        raise FileExistsError(f"실험 결과 덮어쓰기 금지: {output_directory}")
    output_directory.mkdir(parents=True)
    for name in ("실행로그", "최종집계", "중간집계"):
        (output_directory / name).mkdir()
    source_manifest = verify_provided_files()
    input_names = ("agv_graph.json", "equipment.json", "section_regions.json", "track_segments.json",
                   "oht_park_positions.json", "oht_lot_schedule.json")
    input_hashes = {name: file_sha256(case_directory / name) for name in input_names}
    simulator = load_simulator(case_directory, occupancy_mode=arguments.occupancy_mode)
    if arguments.merge_mode is not None:
        simulator.MERGE_CONTROL_MODE = arguments.merge_mode
    if arguments.num_oht > len(simulator.OHT_PARK_POSITIONS):
        raise ValueError("시작 배치 개수 부족")
    required_lots = math.ceil(arguments.sim_duration / arguments.spawn_interval) + 1
    if len(simulator.JOB_SCHEDULE) < required_lots:
        raise ValueError(f"24시간 관측용 입력 부족: 필요 {required_lots}, 준비 {len(simulator.JOB_SCHEDULE)}")
    case_manifest_path = case_directory / "입력생성조건.json"
    manifest = {
        "status": "실행중", "started_at": current_timestamp(), "basis": "제공 버전 2",
        "mode": "작업본 헤드리스 함수" if arguments.without_observation else "작업본 주행·읽기 전용 상태 관측",
        "num_oht": arguments.num_oht, "sim_duration": arguments.sim_duration,
        "process_id": os.getpid(),
        "spawn_interval": arguments.spawn_interval, "seed": arguments.seed,
        "case_directory": str(case_directory), "input_file_sha256": input_hashes,
        "case_generation": json.loads(case_manifest_path.read_text()) if case_manifest_path.exists() else "동봉 입력",
        "provided_file_sha256": source_manifest["provided_file_sha256"],
        "occupancy_mode": arguments.occupancy_mode,
        "segment_model": {"control_clearance_mm": simulator.FOLLOW_CENTER_GAP_MM,
            "control_clearance_basis": "기존 차체 길이+정지 간격. 전환기 실측 길이가 아님. 제공 v1.9 LINK의 ReleaseDistance는 빈 값",
            "travel_time_basis": "실제 링크 위치·속도·T 정차점, 기존 가감속 식과 JUNCTION_SPEED, 기존 LINK_PENALTY. 미래 허가·앞차 대기는 미포함",
            "merge_pass_wait_definition": "직전 분기 이후 또는 전방 관측 범위의 첫 합류점 접근 중 실제 정지한 누적 시간. 주행 중 사전 요청 시간 제외",
            "merge_wait_overlap": "node_wait는 통과 차량의 접근 정지 시간. reserve_wait는 미통과 차량도 포함한 OCS 허가 대기시간. 겹치는 시간이 있으므로 합산 금지"}
            if arguments.occupancy_mode == "segment" else None,
        "working_simulator_sha256": file_sha256(Path(simulator.__file__)),
        "segment_motion_sha256": file_sha256(Path(simulator.__file__).with_name("segment_motion.py")),
        "continuous_travel_time_sha256": file_sha256(Path(simulator.__file__).with_name("continuous_travel_time.py")),
        "runtime_python": sys.executable, "python_version": platform.python_version(),
        "dependency_versions": {name: importlib.metadata.version(name) for name in ("salabim", "numpy", "scipy", "pygame", "Pillow")},
        "provided_control_settings": {name: getattr(simulator, name) for name in (
            "FOLLOW_CHECK_MODE", "FOLLOW_CENTER_GAP_MM", "MERGE_CONTROL_MODE", "CONGESTION_WEIGHT",
            "ENABLE_RING_CONTROL", "RING_CAP_MARGIN", "ENABLE_MERGE_DEADLOCK_BREAK", "ENABLE_MERGE_OUT_FULL",
            "MERGE_REQUEST_AT_MERGE_ENTRY", "RING_WAIT_OVERRIDE_S", "DISPATCH_INTERVAL")},
        "observation": {"protocol": PROTOCOL_VERSION, "target_types": ["TYPE I", "TYPE II", "TYPE III", "TYPE IV"],
            "source_material": "OHT Deadlock 대표 시나리오 10가지 1~4번",
            "observation_interval_seconds": 1, "automatic_structural_sufficient_condition_inspection": True,
            "candidate_stable_seconds": 15, "key_cycle_photo_stable_seconds": 120,
            "long_wait_photo_seconds": 600, "region_aggregation_seconds": 60,
            "before_scene_reference": "관련 차량의 최초 실제 대기 시작 기준 300·120·60초 전. 없으면 미확보로 기록",
            "confirmation_photo": "즉시 전체·확대 지도와 독립 전체 상태 저장",
            "limitation": "매초 관측 및 충분조건 검사. 모든 교착의 부재를 증명하지 않음"},
    }
    write_json(output_directory / "실험조건.json", manifest)
    recorder = None
    wall_started = time.monotonic()
    completed = False
    simulation_second = 0.0

    def handle_shutdown(signum, frame):
        raise KeyboardInterrupt(f"중단 신호 {signum}")

    signal.signal(signal.SIGTERM, handle_shutdown)
    try:
        if arguments.without_observation:
            result = simulator.run_headless_sim(num_oht=arguments.num_oht,
                sim_duration=arguments.sim_duration, spawn_interval=arguments.spawn_interval, seed=arguments.seed)
            simulation_second = result["sim_duration"]
            write_json(output_directory / "최종집계" / "제공함수결과.json", result)
        else:
            recorder = IntegratedDeadlockRecorder(simulator, output_directory, arguments.sim_duration, case_directory)
            simulation_second = run_with_observation(simulator, arguments.num_oht, arguments.sim_duration,
                                                     arguments.spawn_interval, arguments.seed, recorder)
        write_json(output_directory / "최종집계" / "동작대조서명.json", {
            "state_and_original_records_sha256": simulation_signature(simulator),
            "completed_transfers": len(simulator.JOB_STATS), "completed_lots": len(simulator.LOT_STATS),
            "injected_lots": simulator._JOB_SCHEDULE_IDX[0], "simulation_second": simulation_second})
        # 제공 기록을 제거하거나 비우지 않고 압축하여 별도 보존.
        import gzip
        for name, records in (("완료반송기록", simulator.JOB_STATS), ("완료로트기록", simulator.LOT_STATS),
                              ("제공정체기록", simulator.STALL_EVENTS), ("합류통과기록", simulator.MERGE_PASS_LOG)):
            with gzip.open(output_directory / "최종집계" / (name + ".json.gz"), "wt", encoding="utf-8", compresslevel=1) as handle:
                json.dump(records, handle, ensure_ascii=False, separators=(",", ":"), default=json_default)
        completed = simulation_second >= arguments.sim_duration
    except BaseException:
        manifest["error"] = traceback.format_exc()
        if simulator.OHT_LIST:
            simulation_second = simulator.OHT_LIST[0].env.now()
        if recorder is not None and getattr(simulator, "ENABLE_SEGMENT_OCCUPANCY", False):
            try:
                recorder.capture_failure(simulation_second)
            except Exception:
                manifest["failure_capture_error"] = traceback.format_exc()
        raise
    finally:
        try:
            if recorder is not None:
                recorder.close(simulation_second, completed)
        finally:
            manifest.update(status="정상완료" if completed else "중단또는오류",
                ended_at=current_timestamp(), actual_simulation_second=simulation_second,
                wall_elapsed_seconds=time.monotonic() - wall_started,
                maximum_resident_memory_bytes=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
            verify_provided_files()
            if getattr(simulator, "ENABLE_SEGMENT_OCCUPANCY", False):
                controller = simulator.get_segment_motion()
                write_json(output_directory / "최종집계" / "주행안전검사.json", {
                    "is_valid": not controller.safety_failures and completed,
                    "safety_failures": controller.safety_failures,
                    "minimum_observed_gap_mm": None if math.isinf(controller.minimum_observed_gap_mm) else controller.minimum_observed_gap_mm,
                    "scope": "주행 중 제동 여유·간격·제어점 점유 검사. 오류 또는 미완료 실행의 처리량은 유효 결과로 사용하지 않음"})
                write_json(output_directory / "최종집계" / "주행처리지표.json",
                           build_motion_metrics(simulator, controller, simulation_second, completed))
            write_json(output_directory / "실험조건.json", manifest)
            print(json.dumps({name: manifest[name] for name in (
                "status", "actual_simulation_second", "wall_elapsed_seconds", "maximum_resident_memory_bytes")}, ensure_ascii=False), flush=True)


def build_motion_metrics(simulator, controller, now, completed):
    """계속 대기/주행 중인 마지막 부분도 포함하되 시뮬레이터 통계는 변경하지 않음."""
    rows = []
    completed_active_seconds = sum(record["travel"] + record["process"] + record["loadunload"] for record in simulator.JOB_STATS)
    current_active_seconds = 0.0
    for vehicle in simulator.OHT_LIST:
        state = controller.get_position(vehicle)
        wait_record = controller.wait_records.get(vehicle)
        pending_wait_seconds = max(0.0, now - wait_record["accounted_until_second"]) if wait_record else 0.0
        follow_seconds = vehicle.total_follow_stall_time + (pending_wait_seconds if wait_record and wait_record["cause"] == "follow_distance" else 0.0)
        lock_seconds = vehicle.total_lock_stall_time + (pending_wait_seconds if wait_record and wait_record["cause"] != "follow_distance" else 0.0)
        busy_seconds = vehicle.total_busy + (now - vehicle.job_start_t if vehicle.job is not None and vehicle.job_start_t is not None else 0.0)
        if vehicle.job is not None:
            current_active_seconds += vehicle._job_travel + vehicle._job_process + vehicle._job_loadunload
            if vehicle._motion_plan is not None:
                current_active_seconds += min(now - vehicle._motion_plan["start_second"], vehicle._motion_plan["profile"]["dur"])
            if vehicle.state in (simulator.OHT_LOADING, simulator.OHT_UNLOADING):
                start_second = vehicle.job.deposit_t if vehicle.state == simulator.OHT_LOADING else vehicle.job.pickup_t
                if start_second is not None:
                    current_active_seconds += min(simulator.OHT_LOAD_UNLOAD_TIME, now - start_second)
        rows.append({"oht_id": vehicle.oht_id, "logical_node": vehicle.node,
            "total_distance_mm": state["total_distance_mm"], "busy_seconds": busy_seconds,
            "total_lock_stall_seconds": lock_seconds, "total_follow_stall_seconds": follow_seconds})
    denominator = len(rows) * now
    return {"is_valid_experiment": completed and not controller.safety_failures,
        "simulation_seconds": now, "completed_lots": len(simulator.LOT_STATS), "completed_transfers": len(simulator.JOB_STATS),
        "last_transfer_completed_second": max((record["t_end"] for record in simulator.JOB_STATS), default=None),
        "busy_ratio": sum(row["busy_seconds"] for row in rows) / denominator if denominator else 0.0,
        "job_active_ratio": (completed_active_seconds + current_active_seconds) / denominator if denominator else 0.0,
        "total_lock_stall_seconds": sum(row["total_lock_stall_seconds"] for row in rows),
        "total_follow_stall_seconds": sum(row["total_follow_stall_seconds"] for row in rows),
        "physical_wait_ratio": sum(row["total_lock_stall_seconds"] + row["total_follow_stall_seconds"] for row in rows) / denominator if denominator else 0.0,
        **controller.get_wait_summary(now), "vehicles": rows,
        "definitions": "busy는 작업 배정 중 비율, active는 작업 주행·이적재 시간 비율. physical_wait는 유휴 비켜주기를 포함한 실제 정지 대기. 진행 중 대기는 관측 종료로 잘림"}


if __name__ == "__main__":
    main()
