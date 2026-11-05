"""제공 버전 2의 기본 제어를 유지하는 단일 헤드리스 실험 실행."""

import argparse
import importlib.metadata
import json
import math
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
from simulation_observer import SimulationRecorder, create_observer_component


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
    simulator = load_simulator(case_directory)
    if arguments.num_oht > len(simulator.OHT_PARK_POSITIONS):
        raise ValueError("시작 배치 개수 부족")
    required_lots = math.ceil(arguments.sim_duration / arguments.spawn_interval) + 1
    if len(simulator.JOB_SCHEDULE) < required_lots:
        raise ValueError(f"24시간 관측용 입력 부족: 필요 {required_lots}, 준비 {len(simulator.JOB_SCHEDULE)}")
    case_manifest_path = case_directory / "입력생성조건.json"
    manifest = {
        "status": "실행중", "started_at": current_timestamp(), "basis": "제공 버전 2",
        "mode": "제공 원본 헤드리스 함수" if arguments.without_observation else "읽기 전용 상태 관측 추가",
        "num_oht": arguments.num_oht, "sim_duration": arguments.sim_duration,
        "spawn_interval": arguments.spawn_interval, "seed": arguments.seed,
        "case_directory": str(case_directory), "input_file_sha256": input_hashes,
        "case_generation": json.loads(case_manifest_path.read_text()) if case_manifest_path.exists() else "동봉 입력",
        "provided_file_sha256": source_manifest["provided_file_sha256"],
        "runtime_python": sys.executable, "python_version": platform.python_version(),
        "dependency_versions": {name: importlib.metadata.version(name) for name in ("salabim", "numpy", "scipy", "pygame", "Pillow")},
        "provided_control_settings": {name: getattr(simulator, name) for name in (
            "FOLLOW_CHECK_MODE", "FOLLOW_CENTER_GAP_MM", "MERGE_CONTROL_MODE", "CONGESTION_WEIGHT",
            "ENABLE_RING_CONTROL", "RING_CAP_MARGIN", "ENABLE_MERGE_DEADLOCK_BREAK", "ENABLE_MERGE_OUT_FULL",
            "MERGE_REQUEST_AT_MERGE_ENTRY", "RING_WAIT_OVERRIDE_S", "DISPATCH_INTERVAL")},
        "observation": {"observation_interval_seconds": 1, "snapshot_interval_seconds": 10,
            "before_event_seconds": 300, "after_event_seconds": 600,
            "stable_long_event_snapshot_interval_seconds": 60,
            "snapshot_cadence": "발생 전후와 첫 10분은 10초, 장기 지속은 60초. 최초 포착·악화·회복 시점은 즉시 저장",
            "time_series_storage": "change_only_rows_with_forward_fill",
            "snapshot_path_storage": "대기 차량 전체 경로. 이동 차량은 앞쪽 12개 노드와 최종 목적지",
            "snapshot_purpose": "관측과 지도 복원용 상태 자료. 엔진 재시작용 체크포인트가 아님",
            "risk_hold_seconds": arguments.risk_hold_seconds, "long_wait_seconds": arguments.long_wait_seconds,
            "type_labels_are_hypotheses": True, "automatic_deadlock_confirmation": False},
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
            recorder = SimulationRecorder(simulator, output_directory, arguments.sim_duration,
                risk_hold_seconds=arguments.risk_hold_seconds, long_wait_seconds=arguments.long_wait_seconds)
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
            write_json(output_directory / "실험조건.json", manifest)
            print(json.dumps({name: manifest[name] for name in (
                "status", "actual_simulation_second", "wall_elapsed_seconds", "maximum_resident_memory_bytes")}, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
