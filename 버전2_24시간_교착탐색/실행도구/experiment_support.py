"""제공 버전 2의 원본 보존, 입력 확인 및 실험 파일 관리."""

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
from datetime import datetime


EXPERIMENT_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = EXPERIMENT_ROOT.parent
PROVIDED_SIMULATOR_DIRECTORY = PROJECT_ROOT / "D&D_Fab_ver2"
COPIED_SIMULATOR_DIRECTORY = EXPERIMENT_ROOT / "시뮬레이터사본"
MAP_FILE_NAMES = ("agv_graph.json", "equipment.json", "section_regions.json", "track_segments.json")
PROVIDED_FILE_NAMES = ("agv_fab_sim.py", *MAP_FILE_NAMES,
                       "oht_park_positions.json", "oht_lot_schedule.json")


def current_timestamp():
    return datetime.now().astimezone().isoformat(timespec="seconds")


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_json(path, content):
    """중간 쓰기 실패 시 이전 정상 파일을 보존하는 교체 저장."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = path.with_name(path.name + ".writing")
    temporary_path.write_text(json.dumps(content, ensure_ascii=False, indent=2, default=json_default), encoding="utf-8")
    temporary_path.replace(path)


def json_default(value):
    """원본 기록 속 Job 참조를 메모리 주소 대신 의미 있는 고정 속성으로 저장."""
    if hasattr(value, "job_id"):
        return {name: getattr(value, name) for name in type(value).__slots__ if name != "oht"}
    if hasattr(value, "oht_id"):
        return {"oht_id": value.oht_id}
    if isinstance(value, (set, frozenset)):
        return sorted(value)
    if isinstance(value, Path):
        return str(value)
    if hasattr(value, "item"):
        return value.item()
    raise TypeError(f"명시적인 저장 변환이 필요한 자료형: {type(value).__name__}")


def verify_provided_files():
    manifest_path = EXPERIMENT_ROOT / "입력자료" / "원본보존확인.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    for name, expected in manifest["provided_file_sha256"].items():
        directories = (PROVIDED_SIMULATOR_DIRECTORY,) if name == "agv_fab_sim.py" else (PROVIDED_SIMULATOR_DIRECTORY, COPIED_SIMULATOR_DIRECTORY)
        for directory in directories:
            if file_sha256(directory / name) != expected:
                raise RuntimeError(f"제공 원본 또는 사본 불일치: {directory / name}")
    return manifest


def load_simulator(case_directory, occupancy_mode="node", source_directory=None):
    """제공 원본을 확인한 뒤 작업본/대조 원본을 지정 입력 폴더에서 불러옴."""
    verify_provided_files()
    sys.dont_write_bytecode = True
    previous_arguments = sys.argv
    previous_directory = Path.cwd()
    try:
        sys.argv = ["agv_fab_sim.py", "--analyze", "--occupancy-mode", occupancy_mode]
        os.chdir(case_directory)
        specification = importlib.util.spec_from_file_location(
            "provided_version_two_simulator", Path(source_directory or COPIED_SIMULATOR_DIRECTORY) / "agv_fab_sim.py")
        simulator = importlib.util.module_from_spec(specification)
        sys.modules[specification.name] = simulator
        specification.loader.exec_module(simulator)
    finally:
        sys.argv = previous_arguments
        os.chdir(previous_directory)
    if not simulator.HEADLESS:
        raise RuntimeError("헤드리스 실행 설정이 적용되지 않음")
    return simulator


def simulation_signature(simulator):
    """관측 전후 동작 보존 검증용. 원본 통계와 실제 최종 상태 전체를 대조."""
    content = {
        "JOB_STATS": simulator.JOB_STATS,
        "LOT_STATS": simulator.LOT_STATS,
        "STALL_EVENTS": simulator.STALL_EVENTS,
        "RING_STATS": simulator.RING_STATS,
        "MERGE_OCS_STATS": simulator.MERGE_OCS_STATS,
        "REROUTE_COUNTS": simulator.REROUTE_COUNTS,
        "MERGE_HOLDER": simulator.MERGE_HOLDER,
        "MERGE_EXPIRE": simulator.MERGE_EXPIRE,
        "MERGE_RESERVERS": {node: {str(side): [vehicle.oht_id for vehicle in members]
                            for side, members in sides.items()}
                            for node, sides in simulator.MERGE_RESERVERS.items()},
        "NODE_HOLDER": {node: vehicle.oht_id if vehicle is not None else None
                        for node, vehicle in simulator.NODE_HOLDER.items()},
        "NODE_WAITERS": {node: [vehicle.oht_id for vehicle in members]
                         for node, members in simulator.NODE_WAITERS.items() if members},
        "LINK_PENALTY": simulator.LINK_PENALTY.tolist(),
        "_JOB_SCHEDULE_IDX": simulator._JOB_SCHEDULE_IDX,
        "OHT_LIST": [{name: getattr(vehicle, name) for name in (
            "oht_id", "node", "state", "total_distance", "total_busy", "total_stall_time",
            "_stall_next", "_stuck_since", "_seg_start_t", "_cur_speed")}
            | {"job_id": vehicle.job.job_id if vehicle.job is not None else None,
               "next_job_id": vehicle.next_job.job_id if vehicle.next_job is not None else None}
            for vehicle in simulator.OHT_LIST],
    }
    if getattr(simulator, "ENABLE_SEGMENT_OCCUPANCY", False):
        controller = simulator.get_segment_motion()
        content["segment_motion"] = {
            "vehicles": [{"oht_id": vehicle.oht_id, "position": controller.get_position(vehicle),
                "plan": vehicle._motion_plan, "route": controller.routes.get(vehicle, []),
                "exit_speeds": controller.route_exit_speeds.get(vehicle, []),
                "target_port": controller.target_ports.get(vehicle),
                "control_claims": controller.control_claims.get(vehicle, {}),
                "port_departures": [(list(key), value) for key, value in controller.port_departures.get(vehicle, {}).items()],
                "merge_wait_seconds": dict(controller.merge_wait_seconds.get(vehicle, {}))}
                for vehicle in simulator.OHT_LIST],
            "port_claims": [(list(key), vehicle.oht_id) for key, vehicle in sorted(controller.port_claims.items())],
            "merge_waiters": {node: [(vehicle.oht_id, entry) for vehicle, entry in queue.items()]
                              for node, queue in controller.merge_waiters.items() if queue},
            "ring_reservations": {number: sorted(vehicle.oht_id for vehicle in members)
                                  for number, members in controller.ring_reservations.items() if members},
            "merge_statistics": simulator.MERGE_STATS,
            "merge_node_statistics": simulator.MERGE_NODE_STATS,
            "merge_pass_log": simulator.MERGE_PASS_LOG,
        }
    encoded = json.dumps(content, sort_keys=True, separators=(",", ":"), default=json_default).encode()
    return hashlib.sha256(encoded).hexdigest()
