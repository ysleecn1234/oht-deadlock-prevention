"""버전2 입력 준비. 기본은 기존 LOT를 보존한 안전 초기 배치 생성."""

import argparse
import json
from pathlib import Path
import random
import shutil

from experiment_support import (
    EXPERIMENT_ROOT, PROVIDED_SIMULATOR_DIRECTORY, COPIED_SIMULATOR_DIRECTORY,
    MAP_FILE_NAMES, PROVIDED_FILE_NAMES, current_timestamp, file_sha256,
    load_simulator, verify_provided_files, write_json,
)


def prepare_copy():
    manifest_path = EXPERIMENT_ROOT / "입력자료" / "원본보존확인.json"
    if manifest_path.exists():
        verify_provided_files()
        return
    COPIED_SIMULATOR_DIRECTORY.mkdir(parents=True, exist_ok=True)
    hashes = {}
    for name in PROVIDED_FILE_NAMES:
        source = PROVIDED_SIMULATOR_DIRECTORY / name
        target = COPIED_SIMULATOR_DIRECTORY / name
        if target.exists() and file_sha256(target) != file_sha256(source):
            raise RuntimeError(f"기존 사본을 덮어쓸 수 없음: {target}")
        if not target.exists():
            shutil.copy2(source, target)
        hashes[name] = file_sha256(source)
    write_json(manifest_path, {"created_at": current_timestamp(), "basis": "제공 버전 2",
                              "provided_file_sha256": hashes})
    common_directory = EXPERIMENT_ROOT / "입력자료" / "지도와설비"
    common_directory.mkdir(parents=True, exist_ok=True)
    for name in MAP_FILE_NAMES:
        target = common_directory / name
        if not target.exists():
            shutil.copy2(COPIED_SIMULATOR_DIRECTORY / name, target)
    for name in ("실행결과", "실험전체비교", "검증결과"):
        (EXPERIMENT_ROOT / name).mkdir(exist_ok=True)


def prepare_case(simulator, case_number, lot_schedule_count):
    case_directory = EXPERIMENT_ROOT / "입력자료" / "시드별입력" / f"시드{case_number:02d}"
    manifest_path = case_directory / "입력생성조건.json"
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if manifest["lot_schedule_count"] < lot_schedule_count:
            raise RuntimeError("이미 고정한 입력 길이가 부족함. 기존 입력 덮어쓰기 금지")
        for name, expected in manifest["input_file_sha256"].items():
            if file_sha256(case_directory / name) != expected:
                raise RuntimeError(f"입력 파일 불일치: {name}")
        print(f"기존 입력 확인: {case_directory.name}", flush=True)
        return
    case_directory.mkdir(parents=True, exist_ok=True)
    for name in MAP_FILE_NAMES:
        target = case_directory / name
        if not target.exists():
            target.symlink_to(EXPERIMENT_ROOT / "입력자료" / "지도와설비" / name)
    starting_position_random_seed = 100 + case_number
    lot_schedule_random_seed = 200 + case_number
    starting_position_random_generator = random.Random(starting_position_random_seed)
    positions = starting_position_random_generator.sample(simulator.NODE_LIST, 1500)
    write_json(case_directory / "oht_park_positions.json", positions)
    lot_random_generator = random.Random(lot_schedule_random_seed)
    schedule_path = case_directory / "oht_lot_schedule.json"
    if schedule_path.exists():
        raise RuntimeError(f"미완료 입력을 자동 덮어쓰지 않음: {schedule_path}")
    with schedule_path.open("x", encoding="utf-8") as handle:
        handle.write("[")
        for lot_number in range(lot_schedule_count):
            if lot_number:
                handle.write(",")
            # 제공 함수의 포트·패턴·긴급도 추출 방식을 그대로 사용.
            entry = simulator._make_lot_entry(lot_random_generator)
            handle.write(json.dumps(entry, ensure_ascii=False, separators=(",", ":")))
        handle.write("]")
    write_json(manifest_path, {
        "basis": "제공 버전 2", "case_number": case_number,
        "created_at": current_timestamp(), "starting_position_random_seed": starting_position_random_seed,
        "lot_schedule_random_seed": lot_schedule_random_seed, "idle_jitter_random_seed": 2,
        "park_position_count": len(positions), "lot_schedule_count": lot_schedule_count,
        "demand_distribution": "제공 기본 분포", "ordered_node_list_sha256":
            __import__("hashlib").sha256(json.dumps(simulator.NODE_LIST).encode()).hexdigest(),
        "input_file_sha256": {name: file_sha256(case_directory / name) for name in (
            *MAP_FILE_NAMES, "oht_park_positions.json", "oht_lot_schedule.json")},
    })
    print(f"입력 생성: {case_directory.name}, 시작 위치 {len(positions)}, 로트 {lot_schedule_count}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case-numbers", default="1,2,3")
    parser.add_argument("--lot-schedule-count", type=int, default=200000)
    # [2026-11-06] 기존 입력은 원본 재현용으로 남긴다. 새 배치는 안전거리
    # 조건을 적용한 별도 폴더에 저장하며 시뮬레이터를 import하지 않는다.
    parser.add_argument("--initial-position-policy", choices=("safe", "original"), default="safe")
    parser.add_argument("--maximum-vehicle-count", type=int, default=1500)
    parser.add_argument("--output-directory", type=Path)
    arguments = parser.parse_args()
    if arguments.initial_position_policy == "safe":
        from safe_initial_positions import prepare_safe_cases
        reports = prepare_safe_cases(
            list(map(int, arguments.case_numbers.split(","))),
            arguments.maximum_vehicle_count, arguments.output_directory,
            arguments.lot_schedule_count)
        for report in reports:
            maximum = report["validations"][-1]
            print(f"안전 초기 배치: 시드{report['case_number']:02d}, "
                  f"{maximum['vehicle_count']}대, "
                  f"최소 간격 {maximum['minimum_center_distance_mm']}mm, "
                  f"{report['directory']}", flush=True)
        return
    if arguments.output_directory is not None or arguments.maximum_vehicle_count != 1500:
        parser.error("--output-directory와 변경된 --maximum-vehicle-count는 safe 방식에서만 사용")
    prepare_copy()
    simulator = load_simulator(COPIED_SIMULATOR_DIRECTORY)
    for case_number in map(int, arguments.case_numbers.split(",")):
        prepare_case(simulator, case_number, arguments.lot_schedule_count)
    verify_provided_files()


if __name__ == "__main__":
    main()
