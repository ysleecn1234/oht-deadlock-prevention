"""검증한 새 관측으로만 24시간 15조건 실행. 원본 제어·입력 불변, 최대 네 개 병행."""

from datetime import datetime
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

from experiment_support import EXPERIMENT_ROOT, current_timestamp, file_sha256, verify_provided_files, write_json
from integrated_deadlock_observer import PROTOCOL_VERSION


COMPARISON_DIRECTORY = EXPERIMENT_ROOT / "실험전체비교"
QUEUE_PATH = COMPARISON_DIRECTORY / "현재실험목록.json"
REDESIGN_PATH = COMPARISON_DIRECTORY / "실험재설계상태.json"


def read_json(path, default=None):
    return json.loads(path.read_text()) if path.exists() else default


def directory_size_bytes(directory):
    return sum(path.stat().st_size for path in directory.rglob("*") if path.is_file() and not path.is_symlink())


def verify_gates(scripts):
    preservation = read_json(EXPERIMENT_ROOT / "검증결과" / "새관측동작보존검증.json")
    detection = read_json(EXPERIMENT_ROOT / "검증결과" / "유형탐지통합검증.json")
    if not preservation or not detection or not preservation["passed"] or not detection["passed"]:
        raise RuntimeError("새 관측 동작 보존·탐지 통합 시험을 모두 통과해야 실행 가능")
    for name, expected in preservation["dependency_sha256"].items():
        if file_sha256(scripts / name) != expected:
            raise RuntimeError("검증 후 실행 코드 변경: " + name)
    if detection["observer_sha256"] != file_sha256(scripts / "integrated_deadlock_observer.py"):
        raise RuntimeError("탐지 통합 시험 후 관측 코드 변경")
    if verify_provided_files()["provided_file_sha256"] != preservation["provided_file_sha256"]:
        raise RuntimeError("검증한 제공 버전 2와 불일치")
    return preservation


def initial_storage_budget(preservation):
    directory = Path(preservation["validation_directory"])
    forecasts = []
    for comparison in preservation["comparisons"]:
        pilot = directory / f"새관측_차량{comparison['num_oht']}_{comparison['sim_duration']}초"
        forecasts.append(directory_size_bytes(pilot) * 86400 / comparison["sim_duration"] * 2)
    return max(2 * 1024**3, int(max(forecasts)))


def collect_results(tasks, batch_directory):
    rows, inspections, candidates = [], [], []
    for task in tasks:
        output = Path(task["output_directory"])
        manifest = read_json(output / "실험조건.json", {})
        progress = read_json(output / "중간집계" / "현재진행상태.json", {})
        signature = read_json(output / "최종집계" / "동작대조서명.json", {})
        detection = read_json(output / "최종집계" / "탐지요약.json", {})
        candidate_state = read_json(output / "중간집계" / "유형후보현황.json", {})
        if progress:
            task["progress"] = progress
        confirmed_count = progress.get("confirmed_structural_deadlock_count", 0)
        rows.append({"experiment_number": task["experiment_number"], "num_oht": task["num_oht"],
            "case_number": task["case_number"], "runtime_seed": task["seed"], "status": task["status"],
            "simulation_second": manifest.get("actual_simulation_second", progress.get("simulation_second")),
            "wall_elapsed_seconds": manifest.get("wall_elapsed_seconds", progress.get("wall_elapsed_seconds")),
            "completed_lots": signature.get("completed_lots", progress.get("completed_lots")),
            "completed_transfers": signature.get("completed_transfers", progress.get("completed_transfers")),
            "confirmed_structural_deadlock_count": confirmed_count,
            "active_candidate_count": progress.get("active_candidate_count"),
            "inspection_status": "시작부터 매초 충분조건 검사" if progress else "미시작",
            "output_directory": str(output), "detection_summary": detection})
        if progress:
            inspections.append({"experiment_number": task["experiment_number"],
                "simulation_second": progress["simulation_second"], "inspected_observation_count": progress["inspected_observation_count"],
                "confirmed_structural_deadlock_count": confirmed_count,
                "refutations": candidate_state.get("refutations", []),
                "limits": "충분조건으로 확정되지 않은 후보 및 매초 관측 사이 현상에 대한 교착 부재 보장 아님"})
            for event in candidate_state.get("active_candidates", []):
                candidates.append({"experiment_number": task["experiment_number"], "output_directory": str(output), **event})
    extract = COMPARISON_DIRECTORY / "결과추출"
    write_json(extract / "새실험요약.json", {"updated_at": current_timestamp(), "batch_directory": str(batch_directory), "experiments": rows})
    complete = all(task["status"] == "정상완료" for task in tasks)
    write_json(extract / "자동추출상태.json", {"updated_at": current_timestamp(), "batch_directory": str(batch_directory),
        "completed_experiment_count": sum(task["status"] == "정상완료" for task in tasks), "total_experiments": len(tasks),
        "status": "전체 실험·매초 검사·결과 추출 완료" if complete else "새 배치 실행 및 자동 집계 중",
        "all_experiments_and_extraction_complete": complete})
    write_json(COMPARISON_DIRECTORY / "확정교착검증현황.json", {"updated_at": current_timestamp(),
        "batch_directory": str(batch_directory), "protocol": PROTOCOL_VERSION, "experiments": inspections})
    write_json(COMPARISON_DIRECTORY / "우선검토후보현황.json", {"updated_at": current_timestamp(),
        "batch_directory": str(batch_directory), "target_types": ["TYPE I", "TYPE II", "TYPE III", "TYPE IV"],
        "candidates": candidates, "note": "미해소 후보 및 정체 징후. 우선순위는 확률이 아니며 구역별 중복 교착 건수 아님"})
    return any(item.get("refutations") for item in inspections)


def main():
    scripts = Path(__file__).resolve().parent
    preservation = verify_gates(scripts)
    estimated_storage = initial_storage_budget(preservation)
    batch = EXPERIMENT_ROOT / "실행결과" / ("유형1~4_시작부터관측_24시간_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    lock = COMPARISON_DIRECTORY / "실행관리잠금.json"
    with lock.open("x", encoding="utf-8") as handle:
        json.dump({"process_id": os.getpid(), "started_at": current_timestamp(), "protocol": PROTOCOL_VERSION}, handle)
    tasks = []
    running = {}
    keep_awake = None
    intentional_shutdown = False
    try:
        batch.mkdir(parents=True)
        (batch / "실행로그").mkdir()
        for case_number in (1, 2, 3):
            for num_oht in (1500, 1250, 1000, 750, 500):
                number = len(tasks) + 1
                tasks.append({"experiment_number": number, "case_number": case_number, "seed": 1,
                    "num_oht": num_oht, "sim_duration": 86400, "spawn_interval": 0.5, "status": "대기중",
                    "output_directory": str(batch / f"실험{number:04d}_차량{num_oht}대_입력시드{case_number:02d}")})
        write_json(batch / "실험배치조건.json", {"protocol": PROTOCOL_VERSION, "basis": "제공 버전 2·제어 변경 없음",
            "sim_duration": 86400, "spawn_interval": 0.5, "maximum_concurrent_simulations": 4,
            "vehicle_counts": [1500, 1250, 1000, 750, 500], "input_case_numbers": [1, 2, 3], "runtime_seed": 1,
            "seed_note": "입력 사례별 시작위치 101~103·LOT 201~203. 실행 난수 seed=1은 제공 기본값으로 고정하여 대조. 입력생성조건의 idle_jitter_random_seed=2는 실제 실행값으로 사용하지 않음",
            "initial_storage_budget_bytes_per_experiment": estimated_storage,
            "source_file_sha256": preservation["provided_file_sha256"],
            "validated_dependency_sha256": preservation["dependency_sha256"],
            "validation_comparisons": preservation["comparisons"], "manager_sha256": file_sha256(Path(__file__))})
        write_json(COMPARISON_DIRECTORY / "후보검토기준.json", {
            "protocol": PROTOCOL_VERSION, "source_material": "OHT Deadlock 대표 시나리오 10가지 1~4번",
            "target_types": ["TYPE I", "TYPE II", "TYPE III", "TYPE IV"],
            "purpose": "교착 탐색과 그 이전 정체 과정 포착", "observation_interval_seconds": 1,
            "before_event_seconds": [300, 120, 60], "temporary_buffer_seconds": 600,
            "cycle_candidate_seconds": 15, "cycle_key_photo_seconds": 120,
            "saturation_warning": "같은 차단 집단의 3대 이상이 60초 이상 정지하며 관련 용량/노드 포화 동반",
            "long_wait_photo_seconds": 600, "confirmed_photo": "충분조건 충족 시 즉시",
            "limits": "관측 지표와 유형 부분 근거는 후보. 충분조건 검사의 완전성 보장이나 실제 팹 발생 주장 아님"})

        def shutdown(signum, frame):
            raise KeyboardInterrupt(f"관리자 중단 신호 {signum}")
        signal.signal(signal.SIGTERM, shutdown)
        if Path("/usr/bin/caffeinate").exists():
            keep_awake = subprocess.Popen(["/usr/bin/caffeinate", "-i", "-w", str(os.getpid())],
                stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        last_storage_check = -60
        while any(task["status"] == "대기중" for task in tasks) or running:
            for number, (process, handle, task) in list(running.items()):
                if process.poll() is not None:
                    handle.close()
                    manifest = read_json(Path(task["output_directory"]) / "실험조건.json", {})
                    successful = process.returncode == 0 and manifest.get("status") == "정상완료" and manifest.get("actual_simulation_second") == 86400
                    task.update(status="정상완료" if successful else "오류종료", ended_at=current_timestamp(), process_exit_code=process.returncode)
                    del running[number]
                    print(f"[{current_timestamp()}] 실험 {number}: {task['status']}", flush=True)
                    if not successful:
                        for pending in tasks:
                            if pending["status"] == "대기중":
                                pending["status"] = "오류확인대기"
            refutation = collect_results(tasks, batch)
            if refutation or (COMPARISON_DIRECTORY / "새배치신규실행중단요청.json").exists():
                for task in tasks:
                    if task["status"] == "대기중":
                        task["status"] = "신규실행보류"
            memory_total = sum(task.get("progress", {}).get("maximum_resident_memory_bytes", 0) for _, _, task in running.values())
            if time.monotonic() - last_storage_check >= 60:
                for _, _, task in running.values():
                    occupied = directory_size_bytes(Path(task["output_directory"]))
                    second = task.get("progress", {}).get("simulation_second", 0)
                    predicted = max(estimated_storage, occupied * 86400 / max(second, 300) * 1.8)
                    task["storage_occupied_bytes"] = occupied
                    task["storage_remaining_budget_bytes"] = int(max(0, predicted - occupied))
                last_storage_check = time.monotonic()
            free = shutil.disk_usage(EXPERIMENT_ROOT).free
            committed = sum(task.get("storage_remaining_budget_bytes", estimated_storage) for _, _, task in running.values())
            while len(running) < 4 and free >= 8 * 1024**3 + committed + estimated_storage and memory_total < 14 * 1024**3:
                task = next((task for task in tasks if task["status"] == "대기중"), None)
                if task is None:
                    break
                verify_gates(scripts)
                log_path = batch / "실행로그" / f"실험{task['experiment_number']:04d}.log"
                handle = log_path.open("x", encoding="utf-8")
                command = [sys.executable, "-u", str(scripts / "run_integrated_deadlock_experiment.py"),
                    "--case-directory", str(EXPERIMENT_ROOT / "입력자료" / "시드별입력" / f"시드{task['case_number']:02d}"),
                    "--output-directory", task["output_directory"], "--num-oht", str(task["num_oht"]),
                    "--sim-duration", "86400", "--spawn-interval", "0.5", "--seed", str(task["seed"])]
                process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=handle, stderr=subprocess.STDOUT)
                task.update(status="실행중", started_at=current_timestamp(), process_id=process.pid, log_path=str(log_path))
                running[task["experiment_number"]] = (process, handle, task)
                committed += estimated_storage
                memory_total += 1024**3
                print(f"[{current_timestamp()}] 시작: {task['experiment_number']}번 / {task['num_oht']}대 / 입력시드{task['case_number']:02d}", flush=True)
            write_json(QUEUE_PATH, {"updated_at": current_timestamp(), "protocol": PROTOCOL_VERSION,
                "batch_directory": str(batch), "maximum_concurrent_simulations": 4, "running_count": len(running),
                "storage_free_bytes": free, "experiments": tasks})
            redesign = read_json(REDESIGN_PATH, {})
            redesign.update(updated_at=current_timestamp(), phase="검증 통과·새 배치 실행 중", intentional_stop=False,
                new_batch_started=True, new_queue_path=str(QUEUE_PATH), new_result_directory=str(batch),
                observation_purpose="교착과 그 이전 정체 과정 포착")
            write_json(REDESIGN_PATH, redesign)
            if not running and any(task["status"] == "대기중" for task in tasks):
                for task in tasks:
                    if task["status"] == "대기중":
                        task["status"] = "저장공간확인대기"
                break
            time.sleep(10)
    except KeyboardInterrupt:
        intentional_shutdown = True
        raise
    finally:
        for process, handle, task in running.values():
            process.terminate()
            try:
                process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            handle.close()
            task.update(status="관리자중단" if intentional_shutdown else "관리자오류로중단", ended_at=current_timestamp())
        if keep_awake is not None:
            keep_awake.terminate()
        collect_results(tasks, batch)
        write_json(QUEUE_PATH, {"updated_at": current_timestamp(), "protocol": PROTOCOL_VERSION,
            "batch_directory": str(batch), "running_count": 0, "maximum_concurrent_simulations": 4,
            "storage_free_bytes": shutil.disk_usage(EXPERIMENT_ROOT).free, "experiments": tasks})
        write_json(batch / "배치종료상태.json", {"ended_at": current_timestamp(), "experiments": tasks})
        lock.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
