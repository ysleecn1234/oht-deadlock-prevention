"""버전 2 기본 분포의 24시간 대수·시드 탐색을 최대 네 프로세스로 실행."""

import argparse
import json
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

from experiment_support import (
    EXPERIMENT_ROOT, current_timestamp, file_sha256, verify_provided_files, write_json,
)


def log(message):
    print(f"[{current_timestamp()}] {message}", flush=True)


def verify_validation_gate(script_directory):
    manifest_path = EXPERIMENT_ROOT / "검증결과" / "동작보존검증.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if not manifest.get("passed"):
        raise RuntimeError("동작 보존 검증 미통과")
    for name, key in (("simulation_observer.py", "observer_sha256"),
                      ("run_headless_experiment.py", "runner_sha256"),
                      ("experiment_support.py", "support_sha256")):
        if file_sha256(script_directory / name) != manifest[key]:
            raise RuntimeError(f"검증 이후 코드 변경: {name}. 동작 보존 재검증 필요")
    if verify_provided_files()["provided_file_sha256"] != manifest["provided_file_sha256"]:
        raise RuntimeError("검증한 원본과 불일치")


def directory_size_bytes(directory):
    return sum(path.stat().st_size for path in directory.rglob("*") if path.is_file() and not path.is_symlink())


def initial_storage_budget():
    validation = json.loads((EXPERIMENT_ROOT / "검증결과" / "동작보존검증.json").read_text())
    directory = Path(validation["validation_directory"])
    projections = []
    for comparison in validation["comparisons"]:
        observed = directory / f"관측추가_차량{comparison['num_oht']}_{comparison['sim_duration']}초"
        time_series_bytes = (observed / "구간별시계열" / "구간관측.csv.gz").stat().st_size
        snapshot_files = list((observed / "사건상태공유보관").glob("*.gz"))
        average_snapshot_bytes = sum(path.stat().st_size for path in snapshot_files) / max(1, len(snapshot_files))
        # 초기 위험 증가로 촘촘한 캡처가 몰리는 구간을 하루 전체로 단순 외삽하지 않음.
        # 장기 60초 캡처와 전후 상세·추가 사건 2,000프레임을 예산에 포함.
        projected_snapshots = 86400 / 60 + 2000
        projections.append((time_series_bytes * 86400 / comparison["sim_duration"] +
                            average_snapshot_bytes * projected_snapshots) * 1.5 + 128 * 1024**2)
    return max(512 * 1024**2, int(max(projections)))


def prepare_resumed_batch(previous_queue):
    """정상 완료 결과를 유지하고 한 번도 시작하지 않은 저장공간 대기 조건만 재개."""
    if previous_queue.get("running_count") or any(task["status"] == "실행중" for task in previous_queue["experiments"]):
        raise RuntimeError("실행 중인 실험이 있어 관리자 중복 재개 금지")
    batch_directory = Path(previous_queue["batch_directory"]).resolve()
    if not batch_directory.is_relative_to((EXPERIMENT_ROOT / "실행결과").resolve()):
        raise RuntimeError("현재 프로젝트의 실험 배치가 아님")
    tasks = json.loads(json.dumps(previous_queue["experiments"]))
    if len(tasks) != 15 or len({task["experiment_number"] for task in tasks}) != 15:
        raise RuntimeError("기존 15개 실험 조건 목록 불일치")
    for task in tasks:
        directory = Path(task["output_directory"]).resolve()
        if directory.parent != batch_directory or task["sim_duration"] != 86400 or task["spawn_interval"] != 0.5:
            raise RuntimeError("기존 실험 위치·24시간·투입 0.5 조건 불일치")
        if task["status"] == "정상완료":
            manifest = json.loads((directory / "실험조건.json").read_text())
            if manifest["status"] != "정상완료" or manifest["actual_simulation_second"] != 86400:
                raise RuntimeError("완료 상태와 실제 결과 불일치")
        elif task["status"] in ("저장공간확인대기", "대기중"):
            log_path = batch_directory / "실행로그" / f"실험{task['experiment_number']:04d}_실행.log"
            if directory.exists() or log_path.exists() or task.get("started_at") or task.get("process_id"):
                raise RuntimeError("이미 시작한 흔적이 있는 실험은 자동 재실행하지 않음")
            task["status"] = "대기중"
        else:
            raise RuntimeError(f"별도 확인이 필요한 상태: {task['status']}")
    return batch_directory, tasks


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--maximum-concurrent-simulations", type=int, default=4)
    parser.add_argument("--resume-queue", action="store_true")
    arguments = parser.parse_args()
    if not 1 <= arguments.maximum_concurrent_simulations <= 4:
        raise ValueError("현재 컴퓨터의 승인한 동시 실행 상한은 4개")
    script_directory = Path(__file__).resolve().parent
    verify_validation_gate(script_directory)
    estimated_experiment_storage_bytes = initial_storage_budget()
    previous_queue = None
    if arguments.resume_queue:
        previous_queue = json.loads((EXPERIMENT_ROOT / "실험전체비교" / "현재실험목록.json").read_text())
        batch_directory, tasks = prepare_resumed_batch(previous_queue)
    else:
        batch_timestamp = __import__("datetime").datetime.now().strftime("%Y%m%d_%H%M%S")
        batch_directory = EXPERIMENT_ROOT / "실행결과" / f"기본수요_24시간_{batch_timestamp}"
        tasks = []
        for case_number in (1, 2, 3):
            for num_oht in (1500, 1250, 1000, 750, 500):
                tasks.append({"experiment_number": len(tasks) + 1, "case_number": case_number,
                    "num_oht": num_oht, "sim_duration": 86400, "spawn_interval": 0.5,
                    "status": "대기중", "output_directory": str(batch_directory /
                        f"실험{len(tasks) + 1:04d}_차량{num_oht}대_시드{case_number:02d}")})
    lock_path = EXPERIMENT_ROOT / "실험전체비교" / "실행관리잠금.json"
    with lock_path.open("x", encoding="utf-8") as handle:
        import os
        json.dump({"process_id": os.getpid(), "started_at": current_timestamp()}, handle)
    if arguments.resume_queue:
        history_directory = EXPERIMENT_ROOT / "실험전체비교" / "실행관리재개이력" / time.strftime("재개_%Y%m%d_%H%M%S")
        write_json(history_directory / "재개전실험목록.json", previous_queue)
        write_json(history_directory / "재개정보.json", {"resumed_at": current_timestamp(),
            "process_id": os.getpid(), "batch_directory": str(batch_directory),
            "preserved_completed_experiment_numbers": [task["experiment_number"] for task in tasks if task["status"] == "정상완료"],
            "resumed_pending_experiment_numbers": [task["experiment_number"] for task in tasks if task["status"] == "대기중"],
            "storage_free_bytes": shutil.disk_usage(EXPERIMENT_ROOT).free,
            "maximum_concurrent_simulations": arguments.maximum_concurrent_simulations,
            "note": "시작하지 않은 대기 실험만 재개. 완료 결과와 기존 배치 조건 보존"})
        log("저장공간 확인 후 기존 배치 재개: 완료 결과 보존, 미시작 실험만 실행")
    else:
        batch_directory.mkdir(parents=True)
        (batch_directory / "실행로그").mkdir()
    def handle_shutdown(signum, frame):
        raise KeyboardInterrupt(f"실행 관리자 중단 신호 {signum}")
    signal.signal(signal.SIGTERM, handle_shutdown)
    if not arguments.resume_queue:
        write_json(batch_directory / "실험배치조건.json", {
        "basis": "제공 버전 2", "demand_distribution": "제공 기본 분포",
        "CONGESTION_WEIGHT": 1.0, "maximum_concurrent_simulations": arguments.maximum_concurrent_simulations,
        "initial_storage_budget_bytes_per_experiment": estimated_experiment_storage_bytes,
        "sim_duration": 86400, "spawn_interval": 0.5, "task_count": len(tasks),
            "next_stage": "이 배치 결과로 집중 수요 대상 구간과 대수 2개 선정. 미검증 집중 입력은 자동 실행하지 않음"})
    running = {}
    last_storage_measurement = -60.0
    keep_awake_process = None
    if Path("/usr/bin/caffeinate").exists():
        import os
        keep_awake_process = subprocess.Popen(["/usr/bin/caffeinate", "-i", "-w", str(os.getpid())],
                                               stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        while any(task["status"] == "대기중" for task in tasks) or running:
            if (EXPERIMENT_ROOT / "실험전체비교" / "신규실행중단요청.json").exists():
                for task in tasks:
                    if task["status"] == "대기중":
                        task["status"] = "신규실행보류"
            for number, item in list(running.items()):
                process, handle, task = item
                per_experiment_log = Path(task["output_directory"]) / "실행로그" / "실행.log"
                if per_experiment_log.parent.exists() and not per_experiment_log.exists():
                    per_experiment_log.symlink_to(task["log_path"])
                if process.poll() is not None:
                    handle.close()
                    task["process_exit_code"] = process.returncode
                    task["status"] = "정상완료" if process.returncode == 0 else "오류종료"
                    task["ended_at"] = current_timestamp()
                    del running[number]
                    log(f"{task['experiment_number']}번 {task['status']}")
                    if process.returncode != 0:
                        # 같은 원인의 실패를 15개 조건에 전파하지 않음.
                        for pending in tasks:
                            if pending["status"] == "대기중":
                                pending["status"] = "오류확인대기"
            observed_memory_total = 0
            refresh_storage = time.monotonic() - last_storage_measurement >= 60
            committed_storage_bytes = 0
            for process, handle, task in running.values():
                progress_path = Path(task["output_directory"]) / "중간집계" / "현재진행상태.json"
                if progress_path.exists():
                    progress = json.loads(progress_path.read_text())
                    task["progress"] = progress
                    observed_memory_total += progress["maximum_resident_memory_bytes"]
                    if refresh_storage:
                        occupied_bytes = directory_size_bytes(Path(task["output_directory"]))
                        predicted_total = max(estimated_experiment_storage_bytes,
                            occupied_bytes * 86400 / max(progress["simulation_second"], 300) * 1.5)
                        task["storage_occupied_bytes"] = occupied_bytes
                        task["storage_remaining_budget_bytes"] = max(0, int(predicted_total - occupied_bytes))
                committed_storage_bytes += task.get("storage_remaining_budget_bytes", estimated_experiment_storage_bytes)
            if refresh_storage:
                last_storage_measurement = time.monotonic()
            free_storage = shutil.disk_usage(EXPERIMENT_ROOT).free
            while (len(running) < arguments.maximum_concurrent_simulations
                   and free_storage >= 8 * 1024**3 + committed_storage_bytes + estimated_experiment_storage_bytes
                   and observed_memory_total < 14 * 1024**3):
                task = next((task for task in tasks if task["status"] == "대기중"), None)
                if task is None:
                    break
                output_directory = Path(task["output_directory"])
                log_path = batch_directory / "실행로그" / f"실험{task['experiment_number']:04d}_실행.log"
                handle = log_path.open("x", encoding="utf-8")
                case_directory = EXPERIMENT_ROOT / "입력자료" / "시드별입력" / f"시드{task['case_number']:02d}"
                command = [sys.executable, "-u", str(script_directory / "run_headless_experiment.py"),
                    "--case-directory", str(case_directory), "--output-directory", str(output_directory),
                    "--num-oht", str(task["num_oht"]), "--sim-duration", "86400", "--spawn-interval", "0.5"]
                process = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
                task.update(status="실행중", started_at=current_timestamp(), process_id=process.pid,
                            log_path=str(log_path))
                running[task["experiment_number"]] = (process, handle, task)
                committed_storage_bytes += estimated_experiment_storage_bytes
                log(f"시작: {task['experiment_number']}번 / 차량 {task['num_oht']}대 / 시드 {task['case_number']:02d} / 24시간")
            write_json(EXPERIMENT_ROOT / "실험전체비교" / "현재실험목록.json", {
                "updated_at": current_timestamp(), "batch_directory": str(batch_directory),
                "maximum_concurrent_simulations": arguments.maximum_concurrent_simulations,
                "running_count": len(running), "storage_free_bytes": free_storage, "experiments": tasks})
            # 포착 상태를 지도 이미지로 후처리. 제어 프로세스 안에서 렌더링하지 않음.
            for task in tasks:
                if task["status"] not in ("실행중", "정상완료"):
                    continue
                output_directory = Path(task["output_directory"])
                if not (output_directory / "사건별기록").exists():
                    continue
                with (output_directory / "실행로그" / "이미지생성.log").open("a", encoding="utf-8") as handle:
                    try:
                        result = subprocess.run([sys.executable, str(script_directory / "render_event_maps.py"),
                            "--output-directory", str(output_directory), "--maximum-events", "2"],
                            stdout=handle, stderr=subprocess.STDOUT, timeout=60)
                    except subprocess.TimeoutExpired:
                        task["image_rendering_requires_attention"] = True
                        log(f"{task['experiment_number']}번 이미지 생성 지연. 시뮬레이션은 계속 실행")
                        continue
                    if result.returncode:
                        task["image_rendering_requires_attention"] = True
            if not running and free_storage < 8 * 1024**3 + estimated_experiment_storage_bytes:
                for task in tasks:
                    if task["status"] == "대기중":
                        task["status"] = "저장공간확인대기"
                break
            time.sleep(10)
    finally:
        if keep_awake_process is not None:
            keep_awake_process.terminate()
        for process, handle, task in running.values():
            process.terminate()
            try:
                process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            handle.close()
            task.update(status="관리자중단", ended_at=current_timestamp(), process_exit_code=process.returncode)
        lock_path.unlink(missing_ok=True)
        write_json(EXPERIMENT_ROOT / "실험전체비교" / "현재실험목록.json", {
            "updated_at": current_timestamp(), "batch_directory": str(batch_directory),
            "maximum_concurrent_simulations": arguments.maximum_concurrent_simulations,
            "running_count": 0, "storage_free_bytes": shutil.disk_usage(EXPERIMENT_ROOT).free,
            "experiments": tasks})
        write_json(batch_directory / "배치종료상태.json", {"ended_at": current_timestamp(), "experiments": tasks})


if __name__ == "__main__":
    main()
