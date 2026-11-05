"""사용자 요청으로 멈춘 동일 프로세스만 검증하여 재개한다. 기본 동작은 확인만 수행."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from experiment_support import EXPERIMENT_ROOT, current_timestamp, verify_provided_files, write_json


def inspect_paused_processes(record):
    verified = []
    for identity in record["process_identities"]:
        number = identity["process_id"]
        def field(name):
            return subprocess.check_output(["/bin/ps", "-p", str(number), "-o", name + "="], text=True).strip()
        if field("lstart") != identity["started_at_process"] or field("command") != identity["command"]:
            raise RuntimeError(f"원래 프로세스가 유지되지 않음: {number}. 다른 프로세스에 재개 신호를 보내지 않음")
        status = field("stat")
        if "T" not in status:
            raise RuntimeError(f"일시정지 상태 아님: {number}, {status}")
        verified.append({"process_id": number, "status": status})
    return verified


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--resume", action="store_true", help="사용자가 재개를 요청한 경우에만 지정")
    arguments = parser.parse_args()
    comparison = EXPERIMENT_ROOT / "실험전체비교"
    pause_path = comparison / "실험일시정지상태.json"
    record = json.loads(pause_path.read_text())
    if record["status"] != "일시정지":
        raise RuntimeError("일시정지 기록 상태를 확인해야 함")
    verified = inspect_paused_processes(record)
    verify_provided_files()
    if not arguments.resume:
        print(json.dumps({"can_resume_same_processes": True, "resumed": False, "processes": verified}, ensure_ascii=False))
        return
    queue_path = comparison / "현재실험목록.json"
    queue = json.loads(queue_path.read_text())
    if queue["batch_directory"] != record["batch_directory"]:
        raise RuntimeError("현재 배치와 일시정지 배치 불일치")
    resumed_at = current_timestamp()
    record.update(status="재개", resumed_at=resumed_at,
        pause_duration_seconds=time.time() - record["paused_at_epoch_seconds"])
    record["replacement_keep_awake_process_id"] = subprocess.Popen(
        ["/usr/bin/caffeinate", "-i", "-w", str(record["manager_process_id"])],
        stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True).pid
    write_json(pause_path, record)
    write_json(comparison / "일시정지이력" / ("재개_" + time.strftime("%Y%m%d_%H%M%S") + ".json"), record)
    queue.update(updated_at=resumed_at, execution_paused=False, paused_count=0,
        running_count=len(record["worker_process_ids"]), resumed_at=resumed_at)
    for task in queue["experiments"]:
        if task.get("process_id") in record["worker_process_ids"]:
            task["status"] = "실행중"
    write_json(queue_path, queue)
    state_path = comparison / "실험재설계상태.json"
    state = json.loads(state_path.read_text())
    state.update(updated_at=resumed_at, phase="동일 프로세스 재개·새 배치 실행 중", user_paused=False,
        intentional_stop=False, monitoring_automation_resume_required=True)
    write_json(state_path, state)
    # 관리자를 파일 교체 직전에 멈췄던 경우에도 교체 대상 임시 경로를 보존한다.
    # 이 파일들은 관리자 자신의 다음 원자적 저장에서 자연스럽게 교체된다.
    for path in (queue_path, state_path):
        path.with_name(path.name + ".writing").write_text(path.read_text(), encoding="utf-8")
    for number in record["worker_process_ids"]:
        os.kill(number, signal.SIGCONT)
    os.kill(record["manager_process_id"], signal.SIGCONT)
    print(json.dumps({"resumed_at": resumed_at, "resumed_processes": verified,
        "pause_duration_seconds": record["pause_duration_seconds"],
        "monitoring_note": "완료 확인 자동화 2는 Codex 도구에서 별도로 ACTIVE로 재개해야 함"}, ensure_ascii=False))


if __name__ == "__main__":
    main()
