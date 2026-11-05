"""실행 결과를 읽기만 하여 완료·진행 요약과 사건·위험 구간 목록을 자동 추출.

제공 시뮬레이터와 관측기를 불러오거나 수정하지 않는다. 압축 원자료는
복제하지 않고 경로로 연결하며, 관측 후보를 교착 확정으로 바꾸지 않는다.
"""

import argparse
from collections import Counter
from datetime import datetime
import json
import os
from pathlib import Path
import time

from experiment_support import EXPERIMENT_ROOT, current_timestamp, verify_provided_files, write_json


RESULT_DIRECTORY = EXPERIMENT_ROOT / "실험전체비교" / "결과추출"


def read_json_if_present(path, default=None):
    return json.loads(path.read_text(encoding="utf-8")) if path.exists() else default


def extract_results():
    verify_provided_files()
    queue_path = EXPERIMENT_ROOT / "실험전체비교" / "현재실험목록.json"
    queue = read_json_if_present(queue_path, {"experiments": [], "running_count": 0})
    queue_tasks = {task["output_directory"]: task for task in queue["experiments"]}
    summaries, events, region_rankings, attention = [], [], [], []
    collected_directories = set()
    for manifest_path in sorted((EXPERIMENT_ROOT / "실행결과").glob("*/*/실험조건.json")):
        directory = manifest_path.parent
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        collected_directories.add(str(directory))
        task = queue_tasks.get(str(directory), {})
        progress = read_json_if_present(directory / "중간집계" / "현재진행상태.json", {})
        completed = manifest["status"] == "정상완료"
        final_counts = read_json_if_present(directory / "최종집계" / "동작대조서명.json", {}) if completed else {}
        simulation_second = final_counts.get("simulation_second", progress.get("simulation_second", 0))
        completed_transfers = final_counts.get("completed_transfers", progress.get("completed_transfers", 0))
        completed_lots = final_counts.get("completed_lots", progress.get("completed_lots", 0))
        injected_lots = final_counts.get("injected_lots", progress.get("injected_lots", 0))
        event_paths = sorted((directory / "사건별기록").glob("*/사건정보.json"))
        case_number = manifest.get("case_generation", {})
        case_number = case_number.get("case_number") if isinstance(case_number, dict) else None
        summary = {
            "experiment_directory": str(directory), "batch_name": directory.parent.name,
            "experiment_number": task.get("experiment_number"), "case_number": case_number,
            "num_oht": manifest["num_oht"], "spawn_interval": manifest["spawn_interval"],
            "sim_duration": manifest["sim_duration"], "simulation_second": simulation_second,
            "status": manifest["status"], "completed": completed,
            "injected_lots": injected_lots, "completed_lots": completed_lots,
            "unfinished_lots": injected_lots - completed_lots,
            "completed_transfers": completed_transfers,
            "completed_transfers_per_hour": completed_transfers * 3600 / simulation_second if simulation_second else None,
            "pending_jobs": progress.get("pending_jobs"),
            "event_count": len(event_paths), "confirmed_deadlock_count": None,
            "classification_note": "위험·포화·순환 대기 후보의 관측 목록. 교착 확정 수는 별도 구조 검증 전 산정하지 않음",
            "provided_control_settings": manifest["provided_control_settings"],
            "wall_elapsed_seconds": manifest.get("wall_elapsed_seconds", progress.get("wall_elapsed_seconds")),
            "started_at": manifest["started_at"], "ended_at": manifest.get("ended_at"),
            "maximum_resident_memory_bytes": manifest.get("maximum_resident_memory_bytes", progress.get("maximum_resident_memory_bytes")),
        }
        if not completed and progress.get("updated_at"):
            age_seconds = (datetime.now().astimezone() - datetime.fromisoformat(progress["updated_at"])).total_seconds()
            summary["seconds_since_progress_update"] = round(age_seconds)
            if manifest["status"] == "실행중" and age_seconds > 1800:
                attention.append({"experiment_directory": str(directory), "reason": "진행 기록 30분 이상 미갱신",
                                  "note": "잠자기·계산 지연·오류를 확인할 것. 자동으로 교착이나 실패로 판정하지 않음"})
        if manifest["status"] not in ("실행중", "정상완료"):
            attention.append({"experiment_directory": str(directory), "reason": manifest["status"], "error": manifest.get("error")})
        summaries.append(summary)
        for event_path in event_paths:
            event = json.loads(event_path.read_text(encoding="utf-8"))
            event_directory = event_path.parent
            events.append({
                "experiment_directory": str(directory), "num_oht": manifest["num_oht"], "case_number": case_number,
                "event_number": event["event_number"], "label": event["label"], "status": event["status"],
                "first_detection_second": event["first_detection_second"],
                "last_observed_second": event["last_observed_second"],
                "observed_condition_duration_lower_bound_seconds": max(0, event["last_observed_second"] - event["first_detection_second"]),
                "right_censored": event.get("right_censored", False),
                "region_names": event["region_names"], "type_hypotheses": event["type_hypotheses"],
                "verdict": event["verdict"], "event_directory": str(event_directory),
                "images": [str(path) for path in sorted(event_directory.glob("*지도/*.png"))],
                "state_file_index": str(event_directory / "발생전후상태" / "상태파일목록.json"),
            })
        final_regions = read_json_if_present(directory / "최종집계" / "구간별누적대기.json")
        if completed and final_regions:
            for region_name, waiting_seconds in final_regions["region_waiting_vehicle_seconds"].items():
                region_rankings.append({"experiment_directory": str(directory), "num_oht": manifest["num_oht"],
                    "case_number": case_number, "region_name": region_name,
                    "waiting_vehicle_seconds": waiting_seconds,
                    "mean_waiting_vehicle_count": waiting_seconds / simulation_second if simulation_second else None})
        if completed:
            write_json(directory / "최종집계" / "추출결과요약.json", summary)
    for directory_name, task in queue_tasks.items():
        if directory_name not in collected_directories:
            summaries.append({key: task.get(key) for key in (
                "experiment_number", "case_number", "num_oht", "sim_duration", "spawn_interval", "status", "output_directory")})
    completed_count = sum(bool(summary.get("completed")) for summary in summaries)
    type_counts = Counter(hypothesis["type"] for event in events for hypothesis in event["type_hypotheses"])
    write_json(RESULT_DIRECTORY / "실험별요약.json", {"updated_at": current_timestamp(),
        "completed_count": completed_count, "running_count": queue["running_count"], "experiments": summaries})
    write_json(RESULT_DIRECTORY / "사건별요약.json", {"updated_at": current_timestamp(), "events": events})
    write_json(RESULT_DIRECTORY / "유형별후보집계.json", {
        "type_hypothesis_counts": dict(type_counts), "event_count": len(events),
        "note": "하나의 사건에 여러 유형 후보가 있을 수 있어 유형 합계에 중복 가능. 확정 교착 또는 예측 정확도가 아님"})
    write_json(RESULT_DIRECTORY / "완료실험_위험구간순위.json", {
        "metric": "관측 시간으로 나눈 대기 차량 누적 초. 구역이 겹칠 수 있어 전체 합산 금지",
        "regions": sorted(region_rankings, key=lambda row: row["mean_waiting_vehicle_count"] or 0, reverse=True)})
    write_json(RESULT_DIRECTORY / "확인필요사항.json", {"updated_at": current_timestamp(), "items": attention})
    return {"completed_count": completed_count, "running_count": queue["running_count"],
            "event_count": len(events), "attention_count": len(attention),
            "pending_count": sum(task["status"] == "대기중" for task in queue["experiments"])}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--watch", action="store_true")
    arguments = parser.parse_args()
    lock_path = RESULT_DIRECTORY / "자동추출잠금.json"
    if arguments.watch:
        RESULT_DIRECTORY.mkdir(parents=True, exist_ok=True)
        with lock_path.open("x", encoding="utf-8") as handle:
            json.dump({"process_id": os.getpid(), "started_at": current_timestamp()}, handle)
    previous_counts = None
    try:
        while True:
            counts = extract_results()
            write_json(RESULT_DIRECTORY / "자동추출상태.json", {"updated_at": current_timestamp(), **counts})
            if counts != previous_counts:
                print(json.dumps({"updated_at": current_timestamp(), **counts}, ensure_ascii=False), flush=True)
                previous_counts = counts
            if not arguments.watch or not counts["running_count"] and not counts["pending_count"]:
                break
            time.sleep(60)
    finally:
        if arguments.watch:
            lock_path.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
