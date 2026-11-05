"""저장 상태를 별도로 읽어 미해소·유형 근거별 우선 검토 후보를 정리한다."""

import argparse
import bisect
import gzip
import json
import os
from pathlib import Path
import signal
import time

from candidate_review import CandidateReviewer, movement_unchanged
from experiment_support import EXPERIMENT_ROOT, current_timestamp, file_sha256, write_json
from manage_experiment_queue import verify_validation_gate
from render_event_maps import draw_map
from snapshot_storage import read_snapshot, snapshot_sha256


TYPE_REFERENCE = "OHT Deadlock 대표 시나리오 10가지 · 1~4번"


def snapshot_second(path):
    return float(path.name.removeprefix("시각_").removesuffix("초.json.gz"))


class ExperimentCandidateReview:
    def __init__(self, directory, graph):
        self.directory = directory
        self.graph = graph
        self.manifest = json.loads((directory / "실험조건.json").read_text())
        regions = json.loads((directory / "구간별시계열" / "지도구간목록.json").read_text())
        self.reviewer = CandidateReviewer(graph, regions)
        self.output = directory / "유형별우선검토후보"
        self.output.mkdir(exist_ok=True)
        self.status_path = self.output / "검토상태.json"
        self.status = json.loads(self.status_path.read_text()) if self.status_path.exists() else {
            "last_snapshot_second": -1, "records": {}, "current_candidates": []}
        self.review_code_sha256 = file_sha256(Path(__file__).with_name("candidate_review.py"))
        if self.status.get("review_code_sha256") != self.review_code_sha256:
            self.status["last_snapshot_second"] = -1

    def capture_candidate(self, candidate, snapshot, source):
        destination = self.output / candidate["candidate_key"]
        certificate_path = destination / "핵심포착근거.json"
        if certificate_path.exists() and all((destination / name).exists() for name in ("전체시뮬레이션.png", "의심구간확대.png")):
            return str(destination)
        destination.mkdir(exist_ok=True)
        state_path = destination / "핵심포착상태.json.gz"
        if certificate_path.exists() and state_path.exists():
            certificate = json.loads(certificate_path.read_text())
            with gzip.open(state_path, "rt", encoding="utf-8") as handle:
                snapshot = json.load(handle)
            image_candidate = certificate["candidate"]
        else:
            image_candidate = candidate
            temporary = state_path.with_name(state_path.name + ".writing")
            with gzip.open(temporary, "wt", encoding="utf-8", compresslevel=6) as handle:
                json.dump(snapshot, handle, ensure_ascii=False, separators=(",", ":"))
            temporary.replace(state_path)
            write_json(certificate_path, {"captured_at": current_timestamp(), "candidate": candidate,
                "source_snapshot_path": str(source), "snapshot_sha256": snapshot_sha256(snapshot),
                "type_reference": TYPE_REFERENCE,
                "note": "후보 근거와 시각을 함께 보존. 교착 확정이나 발생 확률이 아님"})
        caption = "동일 차량의 장기 미해소 대기와 유형별 부분 근거를 보존한 후보. 교착 확정 아님"
        event = {"vehicle_ids": image_candidate["related_vehicle_ids"]}
        title = f"{self.directory.name} / 우선 검토 후보 / 차량 {image_candidate['oht_id']}"
        draw_map(snapshot, self.graph, destination / "전체시뮬레이션.png", title, event, status_caption=caption)
        draw_map(snapshot, self.graph, destination / "의심구간확대.png", title, event, cropped=True, status_caption=caption)
        return str(destination)

    def poll(self, selected_types):
        paths = sorted(path for path in (self.directory / "사건상태공유보관").glob("시각_*.json.gz")
                       if time.time() - path.stat().st_mtime > 2)
        if not paths:
            return []
        source = paths[-1]
        now = snapshot_second(source)
        if now > self.status["last_snapshot_second"]:
            snapshot = read_snapshot(source)
            seconds = [snapshot_second(path) for path in paths]
            earlier_index = bisect.bisect_right(seconds, now - 120) - 1
            previous = read_snapshot(paths[earlier_index]) if earlier_index >= 0 else None
            current = self.reviewer.review(snapshot, previous)
            vehicles = {vehicle["oht_id"]: vehicle for vehicle in snapshot["vehicles"]}
            self.manifest = json.loads((self.directory / "실험조건.json").read_text())
            completed = self.manifest["status"] == "정상완료"
            current_keys = {candidate["candidate_key"] for candidate in current}
            for key, record in self.status["records"].items():
                if key in current_keys or record.get("status") not in ("관측중", "관측종료미해소"):
                    continue
                vehicle = vehicles[record["oht_id"]]
                if not movement_unchanged(record["anchor_state"], vehicle):
                    record.update(status="대표 차량 진행 재개·집단 재검토", movement_resumed_observed_second=now,
                        note="대표 차량의 이동 확인. 기존 집단 전체가 해소됐다는 뜻은 아니며 잔여 대기는 새 후보에서 추적")
                else:
                    record.update(status="대기 조건 변화·추가 검토", condition_changed_observed_second=now)
            captured = sum(bool(record.get("image_directory")) for record in self.status["records"].values())
            for candidate in current:
                key = candidate["candidate_key"]
                record = self.status["records"].get(key, {})
                first_seen = record.get("first_reviewed_second", now)
                image_directory = record.get("image_directory")
                if candidate["priority"] == 1 and image_directory is None and captured < 6:
                    image_directory = self.capture_candidate(candidate, snapshot, source)
                    captured += 1
                candidate.update(experiment_directory=str(self.directory), num_oht=self.manifest["num_oht"],
                    case_number=self.manifest["case_generation"]["case_number"],
                    status="관측종료미해소" if completed else "관측중", right_censored=completed,
                    first_reviewed_second=first_seen, last_reviewed_second=now,
                    anchor_state=vehicles[candidate["oht_id"]], image_directory=image_directory,
                    type_reference=TYPE_REFERENCE)
                self.status["records"][key] = candidate
            self.status.update(last_snapshot_second=now, current_candidates=current, updated_at=current_timestamp(),
                review_code_sha256=self.review_code_sha256, type_reference=TYPE_REFERENCE,
                image_policy="1순위 후보의 핵심 시점 한 쌍. 같은 후보 반복 생성 금지·실험당 최대 6사례. 원자료는 모두 보존")
            write_json(self.status_path, self.status)
        result = []
        for candidate in self.status["current_candidates"]:
            entry = dict(candidate)
            entry["selected_type_candidates"] = [item for item in candidate["ppt_type_evidence"] if selected_types and item["type"] in selected_types]
            entry["type_selection_pending"] = selected_types is None
            entry.pop("anchor_state", None)
            result.append(entry)
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--watch", action="store_true")
    arguments = parser.parse_args()
    def handle_shutdown(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, handle_shutdown)
    verify_validation_gate(Path(__file__).resolve().parent)
    comparison = EXPERIMENT_ROOT / "실험전체비교"
    lock_path = comparison / "후보우선검토잠금.json"
    if lock_path.exists():
        raise RuntimeError("후보 우선 검토가 이미 실행 중임")
    if arguments.watch:
        with lock_path.open("x", encoding="utf-8") as handle:
            json.dump({"process_id": os.getpid(), "started_at": current_timestamp()}, handle)
    graph = json.loads((EXPERIMENT_ROOT / "입력자료" / "지도와설비" / "agv_graph.json").read_text())
    monitors = {}
    try:
        while True:
            configuration = json.loads((comparison / "후보검토기준.json").read_text())
            selected_types = configuration["selected_ppt_types"]
            candidates = []
            for directory in sorted((EXPERIMENT_ROOT / "실행결과").glob("*/*")):
                if not (directory / "구간별시계열" / "지도구간목록.json").exists():
                    continue
                if directory not in monitors:
                    monitors[directory] = ExperimentCandidateReview(directory, graph)
                candidates.extend(monitors[directory].poll(selected_types))
            candidates.sort(key=lambda item: (item["priority"], -item["maximum_waiting_duration_seconds"], -len(item["corroborated_stationary_vehicle_ids"])))
            write_json(comparison / "우선검토후보현황.json", {"updated_at": current_timestamp(),
                "experiment_count": len(monitors), "candidate_count": len(candidates),
                "selected_ppt_types": selected_types, "type_reference": configuration["type_reference"], "top_candidates": candidates[:20],
                "candidates": candidates, "note": "미해소·진행 정지의 검토 우선순위. 교착 확정 또는 발생 확률 순위가 아님",
                "type_mapping_note": "대상은 OHT Deadlock 대표 시나리오 10가지 자료의 TYPE I~IV만. 7유형 자료의 번호와 혼용 금지",
                "excluded_types": configuration["excluded_types"],
                "classification_limit": "관련 부분 근거가 없는 후보는 유형 미분류로 유지하며 네 유형 중 하나로 강제 분류하지 않음"})
            if not arguments.watch:
                print("후보 우선 검토 완료", len(monitors), "실험", len(candidates), "현재 후보", flush=True)
                break
            time.sleep(30)
    except KeyboardInterrupt:
        pass
    finally:
        if arguments.watch:
            lock_path.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
