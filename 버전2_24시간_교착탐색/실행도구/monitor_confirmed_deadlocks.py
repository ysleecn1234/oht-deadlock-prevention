"""실행 중인 시뮬레이터에 접속하지 않고 저장 상태에서 교착 증거와 이미지를 생성."""

import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import time

from deadlock_proof import DeadlockProofInspector, SUPPORTED_SIMULATOR_SHA256
from experiment_support import EXPERIMENT_ROOT, current_timestamp, file_sha256, verify_provided_files, write_json
from render_event_maps import draw_map
from snapshot_storage import read_snapshot, snapshot_sha256


def check_proof_contract(directory):
    original = verify_provided_files()
    if original["provided_file_sha256"]["agv_fab_sim.py"] != SUPPORTED_SIMULATOR_SHA256:
        raise RuntimeError("검토하지 않은 시뮬레이터 버전: 자동 확정 금지")
    manifest = json.loads((directory / "실험조건.json").read_text())
    settings = manifest["provided_control_settings"]
    if settings["FOLLOW_CHECK_MODE"] != "DIST" or settings["FOLLOW_CENTER_GAP_MM"] != 1150:
        raise RuntimeError("현재 충분조건이 검증된 차간거리 판정과 불일치")
    validation = json.loads((EXPERIMENT_ROOT / "검증결과" / "동작보존검증.json").read_text())
    for name, key in (("run_headless_experiment.py", "runner_sha256"),
                      ("simulation_observer.py", "observer_sha256"), ("experiment_support.py", "support_sha256")):
        if file_sha256(EXPERIMENT_ROOT / "실행도구" / name) != validation[key]:
            raise RuntimeError("검증된 관측 실행과 불일치: 자동 확정 금지")
    graph_path = EXPERIMENT_ROOT / "입력자료" / "지도와설비" / "agv_graph.json"
    if file_sha256(graph_path) != manifest["input_file_sha256"]["agv_graph.json"]:
        raise RuntimeError("실행 지도와 검사 지도 불일치")
    return manifest


class ExperimentMonitor:
    def __init__(self, directory, graph):
        self.directory = directory
        self.graph = graph
        self.manifest = check_proof_contract(directory)
        regions = json.loads((directory / "구간별시계열" / "지도구간목록.json").read_text())
        self.live_inspector = DeadlockProofInspector(graph, regions)
        self.history_inspector = DeadlockProofInspector(graph, regions)
        self.output = directory / "확정교착기록"
        self.output.mkdir(exist_ok=True)
        self.status_path = self.output / "검사상태.json"
        status = json.loads(self.status_path.read_text()) if self.status_path.exists() else {}
        self.processed = set(status.get("processed_snapshot_files", []))
        self.confirmed = status.get("confirmed_records", {})
        for certificate in self.output.glob("확정*/판정근거.json"):
            record = json.loads(certificate.read_text())
            self.confirmed[self.proof_identity(record)] = record
        self.last_live_second = status.get("last_live_snapshot_second", -1)
        self.disabled = status.get("confirmation_disabled", False)
        self.latest_snapshot = None

    @staticmethod
    def proof_identity(proof):
        identity = json.dumps(sorted((vehicle["oht_id"], vehicle["node"]) for vehicle in proof["vehicle_states"]))
        return hashlib.sha256(identity.encode()).hexdigest()[:16]

    @staticmethod
    def contradicting_vehicle(proof, snapshot):
        if snapshot is None or snapshot["simulation_second"] <= proof["simulation_second"]:
            return None
        vehicles = {vehicle["oht_id"]: vehicle for vehicle in snapshot["vehicles"]}
        for previous in proof["vehicle_states"]:
            current = vehicles.get(previous["oht_id"])
            if current is None or any(current[name] != previous[name] for name in ("node", "total_distance")):
                return {"previous": previous, "current": current}
        return None

    def check_existing_proofs(self, snapshot):
        for record in self.confirmed.values():
            if record.get("withdrawn") or snapshot["simulation_second"] <= record["simulation_second"]:
                continue
            counterexample = self.contradicting_vehicle(record, snapshot)
            if counterexample is not None:
                record["withdrawn"] = True
                record["withdrawal_reason"] = "확정 이후 이동 관측. 충분조건 또는 외부 개입 재검토 필요"
                self.disabled = True
                write_json(self.output / "판정반례.json", {"record": record,
                    "counterexample_second": snapshot["simulation_second"], "vehicle_state": counterexample})
                write_json(Path(record["record_directory"]) / "판정철회.json", record)
                write_json(Path(record["record_directory"]) / "판정근거.json", record)
                print("판정 반례 발생: 자동 확정을 중단하고 시뮬레이션은 유지", flush=True)

    def save_confirmation(self, snapshot, proof, path, historical):
        identity = self.proof_identity(proof)
        if identity in self.confirmed:
            previous = self.confirmed[identity]
            previous["last_verified_second"] = max(previous.get("last_verified_second", 0), snapshot["simulation_second"])
            if previous.get("images_generated") or previous.get("withdrawn"):
                return
            record_directory = Path(previous["record_directory"])
            proof = previous
            with gzip.open(record_directory / "확정시점상태.json.gz", "rt", encoding="utf-8") as handle:
                snapshot = json.load(handle)
        else:
            record_directory = self.output / f"확정{len(self.confirmed) + 1:04d}_시뮬레이션{snapshot['simulation_second']:010.1f}초"
            record_directory.mkdir(parents=True, exist_ok=True)
        proof.update(confirmed_at=current_timestamp(), historical_reinspection=historical,
            source_snapshot_path=str(path), snapshot_sha256=snapshot_sha256(snapshot),
            record_directory=str(record_directory), withdrawn=False,
            source_code_sha256=SUPPORTED_SIMULATOR_SHA256,
            confirmation_limit="지원하는 충분조건으로 입증된 사건만 확정. 미확정은 교착 없음의 증거가 아님")
        temporary_state = record_directory / "확정시점상태.json.gz.writing"
        with gzip.open(temporary_state, "wt", encoding="utf-8", compresslevel=6) as handle:
            json.dump(snapshot, handle, ensure_ascii=False, separators=(",", ":"))
        temporary_state.replace(record_directory / "확정시점상태.json.gz")
        write_json(record_directory / "판정근거.json", proof)
        self.confirmed[identity] = proof
        plotting_snapshot = {**snapshot, "wait_dependencies": [{"oht_id": edge["oht_id"],
            "blocking_vehicle_ids": [edge["blocker"]], "cause": edge["cause"]}
            for edge in proof["physical_wait_edges"]]}
        event = {"vehicle_ids": proof["vehicle_ids"]}
        title = f"{self.directory.name} / 구조적 교착 충분조건 확인"
        caption = "저장 상태의 닫힌 진행 불능 구조와 명시된 버전 2 운용 범위에 대한 충분조건 검증"
        draw_map(plotting_snapshot, self.graph, record_directory / "전체시뮬레이션.png", title, event,
                 status_caption=caption)
        draw_map(plotting_snapshot, self.graph, record_directory / "교착구간확대.png", title, event, cropped=True,
                 status_caption=caption)
        proof["images_generated"] = True
        write_json(record_directory / "판정근거.json", proof)
        self.confirmed[identity] = proof
        print(self.directory.name, "구조적 교착 확인", snapshot["simulation_second"], proof["vehicle_ids"], flush=True)

    def inspect_path(self, path, historical):
        snapshot = read_snapshot(path)
        if not historical:
            self.check_existing_proofs(snapshot)
            self.latest_snapshot = snapshot
        inspector = self.history_inspector if historical else self.live_inspector
        proofs = inspector.inspect(snapshot)
        if not self.disabled:
            for proof in proofs:
                counterexample = self.contradicting_vehicle(proof, self.latest_snapshot)
                if historical and counterexample is not None:
                    self.disabled = True
                    write_json(self.output / "확정전반례.json", {"proof": proof,
                        "counterexample_second": self.latest_snapshot["simulation_second"],
                        "vehicle_state": counterexample,
                        "note": "이후 이동 기록과 모순되어 확정하지 않음. 자동 확정만 중단, 실험 유지"})
                    print(self.directory.name, "충분조건 반례: 확정 전 검토 필요", flush=True)
                    break
                self.save_confirmation(snapshot, proof, path, historical)
        self.processed.add(path.name)
        if not historical:
            self.last_live_second = snapshot["simulation_second"]

    def poll(self, backfill_count):
        paths = sorted(path for path in (self.directory / "사건상태공유보관").glob("시각_*.json.gz")
                       if time.time() - path.stat().st_mtime > 2)
        if not paths:
            return
        newest = paths[-1]
        newest_second = float(newest.name.removeprefix("시각_").removesuffix("초.json.gz"))
        if newest_second > self.last_live_second or self.latest_snapshot is None:
            self.inspect_path(newest, historical=False)
        examined = 0
        for path in paths:
            if path.name in self.processed:
                continue
            self.inspect_path(path, historical=True)
            examined += 1
            if examined >= backfill_count:
                break
        write_json(self.status_path, {"updated_at": current_timestamp(),
            "last_live_snapshot_second": self.last_live_second,
            "processed_snapshot_count": len(self.processed), "available_snapshot_count": len(paths),
            "processed_snapshot_files": sorted(self.processed), "confirmed_records": self.confirmed,
            "confirmation_disabled": self.disabled,
            "supported_sufficient_conditions": ["모든 순방향 이동이 닫힌 점유 집단에 차단됨",
                "재할당 불가 운반·비분기·고정 경로의 닫힌 물리적 대기"],
            "limitations": ["분기·유휴·작업 재배정·용량 자원만으로 된 대기는 충분조건 미충족 시 미확정",
                "네 유형의 분류 증거가 부족하면 교착 확정과 유형 미분류를 구분",
                "전체 상태 공간의 교착 부재를 증명하는 도구가 아님"]})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--watch", action="store_true")
    parser.add_argument("--backfill-count", type=int, default=3)
    arguments = parser.parse_args()
    lock_path = EXPERIMENT_ROOT / "실험전체비교" / "확정검증잠금.json"
    if arguments.watch:
        with lock_path.open("x", encoding="utf-8") as handle:
            json.dump({"process_id": os.getpid(), "started_at": current_timestamp()}, handle)
    graph = json.loads((EXPERIMENT_ROOT / "입력자료" / "지도와설비" / "agv_graph.json").read_text())
    monitors = {}
    try:
        while True:
            for directory in sorted((EXPERIMENT_ROOT / "실행결과").glob("*/*")):
                if not (directory / "구간별시계열" / "지도구간목록.json").exists():
                    continue
                if directory not in monitors:
                    monitors[directory] = ExperimentMonitor(directory, graph)
                monitors[directory].poll(arguments.backfill_count)
            confirmed = [record for monitor in monitors.values() for record in monitor.confirmed.values()]
            write_json(EXPERIMENT_ROOT / "실험전체비교" / "확정교착검증현황.json", {
                "updated_at": current_timestamp(), "experiment_count": len(monitors),
                "confirmed_count": sum(not record.get("withdrawn") for record in confirmed),
                "withdrawn_count": sum(bool(record.get("withdrawn")) for record in confirmed),
                "confirmation_disabled": any(monitor.disabled for monitor in monitors.values()),
                "records": confirmed, "note": "현재까지 검사한 저장 상태의 충분조건 결과. 미확정을 교착 없음으로 해석 금지"})
            if not arguments.watch:
                break
            time.sleep(5)
    finally:
        if arguments.watch:
            lock_path.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
