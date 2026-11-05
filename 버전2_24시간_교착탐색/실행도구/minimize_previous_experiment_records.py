"""중단된 구 배치를 예비실험 핵심 보관본으로 축소. 독립 상태와 해시 검증 후 원래 대량 기록 삭제."""

import argparse
import bisect
import gzip
import json
from pathlib import Path
import shutil

from experiment_support import EXPERIMENT_ROOT, current_timestamp, file_sha256, verify_provided_files, write_json
from render_event_maps import draw_map
from snapshot_storage import read_snapshot, snapshot_sha256


def total_bytes(directory):
    return sum(path.stat().st_size for path in directory.rglob("*") if path.is_file() and not path.is_symlink())


def preserve_snapshot(source, destination):
    snapshot = read_snapshot(source)
    with gzip.open(destination, "wt", encoding="utf-8", compresslevel=6) as handle:
        json.dump(snapshot, handle, ensure_ascii=False, separators=(",", ":"))
    with gzip.open(destination, "rt", encoding="utf-8") as handle:
        restored = json.load(handle)
    if snapshot_sha256(snapshot) != snapshot_sha256(restored):
        raise RuntimeError("독립 상태 보관본 복원 불일치")
    return snapshot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--apply", action="store_true")
    arguments = parser.parse_args()
    root = EXPERIMENT_ROOT
    comparison = root / "실험전체비교"
    for name in ("실행관리잠금.json", "확정검증잠금.json", "후보우선검토잠금.json", "상태압축잠금.json", "결과추출/자동추출잠금.json"):
        if (comparison / name).exists():
            raise RuntimeError(f"구 배치 관련 프로세스가 남아 있음: {name}")
    verify_provided_files()
    queue = json.loads((comparison / "현재실험목록.json").read_text())
    if queue["running_count"]:
        raise RuntimeError("실행 중 실험 삭제 금지")
    batch = Path(queue["batch_directory"]).resolve()
    if batch.parent != (root / "실행결과").resolve() or batch.name != "기본수요_24시간_20261101_113704":
        raise RuntimeError("검토한 구 배치 외 삭제 금지")
    initial_size = total_bytes(batch)
    print("구 배치", batch.name, "전체 GiB", round(initial_size / 1024**3, 3), flush=True)
    if not arguments.apply:
        return
    output = root / "이전실험핵심보관" / batch.name
    output.mkdir(parents=True, exist_ok=False)
    graph = json.loads((root / "입력자료" / "지도와설비" / "agv_graph.json").read_text())
    receipt = {"started_at": current_timestamp(), "original_batch_directory": str(batch),
        "classification": "예비실험·유형별 탐지 분석 미검증. 새 본실험 결과로 사용 금지",
        "original_bytes": initial_size, "experiments": [], "files": [],
        "discarded_material": ["대량 연속 공유 상태", "구간별 중간 시계열", "반복·중복 이미지와 사건 참조 목록",
            "대량 합류 통과·정체 원기록", "거의 같은 중간 집계"],
        "preserved_material": ["입력·시드·버전·실행 조건", "최종 처리 실적·구간 누적 대기",
            "대표 후보 독립 상태·전체/확대 이미지·근거", "확정 사례가 있으면 전체 보존", "정리 이력"]}

    def copy_checked(source, destination):
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
        digest = file_sha256(source)
        if file_sha256(destination) != digest:
            raise RuntimeError("핵심 파일 사본 불일치")
        receipt["files"].append({"source": str(source), "saved_path": str(destination), "sha256": digest})

    for name in ("실험배치조건.json", "배치종료상태.json"):
        if (batch / name).exists():
            copy_checked(batch / name, output / name)
    write_json(output / "이전실험목록.json", queue)
    for task in queue["experiments"]:
        run = Path(task["output_directory"])
        if not run.exists():
            continue
        destination = output / run.name
        destination.mkdir()
        for relative in ("실험조건.json", "중간집계/현재진행상태.json", "최종집계/추출결과요약.json",
                         "최종집계/동작대조서명.json", "최종집계/구간별누적대기.json"):
            if (run / relative).exists():
                copy_checked(run / relative, destination / relative)
        for folder in ("유형별우선검토후보", "미해소장기대기사례"):
            if (run / folder).exists():
                shutil.copytree(run / folder, destination / folder)
                for record_path in (destination / folder).glob("**/*.json.gz"):
                    with gzip.open(record_path, "rt", encoding="utf-8") as handle:
                        json.load(handle)
        proof_path = run / "확정교착기록/검사상태.json"
        if proof_path.exists():
            proof = json.loads(proof_path.read_text())
            write_json(destination / "교착검사범위.json", {key: value for key, value in proof.items()
                if key != "processed_snapshot_files"})
            for confirmed in (run / "확정교착기록").glob("확정*"):
                if confirmed.is_dir():
                    shutil.copytree(confirmed, destination / "확정교착기록" / confirmed.name)
        events = []
        for path in (run / "사건별기록").glob("*/사건정보.json"):
            event = json.loads(path.read_text())
            if event["label"] == "순환대기후보":
                events.append((path, event))
        selected = {}
        for target_type in ("TYPE I", "TYPE II", "TYPE III", "TYPE IV"):
            matches = [item for item in events if any(value["type"] == target_type for value in item[1]["type_hypotheses"])]
            if matches:
                path, event = max(matches, key=lambda item: item[1]["evidence"].get("stable_dependency_duration_seconds", 0))
                selected.setdefault(path, {"event": event, "types": []})["types"].append(target_type)
        state_paths = sorted((run / "사건상태공유보관").glob("시각_*.json.gz"))
        seconds = [float(path.name.removeprefix("시각_").removesuffix("초.json.gz")) for path in state_paths]
        for event_path, item in selected.items():
            event = item["event"]
            candidate_output = destination / "대표순환대기후보" / event_path.parent.name
            candidate_output.mkdir(parents=True)
            references = []
            for label, second in (("최초포착", event["first_detection_second"]), ("마지막포착", event["last_observed_second"]),
                                  ("이후진행확인", min(seconds[-1], event["last_observed_second"] + 600))):
                index = min(bisect.bisect_left(seconds, second), len(state_paths) - 1)
                saved = candidate_output / f"{label}상태.json.gz"
                snapshot = preserve_snapshot(state_paths[index], saved)
                references.append({"label": label, "simulation_second": snapshot["simulation_second"], "saved_path": str(saved)})
                if label == "마지막포착":
                    caption = "이전 예비실험의 대표 순환 대기 후보. 유형·교착 확정 검증 미완료"
                    draw_map(snapshot, graph, candidate_output / "전체시뮬레이션.png", run.name, event, status_caption=caption)
                    draw_map(snapshot, graph, candidate_output / "의심구간확대.png", run.name, event, cropped=True, status_caption=caption)
            write_json(candidate_output / "예비후보근거.json", {"event": event, "related_types": item["types"],
                "state_references": references, "verdict": "이전 탐색의 대표 후보·미검증", "type_reference": "10가지 자료의 1~4번"})
        receipt["experiments"].append({"experiment_number": task["experiment_number"], "status": task["status"],
            "saved_directory": str(destination), "representative_circular_candidate_count": len(selected)})
    # 과거 집계는 작은 압축 감사 기록으로 남기고 활성 결과와 분리한다.
    for relative in ("결과추출/실험별요약.json", "결과추출/유형별후보집계.json", "결과추출/완료알림이력.json",
                     "후보검토기준.json", "우선검토후보현황.json", "미해소후보검토현황.json"):
        source = comparison / relative
        if source.exists():
            target = output / "과거집계" / (source.name + ".gz")
            target.parent.mkdir(exist_ok=True)
            original = source.read_bytes()
            encoded = gzip.compress(original, compresslevel=9, mtime=0)
            if gzip.decompress(encoded) != original:
                raise RuntimeError("과거 집계 무손실 보존 검증 실패")
            target.write_bytes(encoded)
    receipt.update(preservation_verified_at=current_timestamp(), saved_bytes=total_bytes(output))
    write_json(output / "핵심보관및정리이력.json", receipt)
    verify_provided_files()
    # 위에서 지정한 단일 구 배치만 제거. 원본·입력·코드·보관본은 대상이 아님.
    shutil.rmtree(batch)
    receipt.update(finished_at=current_timestamp(), removed_bytes=initial_size,
        reclaimed_bytes=initial_size - receipt["saved_bytes"], original_large_batch_removed=True)
    write_json(output / "핵심보관및정리이력.json", receipt)
    write_json(comparison / "이전실험판정/핵심보관위치.json", {"archive_directory": str(output),
        "classification": receipt["classification"], "original_large_batch_removed": True,
        "note": "과거 절대 경로는 출처 기록이며 대량 원본은 요청에 따라 제거됨. 독립 상태 파일은 보관본에서 직접 복원 가능"})
    print(json.dumps({"saved_mib": round(receipt["saved_bytes"] / 1024**2, 2),
        "reclaimed_gib": round(receipt["reclaimed_bytes"] / 1024**3, 2), "archive_directory": str(output)}, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
