"""재생성 가능한 과거 일반 사건 이미지만 정리하고 대기 관계 내보내기를 무손실 압축."""

import argparse
from collections import Counter
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import time

from experiment_support import EXPERIMENT_ROOT, current_timestamp, file_sha256, verify_provided_files, write_json
from snapshot_storage import read_snapshot


OLD_IMAGE_NAMES = {"발생전.png", "최초포착.png", "관측종료.png"}


def check_preservation_contract():
    verify_provided_files()
    validation = json.loads((EXPERIMENT_ROOT / "검증결과" / "동작보존검증.json").read_text())
    for filename, field in (("simulation_observer.py", "observer_sha256"),
                            ("run_headless_experiment.py", "runner_sha256"),
                            ("experiment_support.py", "support_sha256")):
        if file_sha256(EXPERIMENT_ROOT / "실행도구" / filename) != validation[field]:
            raise RuntimeError(f"검증된 실행 파일 불일치: {filename}")


def inspect_candidates():
    image_paths, relationship_paths = [], []
    kept_cycle_images = 0
    checked_runs = []
    for run in sorted((EXPERIMENT_ROOT / "실행결과").glob("*/*")):
        if not (run / "실험조건.json").exists():
            continue
        state_paths = sorted((run / "사건상태공유보관").glob("시각_*.json.gz"))
        if not state_paths:
            continue
        # 기준 파일 체인을 포함한 실제 상태 복원이 가능한지 확인한다.
        restored = read_snapshot(state_paths[len(state_paths) // 2])
        if not restored.get("vehicles") or "wait_dependencies" not in restored:
            raise RuntimeError(f"재생성 원자료 확인 실패: {run}")
        checked_runs.append(run.name)
        for event_path in sorted((run / "사건별기록").glob("*/사건정보.json")):
            event = json.loads(event_path.read_text())
            references_path = event_path.parent / "발생전후상태" / "상태파일목록.json"
            if not references_path.exists():
                continue
            references = json.loads(references_path.read_text()).get("paths_relative_to_experiment_directory", [])
            if not references:
                continue
            endpoints = [run / references[0], run / references[-1]]
            if not all(path.is_file() and path.resolve().is_relative_to(run.resolve() / "사건상태공유보관")
                       for path in endpoints):
                continue
            circular_candidate = "순환대기후보" in {event["label"], *event.get("observed_labels", [])}
            for folder in ("전체지도", "구간확대지도"):
                for name in OLD_IMAGE_NAMES:
                    image_path = event_path.parent / folder / name
                    if not image_path.is_file() or image_path.is_symlink():
                        continue
                    if circular_candidate:
                        kept_cycle_images += 1
                    elif time.time() - image_path.stat().st_mtime > 10:
                        image_paths.append(image_path)
            relationship_path = event_path.parent / "대기관계" / "최초포착관계.json"
            if (relationship_path.is_file() and not relationship_path.is_symlink()
                    and time.time() - relationship_path.stat().st_mtime > 10):
                relationship_paths.append(relationship_path)
    return image_paths, relationship_paths, kept_cycle_images, checked_runs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--apply", action="store_true")
    arguments = parser.parse_args()
    check_preservation_contract()
    images, relationships, kept_cycles, checked_runs = inspect_candidates()
    summary = {"started_at": current_timestamp(), "applied": arguments.apply,
        "candidate_image_count": len(images), "candidate_image_bytes": sum(path.stat().st_size for path in images),
        "relationship_export_count": len(relationships), "kept_circular_candidate_image_count": kept_cycles,
        "checked_experiments": checked_runs,
        "preserved": ["제공 버전 2 원본·사본", "검증된 실행기·관측기", "모든 공유 상태와 압축 기준 파일",
            "구간별 시계열", "사건 정보·상태 참조 목록", "핵심 이미지", "순환 대기 후보 이미지",
            "확정 교착 기록", "오류·실행 로그", "최종 집계"],
        "note": "일반 사건의 과거 발생 전·최초 포착·관측 종료 파생 이미지만 삭제. 대기 관계 내보내기는 바이트 단위 복원 검증 후 gzip으로 보존"}
    if not arguments.apply:
        print(json.dumps(summary, ensure_ascii=False), flush=True)
        return
    output = EXPERIMENT_ROOT / "실험전체비교" / "저장공간정리" / time.strftime("정리_%Y%m%d_%H%M%S")
    output.mkdir(parents=True, exist_ok=False)
    summary["free_bytes_before"] = shutil.disk_usage(EXPERIMENT_ROOT).free
    write_json(output / "정리요약.json", summary)
    removed_count = removed_bytes = compressed_count = compressed_saved_bytes = 0
    experiments = Counter()
    with (output / "파일별정리기록.jsonl").open("w", encoding="utf-8") as journal:
        for path in images:
            if not path.exists():
                continue
            original_size = path.stat().st_size
            entry = {"action": "재생성 가능한 과거 일반 사건 이미지 삭제",
                "path": str(path), "bytes": original_size}
            journal.write(json.dumps(entry, ensure_ascii=False) + "\n")
            journal.flush()
            path.unlink()
            removed_count += 1
            removed_bytes += original_size
            experiments[path.relative_to(EXPERIMENT_ROOT).parts[2]] += 1
        for path in relationships:
            original = path.read_bytes()
            encoded = gzip.compress(original, compresslevel=9, mtime=0)
            if gzip.decompress(encoded) != original:
                raise RuntimeError(f"대기 관계 무손실 복원 검증 실패: {path}")
            destination = path.with_suffix(".json.gz")
            if destination.exists() and gzip.decompress(destination.read_bytes()) != original:
                raise RuntimeError(f"기존 압축 내보내기와 내용 불일치: {destination}")
            if len(encoded) >= len(original):
                continue
            temporary = destination.with_name(destination.name + ".writing")
            temporary.write_bytes(encoded)
            temporary.replace(destination)
            entry = {"action": "대기 관계 내보내기 무손실 압축", "path": str(path),
                "saved_path": str(destination), "original_bytes": len(original), "compressed_bytes": len(encoded),
                "uncompressed_sha256": hashlib.sha256(original).hexdigest(), "round_trip_verified": True}
            journal.write(json.dumps(entry, ensure_ascii=False) + "\n")
            journal.flush()
            path.unlink()
            compressed_count += 1
            compressed_saved_bytes += len(original) - len(encoded)
    check_preservation_contract()
    summary.update(finished_at=current_timestamp(), removed_image_count=removed_count,
        removed_image_bytes=removed_bytes, removed_images_by_experiment=dict(experiments),
        compressed_relationship_count=compressed_count, compressed_relationship_bytes_saved=compressed_saved_bytes,
        logical_bytes_reclaimed=removed_bytes + compressed_saved_bytes,
        free_bytes_after=shutil.disk_usage(EXPERIMENT_ROOT).free, preservation_contract_verified=True)
    write_json(output / "정리요약.json", summary)
    print(json.dumps({**summary, "report_directory": str(output)}, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
