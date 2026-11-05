"""저장이 끝난 관측 상태만 무손실 압축. 원본·시뮬레이터·실행 중인 상태는 건드리지 않음."""

import argparse
import gzip
import json
import os
from pathlib import Path
import time

from experiment_support import EXPERIMENT_ROOT, current_timestamp, verify_provided_files, write_json
from snapshot_storage import encode_snapshot, read_snapshot, restore_patch, snapshot_sha256


def compress_directory(directory, maximum_files):
    record_path = directory / "압축보관정보.json"
    record = json.loads(record_path.read_text()) if record_path.exists() else {"processed_files": [], "bytes_saved": 0}
    processed_files = set(record["processed_files"])
    paths = sorted(directory.glob("시각_*.json.gz"))
    previous = previous_path = None
    compressed_count = examined_count = chain_length = 0
    saved_bytes = 0
    for path in paths:
        if path.name in processed_files:
            # 앞선 프레임은 필요할 때 복원. 기존 압축 체인은 최대 20개.
            previous_path = path
            previous = None
            continue
        if time.time() - path.stat().st_mtime < 10:
            break
        with gzip.open(path, "rt", encoding="utf-8") as handle:
            content = json.load(handle)
        if content.get("snapshot_encoding") == "lossless_difference_version_one":
            processed_files.add(path.name)
            previous_path, previous = path, None
            continue
        current = content
        # 매 20번째 프레임은 독립된 전체 상태로 유지하여 복원 길이를 제한.
        frame_number = len(processed_files)
        if previous_path is not None and frame_number % 20 != 0:
            if previous is None:
                previous = read_snapshot(previous_path)
            encoded = encode_snapshot(previous_path, previous, current)
            original_size = path.stat().st_size
            if len(encoded) < original_size * 0.95:
                # 쓰기 전에 압축된 실제 바이트를 다시 풀어 현재 상태와 대조.
                wrapper = json.loads(gzip.decompress(encoded))
                restored = restore_patch(previous, wrapper["difference_patch"])
                if snapshot_sha256(restored) != snapshot_sha256(current):
                    raise RuntimeError("저장 전 무손실 대조 실패")
                temporary = path.with_name(path.name + ".writing")
                temporary.write_bytes(encoded)
                temporary.replace(path)
                saved_bytes += original_size - len(encoded)
                compressed_count += 1
        previous_path, previous = path, current
        processed_files.add(path.name)
        examined_count += 1
        if examined_count >= maximum_files:
            break
    record.update(updated_at=current_timestamp(), processed_files=sorted(processed_files),
                  bytes_saved=record["bytes_saved"] + saved_bytes)
    write_json(record_path, record)
    write_json(directory / "상태복원방법.json", {
        "encoding": "전체 상태 또는 SHA-256으로 복원 검증한 무손실 차이 압축",
        "loader": str(EXPERIMENT_ROOT / "실행도구" / "snapshot_storage.py"),
        "function": "read_snapshot(path)", "maximum_difference_chain_length": 19,
        "note": "기준 파일을 포함한 상태 폴더 전체를 함께 보존. 시뮬레이션 제어와 무관한 저장 후처리"})
    return examined_count, compressed_count, saved_bytes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--watch", action="store_true")
    parser.add_argument("--maximum-files-per-directory", type=int, default=60)
    arguments = parser.parse_args()
    lock_path = EXPERIMENT_ROOT / "실험전체비교" / "상태압축잠금.json"
    if arguments.watch:
        with lock_path.open("x", encoding="utf-8") as handle:
            json.dump({"process_id": os.getpid(), "started_at": current_timestamp()}, handle)
    try:
        while True:
            verify_provided_files()
            examined = compressed = saved = 0
            for directory in sorted((EXPERIMENT_ROOT / "실행결과").glob("*/*/사건상태공유보관")):
                result = compress_directory(directory, arguments.maximum_files_per_directory)
                examined += result[0]
                compressed += result[1]
                saved += result[2]
            write_json(EXPERIMENT_ROOT / "실험전체비교" / "상태압축진행.json", {
                "updated_at": current_timestamp(), "examined_count": examined,
                "compressed_count": compressed, "bytes_saved_this_pass": saved,
                "lossless_round_trip_verification": True})
            if examined:
                print(current_timestamp(), examined, compressed, saved, flush=True)
            if not arguments.watch:
                break
            time.sleep(10)
    finally:
        if arguments.watch:
            lock_path.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
