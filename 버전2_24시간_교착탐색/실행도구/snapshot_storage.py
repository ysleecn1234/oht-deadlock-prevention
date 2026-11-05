"""관측 상태 파일의 무손실 차이 압축과 복원. 시뮬레이션 코드와 독립."""

from functools import lru_cache
import gzip
import hashlib
import json
from pathlib import Path


def canonical_bytes(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")


def snapshot_sha256(value):
    return hashlib.sha256(canonical_bytes(value)).hexdigest()


def difference_patch(previous, current):
    if not isinstance(current, (dict, list)) and type(previous) is type(current) and previous == current:
        return None
    if isinstance(previous, dict) and isinstance(current, dict):
        changes = {}
        removed = sorted(set(previous) - set(current))
        for name, value in current.items():
            patch = difference_patch(previous[name], value) if name in previous else {"value": value}
            if patch is not None:
                changes[name] = patch
        return {"dictionary_changes": changes, "removed_keys": removed} if changes or removed else None
    if isinstance(previous, list) and isinstance(current, list) and len(previous) == len(current) and len(current) >= 20:
        changes = {}
        for index, (previous_value, current_value) in enumerate(zip(previous, current)):
            patch = difference_patch(previous_value, current_value)
            if patch is not None:
                changes[str(index)] = patch
        return {"list_changes": changes} if changes else None
    if isinstance(previous, list) and isinstance(current, list) and canonical_bytes(previous) == canonical_bytes(current):
        return None
    return {"value": current}


def restore_patch(previous, patch):
    if patch is None:
        return previous
    if "value" in patch:
        return patch["value"]
    if "dictionary_changes" in patch:
        result = dict(previous)
        for name in patch["removed_keys"]:
            del result[name]
        for name, change in patch["dictionary_changes"].items():
            result[name] = restore_patch(previous.get(name), change)
        return result
    result = list(previous)
    for index, change in patch["list_changes"].items():
        result[int(index)] = restore_patch(previous[int(index)], change)
    return result


@lru_cache(maxsize=64)
def read_snapshot(path):
    """현재 형식과 전체 JSON 형식을 모두 읽어 동일한 관측 상태 딕셔너리 반환."""
    path = Path(path).resolve()
    with gzip.open(path, "rt", encoding="utf-8") as handle:
        content = json.load(handle)
    if content.get("snapshot_encoding") != "lossless_difference_version_one":
        return content
    base_name = content["base_snapshot_file_name"]
    if Path(base_name).name != base_name:
        raise ValueError("상태 기준 파일은 같은 폴더의 파일이어야 함")
    previous = read_snapshot(path.parent / base_name)
    if snapshot_sha256(previous) != content["base_snapshot_sha256"]:
        raise ValueError(f"기준 상태 해시 불일치: {base_name}")
    restored = restore_patch(previous, content["difference_patch"])
    if snapshot_sha256(restored) != content["snapshot_sha256"]:
        raise ValueError(f"복원 상태 해시 불일치: {path}")
    return restored


def encode_snapshot(previous_path, previous, current):
    patch = difference_patch(previous, current)
    restored = restore_patch(previous, patch)
    expected_hash = snapshot_sha256(current)
    if snapshot_sha256(restored) != expected_hash:
        raise ValueError("무손실 상태 복원 대조 실패")
    wrapper = {
        "snapshot_encoding": "lossless_difference_version_one",
        "base_snapshot_file_name": Path(previous_path).name,
        "base_snapshot_sha256": snapshot_sha256(previous),
        "snapshot_sha256": expected_hash,
        "difference_patch": patch,
    }
    return gzip.compress(canonical_bytes(wrapper), compresslevel=6, mtime=0)
