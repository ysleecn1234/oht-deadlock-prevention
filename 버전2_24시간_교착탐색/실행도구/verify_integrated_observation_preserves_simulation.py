"""새 관측과 제공 버전 2 헤드리스 함수의 동작을 독립 프로세스에서 비교."""

import json
from pathlib import Path
import subprocess
import sys

from experiment_support import EXPERIMENT_ROOT, file_sha256, verify_provided_files


DEPENDENCIES = ("integrated_deadlock_observer.py", "run_integrated_deadlock_experiment.py", "simulation_observer.py",
    "run_headless_experiment.py", "experiment_support.py", "deadlock_proof.py", "candidate_review.py",
    "render_event_maps.py", "snapshot_storage.py", "key_event_image_policy.py")


def main():
    scripts = Path(__file__).resolve().parent
    hashes = {name: file_sha256(scripts / name) for name in DEPENDENCIES}
    source = verify_provided_files()
    stamp = __import__("datetime").datetime.now().strftime("%Y%m%d_%H%M%S")
    directory = EXPERIMENT_ROOT / "검증결과" / ("새관측동작보존검증_" + stamp)
    directory.mkdir()
    comparisons = []
    for num_oht, duration in ((100, 120), (1000, 600), (1500, 300)):
        signatures = []
        for observed in (False, True):
            name = f"{'새관측' if observed else '제공함수'}_차량{num_oht}_{duration}초"
            output = directory / name
            command = [sys.executable, "-u", str(scripts / ("run_integrated_deadlock_experiment.py" if observed else "run_headless_experiment.py")),
                "--case-directory", str(EXPERIMENT_ROOT / "입력자료" / "시드별입력" / "시드01"),
                "--output-directory", str(output), "--num-oht", str(num_oht), "--sim-duration", str(duration), "--seed", "1"]
            if not observed:
                command.append("--without-observation")
            with (directory / (name + ".log")).open("w", encoding="utf-8") as handle:
                subprocess.run(command, stdout=handle, stderr=subprocess.STDOUT, check=True)
            signatures.append(json.loads((output / "최종집계" / "동작대조서명.json").read_text()))
            if observed:
                summary = json.loads((output / "최종집계" / "탐지요약.json").read_text())
                assert summary["inspected_observation_count"] == duration + 1
            print(name, "완료", flush=True)
        identical = signatures[0] == signatures[1]
        comparisons.append({"num_oht": num_oht, "sim_duration": duration, "seed": 1,
            "matches": identical, "signatures": signatures})
        if not identical:
            raise RuntimeError("새 관측과 제공 함수 동작 불일치. 본실험 실행 금지")
    verify_provided_files()
    assert hashes == {name: file_sha256(scripts / name) for name in hashes}
    print("새 관측 동작 보존 검증 통과", flush=True)


if __name__ == "__main__":
    main()
