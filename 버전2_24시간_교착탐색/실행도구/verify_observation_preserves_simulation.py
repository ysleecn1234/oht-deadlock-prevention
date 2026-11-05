"""제공 헤드리스 함수와 읽기 전용 관측 실행을 독립 프로세스로 대조."""

import argparse
import json
from pathlib import Path
import subprocess
import sys

from experiment_support import COPIED_SIMULATOR_DIRECTORY, EXPERIMENT_ROOT, file_sha256, verify_provided_files


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--regression-duration", type=int, default=600)
    arguments = parser.parse_args()
    timestamp = __import__("datetime").datetime.now().strftime("%Y%m%d_%H%M%S")
    validation_directory = EXPERIMENT_ROOT / "검증결과" / f"동작보존검증_{timestamp}"
    validation_directory.mkdir(parents=True)
    script_directory = Path(__file__).resolve().parent
    initial_hashes = {name: file_sha256(script_directory / name) for name in (
        "simulation_observer.py", "run_headless_experiment.py", "experiment_support.py")}
    cases = [(100, 120, COPIED_SIMULATOR_DIRECTORY),
             (1000, arguments.regression_duration, COPIED_SIMULATOR_DIRECTORY),
             (1500, min(arguments.regression_duration, 300), EXPERIMENT_ROOT / "입력자료" / "시드별입력" / "시드01")]
    comparisons = []
    for num_oht, sim_duration, case_directory in cases:
        signatures = []
        for without_observation in (True, False):
            name = f"{'제공함수' if without_observation else '관측추가'}_차량{num_oht}_{sim_duration}초"
            output_directory = validation_directory / name
            command = [sys.executable, "-u", str(script_directory / "run_headless_experiment.py"),
                "--case-directory", str(case_directory), "--output-directory", str(output_directory),
                "--num-oht", str(num_oht), "--sim-duration", str(sim_duration)]
            if without_observation:
                command.append("--without-observation")
            with (validation_directory / (name + ".log")).open("w", encoding="utf-8") as handle:
                subprocess.run(command, stdout=handle, stderr=subprocess.STDOUT, check=True)
            signature = json.loads((output_directory / "최종집계" / "동작대조서명.json").read_text())
            signatures.append(signature)
            print(json.dumps({"검증": name, **signature}, ensure_ascii=False), flush=True)
        matches = signatures[0] == signatures[1]
        comparisons.append({"num_oht": num_oht, "sim_duration": sim_duration,
                            "matches": matches, "signatures": signatures})
        if not matches:
            raise RuntimeError("관측 전후 제어 상태 또는 제공 기록 불일치. 본실험 실행 금지")
    manifest = verify_provided_files()
    if initial_hashes != {name: file_sha256(script_directory / name) for name in initial_hashes}:
        raise RuntimeError("검증 도중 실행 도구가 변경됨. 같은 버전으로 재검증 필요")
    print("동작 보존 검증 통과", flush=True)


if __name__ == "__main__":
    main()
