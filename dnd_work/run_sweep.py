# -*- coding: utf-8 -*-
"""대수를 바꿔가며 deadlock_detector.py 를 병렬로 돌린다.

헤드리스 실행에는 배속 개념이 없다. 화면을 그리지 않으므로 처음부터 CPU가
낼 수 있는 최대 속도로 돈다. 더 빠르게 하는 방법은 여러 조건을 동시에
돌리는 것뿐이다. 이 스크립트가 그 역할을 한다.

환경변수

    OHT_COUNT_LIST          쉼표로 구분한 대수 목록
                            기본 "200,300,400,500,600,800,1000"
    SPAWN_INTERVAL_SECONDS  로트 투입 간격(초)   기본 0.45 (= 8,000건/시간)
    SIMULATION_HOURS        시뮬레이션 시간 상한  기본 6
    APPLY_FIXES             1이면 수정 적용, 0이면 원문  기본 1
    PARALLEL_PROCESSES      동시에 돌릴 개수      기본 CPU 코어 수 - 1
    RESULT_DIRECTORY        결과 저장 폴더        기본 ../runs
    CASE_DIRECTORY          시드 케이스 폴더 (generate_case_files.py 로 만든다)
    PENDING_JOB_LIMIT       대기열 상한(건). 넘으면 투입을 건너뛴다
    ENABLE_RING_CONTROL     베이 고리 진입 제어
    RING_CAPACITY_MARGIN    고리 정원 = 노드수//2 - 이 값
"""
import os
import sys
import json
import time
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))


def _environment_value(name, cast, default):
    raw = os.environ.get(name)
    return default if raw is None else cast(raw)


OHT_COUNT_LIST = [int(v) for v in os.environ.get(
    "OHT_COUNT_LIST", "200,300,400,500,600,800,1000").split(",") if v.strip()]
SPAWN_INTERVAL_SECONDS = _environment_value("SPAWN_INTERVAL_SECONDS", float, 0.45)
SIMULATION_HOURS       = _environment_value("SIMULATION_HOURS", float, 6.0)
APPLY_FIXES            = _environment_value("APPLY_FIXES", lambda v: v != "0", True)
PARALLEL_PROCESSES     = _environment_value(
    "PARALLEL_PROCESSES", int, max(1, (os.cpu_count() or 2) - 1))
RESULT_DIRECTORY       = os.environ.get(
    "RESULT_DIRECTORY", os.path.join(HERE, "..", "runs"))

os.makedirs(RESULT_DIRECTORY, exist_ok=True)
FIX_LABEL = "fix_on" if APPLY_FIXES else "fix_off"


CASE_LABEL = ""
if os.environ.get("CASE_DIRECTORY"):
    CASE_LABEL = os.path.basename(os.path.abspath(os.environ["CASE_DIRECTORY"])) + "_"


def build_condition(oht_count):
    name = "%soht%d_interval%.2f_%s" % (
        CASE_LABEL, oht_count, SPAWN_INTERVAL_SECONDS, FIX_LABEL)
    environment = dict(os.environ)
    environment.update(
        OHT_COUNT=str(oht_count),
        SPAWN_INTERVAL_SECONDS=str(SPAWN_INTERVAL_SECONDS),
        SIMULATION_HOURS=str(SIMULATION_HOURS),
        APPLY_FIXES="1" if APPLY_FIXES else "0",
        RESULT_JSON_PATH=os.path.join(RESULT_DIRECTORY, name + ".json"),
        MPLBACKEND="Agg",
    )
    return {
        "name": name,
        "oht_count": oht_count,
        "environment": environment,
        "log_path": os.path.join(RESULT_DIRECTORY, name + ".log"),
        "result_path": environment["RESULT_JSON_PATH"],
    }


def main():
    # 대수가 클수록 오래 걸리므로 큰 것부터 띄운다. 그래야 마지막에
    # 느린 조건 하나만 남아 전체가 늘어지는 일이 줄어든다.
    conditions = [build_condition(n) for n in sorted(OHT_COUNT_LIST, reverse=True)]
    print("조건 %d개를 최대 %d개씩 동시에 실행한다." % (len(conditions), PARALLEL_PROCESSES))
    print("투입 %.2f초(= %.0f건/시간) / 시뮬 상한 %.1f시간 / %s"
          % (SPAWN_INTERVAL_SECONDS, 3600 / SPAWN_INTERVAL_SECONDS,
             SIMULATION_HOURS, "수정 적용" if APPLY_FIXES else "원문"))
    print("결과 폴더: %s" % os.path.abspath(RESULT_DIRECTORY))
    print("")

    waiting, running, finished = list(conditions), [], []
    started_at = time.time()
    while waiting or running:
        while waiting and len(running) < PARALLEL_PROCESSES:
            condition = waiting.pop(0)
            handle = open(condition["log_path"], "w", encoding="utf-8")
            process = subprocess.Popen(
                [sys.executable, "-u", os.path.join(HERE, "deadlock_detector.py")],
                env=condition["environment"], stdout=handle,
                stderr=subprocess.STDOUT, text=True)
            condition["process"] = process
            condition["log_handle"] = handle
            condition["started_at"] = time.time()
            running.append(condition)
            print("[시작] %s" % condition["name"], flush=True)

        time.sleep(5)
        for condition in list(running):
            if condition["process"].poll() is None:
                continue
            condition["log_handle"].close()
            running.remove(condition)
            finished.append(condition)
            elapsed = time.time() - condition["started_at"]
            print("[완료] %-34s %6.0f초" % (condition["name"], elapsed), flush=True)

    print("")
    print("전체 %.0f분 소요" % ((time.time() - started_at) / 60))
    print("")
    print("%-6s %7s %7s %7s %9s %8s %6s %9s %7s"
          % ("대수", "부하율", "실동률", "대기율", "반송완료", "로트환산",
             "순환수", "최장(초)", "600초↑"))
    rows = []
    for condition in sorted(finished, key=lambda c: c["oht_count"]):
        try:
            with open(condition["result_path"], encoding="utf-8") as handle:
                result = json.load(handle)
        except OSError:
            print("%-6d  (결과 파일 없음 - %s 확인)"
                  % (condition["oht_count"], os.path.basename(condition["log_path"])))
            continue
        print("%-6d %6.1f%% %6.1f%% %6.1f%% %9d %8d %6d %9.0f %7d%s"
              % (condition["oht_count"],
                 result["utilization_percent"], result["active_percent"],
                 result["stall_percent"],
                 result["completed_transfers"], result["completed_lots"],
                 result["cycle_count"], result["longest_cycle_seconds"],
                 result["cycles_over_threshold"]["over_600s"],
                 "  <-- 교착" if result["held_over_limit_count"] else ""))
        rows.append(result)

    found = [r for r in rows if r["held_over_limit_count"] > 0]
    if found:
        print("")
        print("★ 교착 판정이 나온 조건: %s"
              % ", ".join("%d대" % r["oht_count"] for r in found))


if __name__ == "__main__":
    main()
