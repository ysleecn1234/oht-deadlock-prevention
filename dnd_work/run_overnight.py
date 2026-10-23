# -*- coding: utf-8 -*-
"""1차(대수 격자)와 2차(시드 반복)를 이어서 돌린다.

[왜 필요한가]
1차만 돌리면 코어가 논다. 대수가 클수록 오래 걸려서, 2,000대 하나가
12~15시간 도는 동안 먼저 끝난 7개 자리가 비어 있게 된다.

그래서 작업 대기열 방식으로 바꾼다. 1차 조건이 하나 끝날 때마다 결과를
읽어, 부하율이 기준(기본 85%) 이상이면 그 대수를 여러 시드로 돌리는
2차 작업을 대기열에 밀어넣는다. 빈 코어가 생기는 즉시 거기 들어간다.

[왜 부하율 기준인가]
교수님 지시가 "OHT 수를 점차 증가시켜 부하율이 80 85 90 95 로 올라가는
걸 보라" 이고, 데드락은 부하가 높은 구간에서 기대된다. 부하율이 낮은
대수는 시드를 여러 개 돌려도 얻을 게 적으므로 2차에서 제외한다.

환경변수
    CASE_ROOT                   케이스 폴더들의 상위 폴더     기본 ../cases
    FIRST_PASS_CASE             1차에 쓸 케이스               기본 s01
    FIRST_PASS_OHT_COUNTS       1차 대수 목록
    SECOND_PASS_CASES           2차에 쓸 케이스 목록          기본 s02~s10
    UTILIZATION_THRESHOLD       2차로 넘길 부하율 기준(%)     기본 85
    SPAWN_INTERVAL_SECONDS      로트 투입 간격(초)            기본 0.45
    SIMULATION_HOURS            시뮬레이션 시간 상한          기본 24
    PARALLEL_PROCESSES          동시 실행 개수                기본 코어 수
    PENDING_JOB_LIMIT           대기열 상한                   기본 2000
    RESULT_DIRECTORY            결과 폴더                     기본 ../runs
    SKIP_EXISTING_RESULTS       이미 결과 파일이 있으면 건너뛴다  기본 1
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


CASE_ROOT              = os.path.abspath(os.environ.get(
    "CASE_ROOT", os.path.join(HERE, "..", "cases")))
FIRST_PASS_CASE        = os.environ.get("FIRST_PASS_CASE", "s01")
FIRST_PASS_OHT_COUNTS  = [int(v) for v in os.environ.get(
    "FIRST_PASS_OHT_COUNTS", "200,400,600,800,1000,1200,1500,2000").split(",")]
SECOND_PASS_CASES      = [v.strip() for v in os.environ.get(
    "SECOND_PASS_CASES", "s02,s03,s04,s05,s06,s07,s08,s09,s10").split(",")
    if v.strip()]
UTILIZATION_THRESHOLD  = _environment_value("UTILIZATION_THRESHOLD", float, 85.0)
SPAWN_INTERVAL_SECONDS = _environment_value("SPAWN_INTERVAL_SECONDS", float, 0.45)
SIMULATION_HOURS       = _environment_value("SIMULATION_HOURS", float, 24.0)
PARALLEL_PROCESSES     = _environment_value(
    "PARALLEL_PROCESSES", int, max(1, os.cpu_count() or 4))
PENDING_JOB_LIMIT      = _environment_value("PENDING_JOB_LIMIT", int, 2000)
# 중간에 멈췄다 다시 돌릴 때 이미 끝난 조건을 또 돌리지 않는다.
SKIP_EXISTING_RESULTS  = _environment_value(
    "SKIP_EXISTING_RESULTS", lambda v: v != "0", True)
RESULT_DIRECTORY       = os.path.abspath(os.environ.get(
    "RESULT_DIRECTORY", os.path.join(HERE, "..", "runs")))

os.makedirs(RESULT_DIRECTORY, exist_ok=True)


def make_job(case_name, oht_count, pass_number):
    name = "%s_oht%d_interval%.2f_fix_on" % (
        case_name, oht_count, SPAWN_INTERVAL_SECONDS)
    environment = dict(os.environ)
    environment.update(
        CASE_DIRECTORY=os.path.join(CASE_ROOT, case_name),
        OHT_COUNT=str(oht_count),
        SPAWN_INTERVAL_SECONDS=str(SPAWN_INTERVAL_SECONDS),
        SIMULATION_HOURS=str(SIMULATION_HOURS),
        PENDING_JOB_LIMIT=str(PENDING_JOB_LIMIT),
        APPLY_FIXES="1",
        RESULT_JSON_PATH=os.path.join(RESULT_DIRECTORY, name + ".json"),
        MPLBACKEND="Agg",
    )
    return {"name": name, "case": case_name, "oht_count": oht_count,
            "pass": pass_number, "environment": environment,
            "log_path": os.path.join(RESULT_DIRECTORY, name + ".log"),
            "result_path": environment["RESULT_JSON_PATH"]}


def log(message):
    print("[%s] %s" % (time.strftime("%m/%d %H:%M"), message), flush=True)


def main():
    missing = [c for c in [FIRST_PASS_CASE] + SECOND_PASS_CASES
               if not os.path.isdir(os.path.join(CASE_ROOT, c))]
    if missing:
        print("케이스 폴더가 없다: %s" % ", ".join(missing))
        print("generate_case_files.py 로 먼저 만들어야 한다.")
        return 1

    # 대수가 큰 것부터 띄운다. 느린 조건이 마지막에 혼자 남지 않게.
    queue = [make_job(FIRST_PASS_CASE, n, 1)
             for n in sorted(FIRST_PASS_OHT_COUNTS, reverse=True)]
    running, finished, promoted = [], [], []

    log("1차 %d개 조건, 동시 실행 %d개, 시뮬 %.0f시간"
        % (len(queue), PARALLEL_PROCESSES, SIMULATION_HOURS))
    log("부하율 %.0f%% 이상인 대수는 2차(시드 %d개)로 넘긴다"
        % (UTILIZATION_THRESHOLD, len(SECOND_PASS_CASES)))
    started_at = time.time()

    while queue or running:
        while queue and len(running) < PARALLEL_PROCESSES:
            job = queue.pop(0)
            if SKIP_EXISTING_RESULTS and os.path.exists(job["result_path"]):
                # 이미 끝난 조건이다. 다시 돌리지 않고 결과만 읽어
                # 2차 승급 판단에 쓴다.
                job["process"] = None
                job["started_at"] = time.time()
                finished.append(job)
                try:
                    with open(job["result_path"], encoding="utf-8") as handle:
                        done = json.load(handle)
                except OSError:
                    continue
                log("건너뜀  %s  (이미 결과 있음, 부하율 %.1f%%)"
                    % (job["name"], done["utilization_percent"]))
                if (job["pass"] == 1
                        and done["utilization_percent"] >= UTILIZATION_THRESHOLD
                        and job["oht_count"] not in promoted):
                    promoted.append(job["oht_count"])
                    for case_name in SECOND_PASS_CASES:
                        queue.append(make_job(case_name, job["oht_count"], 2))
                continue
            handle = open(job["log_path"], "w", encoding="utf-8")
            job["process"] = subprocess.Popen(
                [sys.executable, "-u", os.path.join(HERE, "deadlock_detector.py")],
                env=job["environment"], stdout=handle,
                stderr=subprocess.STDOUT, text=True)
            job["log_handle"] = handle
            job["started_at"] = time.time()
            running.append(job)
            log("시작  %d차  %s" % (job["pass"], job["name"]))

        time.sleep(10)
        for job in list(running):
            if job["process"].poll() is None:
                continue
            job["log_handle"].close()
            running.remove(job)
            finished.append(job)
            minutes = (time.time() - job["started_at"]) / 60

            result = None
            try:
                with open(job["result_path"], encoding="utf-8") as handle:
                    result = json.load(handle)
            except OSError:
                log("실패  %s  (%.0f분) - 결과 없음, 로그 확인" % (job["name"], minutes))
                continue

            found = result["held_over_limit_count"]
            log("완료  %s  (%.0f분)  부하율 %.1f%%  순환 %d건  교착 %d건%s"
                % (job["name"], minutes, result["utilization_percent"],
                   result["cycle_count"], found, "   <<<<< 교착 발생" if found else ""))

            # 1차 결과가 기준을 넘으면 그 대수를 2차로 넘긴다
            if (job["pass"] == 1
                    and result["utilization_percent"] >= UTILIZATION_THRESHOLD
                    and job["oht_count"] not in promoted):
                promoted.append(job["oht_count"])
                for case_name in SECOND_PASS_CASES:
                    queue.append(make_job(case_name, job["oht_count"], 2))
                log("  -> 부하율 %.1f%% >= %.0f%%. %d대를 시드 %d개로 2차 투입 (대기열 %d개)"
                    % (result["utilization_percent"], UTILIZATION_THRESHOLD,
                       job["oht_count"], len(SECOND_PASS_CASES), len(queue)))

    log("전체 %.1f시간 소요, %d개 실행" % ((time.time() - started_at) / 3600, len(finished)))
    print("")
    print("%-6s %-6s %5s %7s %7s %7s %9s %8s %6s %9s %7s"
          % ("차수", "케이스", "대수", "부하율", "실동률", "대기율",
             "반송완료", "로트완료", "순환수", "최장(초)", "교착"))
    rows = []
    for job in finished:
        try:
            with open(job["result_path"], encoding="utf-8") as handle:
                result = json.load(handle)
        except OSError:
            continue
        rows.append((job, result))
    for job, result in sorted(rows, key=lambda r: (r[0]["pass"], -r[0]["oht_count"], r[0]["case"])):
        print("%-6d %-6s %5d %6.1f%% %6.1f%% %6.1f%% %9d %8d %6d %9.0f %7d%s"
              % (job["pass"], job["case"], job["oht_count"],
                 result["utilization_percent"], result["active_percent"],
                 result["stall_percent"], result["completed_transfers"],
                 result["completed_lots"], result["cycle_count"],
                 result["longest_cycle_seconds"], result["held_over_limit_count"],
                 "  <-- 교착" if result["held_over_limit_count"] else ""))

    found = [(j, r) for j, r in rows if r["held_over_limit_count"]]
    print("")
    if found:
        print("교착이 나온 조건 %d개:" % len(found))
        for job, result in found:
            print("  %s %d대  -  결과 파일 %s"
                  % (job["case"], job["oht_count"], os.path.basename(job["result_path"])))
    else:
        print("교착 판정 0건.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
