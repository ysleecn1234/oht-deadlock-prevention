# -*- coding: utf-8 -*-
"""시드 케이스 폴더를 만든다.

교수님 지시 "랜덤 시드를 변경해가면서 데드락 발생을 관찰" 을 위한 준비다.
원본의 run_headless_sim(seed=...) 인자는 작동하지 않는다. 코드 어디에서도
전역 random 을 직접 쓰지 않고, 시작위치는 Random(0), 로트 스케줄은
Random(1) 로 각각 고정되어 있기 때문이다. 그래서 시드를 바꾸려면 두 파일을
시드별로 따로 구워야 한다.

    oht_park_positions.json   OHT 시작(주차) 위치 목록
    oht_lot_schedule.json     로트 사양(출발/목적 포트, 긴급도) 목록

본체는 이 파일들을 파일명만으로 연다(= 현재 작업 폴더 기준). 그래서 케이스마다
폴더를 하나씩 만들고, 공통 지도 파일 4종은 링크로 걸어둔다. 그 폴더로 옮겨간
뒤 실행하면 그 케이스의 시작위치와 로트가 쓰인다. 폴더가 독립이라 여러 시드를
동시에 돌려도 서로 간섭하지 않는다.

s00 은 지금 쓰고 있는 파일을 그대로 복사해 보관한다. 과거 실험과 대조할 때
필요하며, 새로 뽑지 않는다.

환경변수

    CASE_NUMBERS            만들 케이스 번호. 예 "1,2,3" 또는 "1-10"
    PARK_POSITION_COUNT     시작위치 개수(= 돌릴 수 있는 최대 대수)  기본 3000
    LOT_SCHEDULE_COUNT      로트 스케줄 건수                        기본 60000
    CASE_ROOT               케이스 폴더들을 담을 상위 폴더           기본 ../cases
    INCLUDE_BASE_CASE       1이면 s00(현재 파일 복사)도 만든다       기본 1
"""
import os
import sys
import json
import random
import shutil

sys.argv = ["agv_fab_sim.py", "--analyze"]
SCRIPT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIRECTORY)
os.chdir(SCRIPT_DIRECTORY)
import agv_fab_sim as simulator

MANIFEST_FILE = "case_info.json"
SHARED_MAP_FILES = ["track_segments.json", "agv_graph.json",
                    "equipment.json", "section_regions.json"]
CASE_FILES = ["oht_park_positions.json", "oht_lot_schedule.json"]


def _environment_value(name, cast, default):
    raw = os.environ.get(name)
    return default if raw is None else cast(raw)


def parse_case_numbers(text):
    """\"1,2,3\" 또는 \"1-10\" 또는 \"1-5,8\" 을 번호 목록으로."""
    numbers = []
    for piece in text.split(","):
        piece = piece.strip()
        if not piece:
            continue
        if "-" in piece:
            start, end = piece.split("-", 1)
            numbers.extend(range(int(start), int(end) + 1))
        else:
            numbers.append(int(piece))
    return sorted(set(numbers))


PARK_POSITION_COUNT = _environment_value("PARK_POSITION_COUNT", int, 3000)
LOT_SCHEDULE_COUNT  = _environment_value("LOT_SCHEDULE_COUNT", int, 60000)
CASE_ROOT           = os.path.abspath(os.environ.get(
    "CASE_ROOT", os.path.join(SCRIPT_DIRECTORY, "..", "cases")))
INCLUDE_BASE_CASE   = _environment_value(
    "INCLUDE_BASE_CASE", lambda v: v != "0", True)
CASE_NUMBERS = parse_case_numbers(os.environ.get("CASE_NUMBERS", "1-5"))


def link_shared_map_files(case_directory):
    """공통 지도 파일 4종을 케이스 폴더에 링크로 걸어둔다.

    복사하면 케이스마다 6.7MB씩 늘어난다. 링크는 공간을 먹지 않고,
    원본이 한 곳이라 지도가 바뀌어도 전부 따라간다."""
    for name in SHARED_MAP_FILES:
        source = os.path.join(SCRIPT_DIRECTORY, name)
        target = os.path.join(case_directory, name)
        if os.path.lexists(target):
            os.remove(target)
        try:
            os.symlink(os.path.relpath(source, case_directory), target)
        except OSError:
            shutil.copy2(source, target)       # 링크를 못 만드는 환경이면 복사


def write_base_case():
    """s00 - 지금 쓰고 있는 파일을 그대로 복사한다. 새로 뽑지 않는다."""
    case_directory = os.path.join(CASE_ROOT, "s00")
    os.makedirs(case_directory, exist_ok=True)
    for name in CASE_FILES:
        shutil.copy2(os.path.join(SCRIPT_DIRECTORY, name),
                     os.path.join(case_directory, name))
    link_shared_map_files(case_directory)
    positions = json.load(open(os.path.join(case_directory, CASE_FILES[0])))
    schedule = json.load(open(os.path.join(case_directory, CASE_FILES[1])))
    write_manifest(case_directory, 0, 0, 1, len(positions), len(schedule),
                   note="원본 파일 복사. 새로 뽑지 않음(과거 실험 대조용)")
    print("s00  (현재 파일 복사)   시작위치 %d개 / 로트 %d건"
          % (len(positions), len(schedule)))


def write_manifest(case_directory, case_number, park_seed, lot_seed,
                   position_count, schedule_count, note=""):
    """이 케이스가 어떤 시드로 만들어졌는지 폴더 안에 남긴다.

    결과만 보고도 "어느 시드에서 난 데드락인지" 추적할 수 있어야 한다.
    유휴 지터 시드는 바꾸지 않는다. 그것까지 변하면 데드락의 원인을
    "시작 배치와 로트 조합" 으로 귀속시킬 수 없고, 같은 케이스를 다시
    돌렸을 때 똑같이 재현되지도 않는다.
    """
    manifest = {
        "case_name": "s%02d" % case_number,
        "case_number": case_number,
        "park_position_seed": park_seed,     # 시작 위치를 뽑은 난수 시드
        "lot_schedule_seed": lot_seed,       # 로트 사양을 뽑은 난수 시드
        "idle_jitter_seed": 2,               # 유휴 지터 - 전 케이스 공통 고정
        "park_position_count": position_count,
        "lot_schedule_count": schedule_count,
        "note": note,
    }
    with open(os.path.join(case_directory, MANIFEST_FILE), "w",
              encoding="utf-8") as handle:
        json.dump(manifest, handle, ensure_ascii=False, indent=1)


def write_case(case_number):
    """sNN - 시작위치와 로트 스케줄을 새로 뽑는다.

    시드는 기존에 쓰이던 0 / 1 / 2 와 겹치지 않게 100+n, 200+n 을 쓴다.
    """
    case_directory = os.path.join(CASE_ROOT, "s%02d" % case_number)
    os.makedirs(case_directory, exist_ok=True)

    position_count = min(PARK_POSITION_COUNT, len(simulator.NODE_LIST))
    position_random = random.Random(100 + case_number)
    positions = position_random.sample(simulator.NODE_LIST, position_count)
    with open(os.path.join(case_directory, CASE_FILES[0]), "w",
              encoding="utf-8") as handle:
        json.dump(positions, handle)

    schedule_random = random.Random(200 + case_number)
    schedule = [simulator._make_lot_entry(schedule_random)
                for _ in range(LOT_SCHEDULE_COUNT)]
    with open(os.path.join(case_directory, CASE_FILES[1]), "w",
              encoding="utf-8") as handle:
        json.dump(schedule, handle)

    link_shared_map_files(case_directory)
    write_manifest(case_directory, case_number, 100 + case_number,
                   200 + case_number, len(positions), len(schedule))
    urgent = sum(1 for entry in schedule if entry["priority"])
    print("s%02d  시드 %d/%d   시작위치 %d개 / 로트 %d건 (긴급 %d건, %.1f%%)"
          % (case_number, 100 + case_number, 200 + case_number,
             len(positions), len(schedule), urgent, urgent / len(schedule) * 100))


def main():
    os.makedirs(CASE_ROOT, exist_ok=True)
    print("케이스 폴더: %s" % CASE_ROOT)
    print("시작위치 %d개 / 로트 %d건 기준 (맵 노드 %d개)"
          % (PARK_POSITION_COUNT, LOT_SCHEDULE_COUNT, len(simulator.NODE_LIST)))
    print("")
    if INCLUDE_BASE_CASE:
        write_base_case()
    for case_number in CASE_NUMBERS:
        write_case(case_number)
    print("")
    print("돌릴 때는 CASE_DIRECTORY 로 폴더를 지정한다.")
    print("  CASE_DIRECTORY=%s/s01 OHT_COUNT=800 python3 deadlock_detector.py"
          % os.path.relpath(CASE_ROOT, SCRIPT_DIRECTORY))


if __name__ == "__main__":
    main()
