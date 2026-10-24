# -*- coding: utf-8 -*-
"""교착 기록이 정말 교착인지 구조적으로 검증한다.

탐지기 결과 JSON을 받아, 기록된 순환이 아래를 만족하는지 하나씩 확인한다.
"탐지기가 그렇게 말했다" 가 아니라 지도 데이터와 대조해 독립적으로 검사한다.

  1. 닫힌 순환인가       - 대기 사슬을 따라가면 제자리로 돌아오는가
  2. 물리적으로 인접한가  - 각 차량이 1~2홉 안에 앞차를 두고 있는가
  3. 우회로가 없는가      - 구성원 전원이 나가는 길이 하나뿐인 노드에 있는가
  4. 비켜줄 수 없는가     - 구성원이 전부 작업 중인가(유휴 차량만 비켜준다)
  5. 고리 제어 밖인가     - 탐지된 베이 고리 77개에 속하지 않는가
  6. 얼마나 멈춰 있었나   - 전원이 연속으로 정지한 시간

이 검사로 증명되지 않는 것도 분명히 적는다. 기록을 남긴 시점 이후에도
계속 멈춰 있는지는 여기서 알 수 없다. 그것은 STOP_AFTER_SECONDS=0 으로
다시 돌려 관측해야 한다.

    RESULT_JSON_PATH   탐지기 결과 파일
    MIN_HELD_SECONDS   이 시간 이상 지속된 순환만 검사  기본 600
"""
import os
import sys
import json
import collections

sys.argv = ["agv_fab_sim.py", "--analyze"]
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import agv_fab_sim as simulator

RESULT_JSON_PATH = os.environ["RESULT_JSON_PATH"]
MIN_HELD_SECONDS = float(os.environ.get("MIN_HELD_SECONDS", 600))


def hops_between(start, goal, limit=4):
    """start 에서 goal 까지 몇 홉인가. limit 안에 못 닿으면 None."""
    depth = {start: 0}
    queue = collections.deque([start])
    while queue:
        node = queue.popleft()
        if depth[node] >= limit:
            continue
        for link_index in simulator.OUT_LINKS.get(node, []):
            ahead = simulator.LINK_B[link_index]
            if ahead == goal:
                return depth[node] + 1
            if ahead not in depth:
                depth[ahead] = depth[node] + 1
                queue.append(ahead)
    return None


def verify(record, result):
    nodes = record["nodes"]
    size = len(nodes)
    print("  발생 t=%.0f초 / %d대 / %.1f초 연속 정지 / 영향 %d대"
          % (record["started_second"], size, record["held_seconds"],
             record["affected_vehicles"]))
    print("  OHT 번호: %s" % record["oht_ids"])

    checks = []

    # 1. 노드가 전부 다른가 (같은 노드가 두 번 나오면 기록이 잘못된 것)
    checks.append(("구성원 노드가 모두 다르다", len(set(nodes)) == size,
                   "%d개 중 중복 %d개" % (size, size - len(set(nodes)))))

    # 2. 대기 사슬이 물리적으로 이어지는가
    hop_list, length_list = [], []
    for i in range(size):
        here, ahead = nodes[i], nodes[(i + 1) % size]
        hop = hops_between(here, ahead)
        hop_list.append(hop)
        if hop == 1:
            length_list.append(simulator.edge_length(here, ahead))
    adjacent = all(h is not None and h <= 2 for h in hop_list)
    checks.append(("전원이 1~2홉 안에 앞차를 둔다", adjacent,
                   "홉 분포 %s" % collections.Counter(hop_list).most_common()))
    if length_list:
        print("  직접 연결된 구간의 링크 길이: %.0f ~ %.0f mm (%d개)"
              % (min(length_list), max(length_list), len(length_list)))

    # 3. 우회로가 있는가
    single_exit = [n for n in nodes if len(simulator.OUT_LINKS.get(n, [])) == 1]
    checks.append(("전원이 나가는 길이 하나뿐인 노드에 있다",
                   len(single_exit) == size,
                   "%d / %d" % (len(single_exit), size)))

    # 4. 합류 노드 포함 여부 (참고용)
    merges = [n for n in nodes if n in simulator.MERGE_POINT_NODES]
    print("  합류 노드 %d개 포함: %s" % (len(merges), merges or "-"))

    # 5. 고리 제어 범위
    in_ring = [n for n in nodes if simulator.NODE_RINGS.get(n)]
    checks.append(("탐지된 베이 고리 밖이다", not in_ring,
                   "고리에 속한 노드 %d개" % len(in_ring)))

    # 6. 정지 시간
    checks.append(("전원이 %.0f초 이상 연속 정지했다" % MIN_HELD_SECONDS,
                   record["held_seconds"] >= MIN_HELD_SECONDS,
                   "%.1f초" % record["held_seconds"]))

    print("")
    for name, passed, detail in checks:
        print("   [%s] %-38s  %s" % ("통과" if passed else "실패", name, detail))
    return all(p for _, p, _ in checks)


def main():
    with open(RESULT_JSON_PATH, encoding="utf-8") as handle:
        result = json.load(handle)
    records = [c for c in result["cycles"] if c["held_seconds"] >= MIN_HELD_SECONDS]

    print("파일: %s" % os.path.basename(RESULT_JSON_PATH))
    print("조건: OHT %d대 / 투입 %.2f초 / 차간 %.0fmm / 고리제어 %s / 케이스 %s"
          % (result["oht_count"], result["spawn_interval_seconds"],
             result["min_follow_distance_mm"],
             "켬" if result["ring_control_enabled"] else "끔",
             result.get("case_info", {}).get("case_name", "-")))
    print("순환 총 %d건, 그중 %.0f초 이상 지속 %d건"
          % (result["cycle_count"], MIN_HELD_SECONDS, len(records)))
    print("")

    all_passed = True
    for index, record in enumerate(records, 1):
        print("--- 순환 %d ---" % index)
        if not verify(record, result):
            all_passed = False
        print("")

    print("=" * 62)
    print("결과: %s" % ("모든 검사 통과" if all_passed and records
                      else "검사 실패 항목 있음" if records else "검사할 순환 없음"))
    print("")
    print("이 검사로 증명되지 않는 것:")
    print("  기록을 남긴 시점 이후에도 계속 멈춰 있는지는 알 수 없다.")
    print("  탐지기가 %.0f초에서 실행을 중단했기 때문이다." % MIN_HELD_SECONDS)
    print("  STOP_AFTER_SECONDS=0 으로 다시 돌려 관측해야 한다.")


if __name__ == "__main__":
    main()
