# -*- coding: utf-8 -*-
"""교착이 발생한 구간을 그림으로 그린다.

탐지기가 남긴 결과 JSON에서 순환 기록을 읽어, 그 노드들 주변만 잘라
레일과 차량 위치, 그리고 "누가 누구를 기다리는지" 를 화살표로 그린다.
시뮬레이터 전체 지도(노드 6,983개)는 너무 커서 교착 구간이 안 보이므로
해당 부분만 확대한다.

    RESULT_JSON_PATH   탐지기 결과 파일
    OUTPUT_PNG_PATH    저장할 그림 경로
    MIN_HELD_SECONDS   이 시간 이상 지속된 순환만 그린다  기본 600
"""
import os
import sys
import json
import collections

sys.argv = ["agv_fab_sim.py", "--analyze"]
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["font.family"] = "Noto Sans CJK JP"
matplotlib.rcParams["axes.unicode_minus"] = False
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
import agv_fab_sim as simulator

RESULT_JSON_PATH = os.environ["RESULT_JSON_PATH"]
OUTPUT_PNG_PATH  = os.environ["OUTPUT_PNG_PATH"]
MIN_HELD_SECONDS = float(os.environ.get("MIN_HELD_SECONDS", 600))


def shortest_hops(start, goal, limit=4):
    """start 에서 goal 까지의 링크 경로(최대 limit 홉)."""
    previous = {start: None}
    queue = collections.deque([start])
    while queue:
        node = queue.popleft()
        depth = 0
        probe = node
        while previous[probe] is not None:
            probe = previous[probe]
            depth += 1
        if depth >= limit:
            continue
        for link_index in simulator.OUT_LINKS.get(node, []):
            next_node = simulator.LINK_B[link_index]
            if next_node == goal:
                path = [goal, node]
                probe = node
                while previous[probe] is not None:
                    probe = previous[probe]
                    path.append(probe)
                return list(reversed(path))
            if next_node not in previous:
                previous[next_node] = node
                queue.append(next_node)
    return [start, goal]


def draw(record, result, axis):
    cycle_nodes = record["nodes"]
    coordinates = [simulator.NODE_XY[n] for n in cycle_nodes]
    xs = [c[0] for c in coordinates]
    ys = [c[1] for c in coordinates]
    margin = max(4000.0, (max(xs) - min(xs) + max(ys) - min(ys)) * 0.18)
    x0, x1 = min(xs) - margin, max(xs) + margin
    y0, y1 = min(ys) - margin, max(ys) + margin

    # 주변 레일을 옅게 깔아 위치 감을 준다
    for link_index in range(simulator.NUM_LINKS):
        a, b = simulator.LINK_A[link_index], simulator.LINK_B[link_index]
        ax_, ay_ = simulator.NODE_XY[a]
        bx_, by_ = simulator.NODE_XY[b]
        if not (x0 <= ax_ <= x1 and y0 <= ay_ <= y1):
            continue
        axis.plot([ax_, bx_], [ay_, by_], color="#d8dde4", linewidth=1.4,
                  zorder=1, solid_capstyle="round")

    # 대기 사슬: i 번 차가 i+1 번 차를 기다린다
    in_cycle = set(cycle_nodes)
    for i, node in enumerate(cycle_nodes):
        ahead = cycle_nodes[(i + 1) % len(cycle_nodes)]
        path = shortest_hops(node, ahead)
        for j in range(len(path) - 1):
            ax_, ay_ = simulator.NODE_XY[path[j]]
            bx_, by_ = simulator.NODE_XY[path[j + 1]]
            axis.add_patch(FancyArrowPatch(
                (ax_, ay_), (bx_, by_), arrowstyle="-|>", mutation_scale=13,
                color="#d4322c", linewidth=2.4, zorder=3,
                shrinkA=9 if path[j] in in_cycle else 2,
                shrinkB=9 if path[j + 1] in in_cycle else 2))

    # 차량
    merge_nodes = simulator.MERGE_POINT_NODES
    for i, node in enumerate(cycle_nodes):
        cx, cy = simulator.NODE_XY[node]
        is_merge = node in merge_nodes
        axis.scatter([cx], [cy], s=330,
                     facecolor="#8a1f1a" if is_merge else "#d4322c",
                     edgecolor="white", linewidth=1.6, zorder=4)
        axis.annotate(str(record["oht_ids"][i]), (cx, cy), color="white",
                      fontsize=7.5, ha="center", va="center", zorder=5,
                      fontweight="bold")

    axis.set_aspect("equal")
    axis.set_xlim(x0, x1)
    axis.set_ylim(y1, y0)          # 화면 좌표와 같게 위아래 뒤집는다
    axis.set_xticks([]); axis.set_yticks([])
    for spine in axis.spines.values():
        spine.set_color("#c9ced6")

    where = []
    if record["bay_ids"]:
        where.append("베이 " + ", ".join(str(b) for b in record["bay_ids"]))
    if record["corridor_ids"]:
        where.append("중앙통로 " + ", ".join(str(c) for c in record["corridor_ids"]))
    if record["merge_ids"]:
        where.append("합류부 " + ", ".join(str(m) for m in record["merge_ids"][:4]))
    axis.set_title(
        "OHT %d대 / 투입 0.45초 — 교착 %d대, %.0f초 정지\n%s   (영향 %d대)"
        % (result["oht_count"], record["cycle_size"], record["held_seconds"],
           " · ".join(where), record["affected_vehicles"]),
        fontsize=11, pad=12)


def main():
    with open(RESULT_JSON_PATH, encoding="utf-8") as handle:
        result = json.load(handle)
    records = [c for c in result["cycles"]
               if c["held_seconds"] >= MIN_HELD_SECONDS]
    if not records:
        print("그릴 순환이 없다 (%.0f초 이상 지속된 것 없음)" % MIN_HELD_SECONDS)
        return
    figure, axes = plt.subplots(1, len(records),
                                figsize=(7.5 * len(records), 7.0))
    if len(records) == 1:
        axes = [axes]
    for axis, record in zip(axes, records):
        draw(record, result, axis)
    figure.text(0.5, 0.015,
                "빨간 화살표 = 기다리는 방향(나 → 내 앞을 막은 차) · "
                "진한 점 = 합류 노드 · 숫자 = OHT 번호 · 회색 선 = 레일",
                ha="center", fontsize=9, color="#555")
    figure.tight_layout(rect=(0, 0.04, 1, 1))
    figure.savefig(OUTPUT_PNG_PATH, dpi=165, facecolor="white")
    print("저장: %s  (순환 %d개)" % (OUTPUT_PNG_PATH, len(records)))


if __name__ == "__main__":
    main()
