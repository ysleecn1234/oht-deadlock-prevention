"""
agv_fab_sim.py
=================================================================
[2026-08 맵 교체] 기존 FabLayout.txt(11,673 노드 / 12,397 엣지) 기반에서
MapEditor 포맷 신규 맵 Map_LargeFab_CCW_v1_2.txt(주행노드 6,983 /
주행링크 7,785 / PORT 19,960)로 전면 교체했다. 필드 정의는
"Map 구성 요소 D&D Rev7.1" 문서를 따른다(MAP_TERMS.md 참고).

실제 팹 트랙에서 OHT(OHT)가 Dijkstra 라우팅 + 노드 예약(lock)
충돌회피로 주행하며, JOB 파일의 실측 작업(STB <-> EQ 직송)을
수행하는 salabim 시뮬레이션.

입력 파일 (같은 폴더에 둘 것 - 전부 build_map_data.py가 생성):
    agv_graph.json       - OHT 주행 방향그래프
                           {"nodes":{id:[x,y]}, "edges":[[a,b],...],
                            "links":[{id,a,b,type,length,curve,speed},...]}
    track_segments.json  - 렌더링용 레일 폴리라인 (y-up)
    equipment.json       - PORT 목록 (EQ/STK/STB/CONV/MLUD/TL, node, io)
    real_job_routes.json - JOB 파일의 (From port -> To port) 집계
    section_regions.json - 베이/중앙통로/합류부 구역 분류

    원본:
        Map_LargeFab_CCW_v1_2.txt  (맵)
        JOB_LargeFab_CCW_v1_1.txt  (작업 5만 건, From port / To port)
        Map_구성_요소_D_D_Rev7_1.xlsx (필드 정의서 = 용어정리)

    갱신 방법:
        python build_map_data.py --map Map_LargeFab_CCW_v1_2.txt \
                                 --job JOB_LargeFab_CCW_v1_1.txt

좌표계: 신규 맵 원본은 y-down(도면 위쪽 Y=-12,860, 아래쪽 Y=-112,760)이다.
        build_map_data.py가 굽는 시점에 x'=x-x_min, y'=y-y_min 평행이동을
        한 번만 걸어 전부 양수 y-up으로 바꿔둔다. 따라서 이 파일 안에서는
        예전과 똑같이 "데이터는 이미 y-up"을 전제로 동작한다.
        PIL은 항상 row0=top 이므로 배경을 구울 때만 1회 뒤집는다.

이적재노드(Type=T): Rev3부터 이적재노드는 주행그래프의 노드가 아니라
        "주행링크 위의 한 점"(ParentLinkID + RelativeDistance)으로 정의된다.
        그래서 주행그래프에는 주행노드(Type=G)만 넣고, PORT는 자기
        이적재노드가 올라탄 링크의 EndNode를 접근노드(equipment.json의
        "node")로 갖는다. 링크 1개 길이(중앙값 1.7m) 이내 오차이며,
        PORT 19,960개가 접근노드 3,432개를 공유한다.

라우팅: scipy.sparse.csgraph.dijkstra 로 노드 6,983개짜리 그래프에서
        구간마다 최단경로를 계산한다(구간당 한 번, 프레임마다 아님).
        엣지 가중치는 유클리드 거리가 아니라 맵의 Link.Length(실측
        주행거리 - 특히 커브 구간에서 직선거리와 크게 다르다)를 쓴다.

충돌회피: 노드 단위 예약(lock). OHT는 다음 노드가 비어있을 때만 진입하고,
        진입 즉시 이전 노드 lock을 해제한다(락 미해제로 인한 데드락 방지).
        [2500대 대응] 락이 막혀있을 때 0.2초 busy-poll 하던 방식을 폐기하고,
        release() 시점에 대기 큐(NODE_WAITERS)의 첫 OHT만 깨우는 event-driven
        방식으로 전환했다(엔진 이벤트 폭주 방지). follow-distance 체크(앞차와의
        최소 거리)는 노드 단위 락과 무관하게 여러 노드를 동시에 봐야 하므로
        이 부분만 짧은 재시도(hold)를 유지한다.

자재흐름: MAX_OHT 대수의 OHT를 시작 시 한 번만 생성하는 "영구 풀" 방식.
        Manager가 SPAWN_INTERVAL마다 새 로트 하나를 만든다.
        자재는 로트 단위로 움직인다: STK -> [STB] -> EQ(가공, 딱 1번) ->
        [STB] -> STK. 로트 내용(패턴/포트/긴급도)은 oht_lot_schedule.json에
        미리 고정돼 있어(JOB_SCHEDULE), 같은 조건으로 다시 돌리면 완전히
        같은 순서로 재현된다. 로트 안의 각 구간(leg)은 매번 별도 Job으로
        배정 경쟁에 부쳐지며, 완료 후 OHT는 다시 유휴 풀로 복귀한다
        (소멸하지 않음). 자세한 패턴 비율/EQ 가공 처리는 아래
        LOT_PATTERN_WEIGHTS와 build_lot_legs() 참고.

시각화: 설비(EQ) 상태색상(empty/reserved/processing/done), OHT 상태색상
        (idle/empty/loaded), 화면 하단 통계 텍스트 + 범례.
        [2500대 대응] EQ 원의 fillcolor를 매 프레임 재평가하는 람다 대신,
        상태가 실제로 바뀌는 시점에만 값을 직접 갱신하도록 변경했다
        (tkinter 캔버스 아이템 수천 개가 매 프레임 색상 함수를 호출하는
        비용이 2500대 규모에서 렉의 주 원인이었음).
        [줌/팬] 마우스 휠/키보드 +-(확대·축소), 오른쪽 버튼 드래그(이동),
        R(리셋). main()/replay_main()이 Viewport 클래스를 공유해서 쓴다.
        [한글 폰트] 기본 DejaVuSans는 한글 글리프가 없어 통계/범례 텍스트가
        네모(□)로 깨져 보였다. _load_font()가 malgun.ttf(윈도우 맑은 고딕),
        NanumGothic(리눅스), DejaVuSans 순서로 시도하도록 수정해서 해결했다.
        이 폰트 로딩은 모듈 로드 시(HEADLESS가 아닐 때) 딱 한 번만 실행되고,
        FONT_STATS/FONT_LEGEND 전역 변수에 담겨 매 프레임 재사용된다
        (프레임마다 폰트 파일을 다시 여는 건 낭비이므로 render_frame()/
        render_replay_frame() 안에서는 절대 이 함수를 다시 호출하지 않는다).
        [Idle 설비 표시 방식] SHOW_EQ_IDLE 토글로 두 방식 중 선택 가능하다.
        True(기본값)면 Idle 설비를 EQ_IDLE_ALPHA 투명도의 반투명 원으로
        표시해 레일 색과 설비 위치가 동시에 보이게 하고, False면 Idle
        설비는 아예 그리지 않고 실제로 진행 중인(예약/가공/완료) 설비만
        표시해 레일 색이 최대한 또렷하게 보이도록 한다.

재생(replay) 모드: `--replay` 플래그를 주면 위 생성형 시뮬레이션 전체를
        건너뛰고, extract_oht_replay.py로 OHT_log_only.log에서 미리 뽑아둔
        실제 OHT 위치 기록(REPLAY_JSON, 기본 oht_replay_trajectories.json)을
        시간 순서 그대로 재생한다. Dijkstra 라우팅도 물리모델도 노드락도
        전혀 쓰지 않는다 - 그날 실제로 있었던 그대로를 화면에 옮길 뿐이라,
        이 모드에서는 OHT 대수를 바꿔서 실험하는 게 불가능하다(로그에
        정확히 그 대수가 정확히 그렇게 움직인 기록만 있어서). "몇 대
        필요한가"는 --analyze/main() 쪽 질문이고, "그날 실제로 뭐가
        있었나"는 --replay 쪽 질문이다.
        사용법: python oht_fab_sim.py --replay [--speed 10] [--start 300]
        조작은 위와 동일 + Space(일시정지/재개).
        [주의 - 맵 교체 영향] oht_replay_trajectories.json은 구 맵
        (FabLayout.txt) 노드 ID로 기록된 데이터라, 신규 맵의 노드 ID
        체계와 맞지 않는다. replay_main()이 로드 직후 노드 ID 일치율을
        검사해서 낮으면 경고를 띄운다. 신규 맵으로 재생하려면
        extract_oht_replay.py를 신규 맵 기준으로 다시 돌려야 한다.
=================================================================
"""

import argparse
import bisect
import csv
import heapq
import json
import math
import os
import pickle
import random
import statistics
import sys
import time
from collections import Counter

import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra
from PIL import Image, ImageDraw, ImageFont, ImageColor

import salabim as sim
import pygame

# --analyze(용량 산정 headless 분석)일 때는 PIL 배경 굽기/pygame 창을 전부
# 생략한다. 배경 굽기(레일 12,397개)가 매 회차마다 반복되면 낭비이므로,
# 이 플래그는 모듈 최상단에서 미리 정해 아래 로딩 단계에서부터 분기한다.
HEADLESS = "--analyze" in sys.argv

# ----------------------------------------------------------------------
# 설정값
# ----------------------------------------------------------------------
# [맵 교체] 아래 5개는 전부 build_map_data.py가 Map_LargeFab_CCW_v1_2.txt와
# JOB_LargeFab_CCW_v1_1.txt에서 구워내는 파일이다. 맵이 또 바뀌면 이 파일들을
# 손으로 고치지 말고 build_map_data.py를 다시 돌리면 된다.
TRACK_JSON = "track_segments.json"
GRAPH_JSON = "agv_graph.json"
EQUIP_JSON = "equipment.json"
SECTION_REGIONS_JSON = "section_regions.json"  # 베이/중앙통로/합류부 구역 분류
    # (구 버전은 사용자 엑셀 기반이었지만, 신규 맵에는 Node.ZoneID가 비어
    #  있어서 build_map_data.py가 트랙 기하로 자동 분류한다.)

# [정리] 실측 real_job_routes.json 기반 직송 모델은 완전히 폐기됐다.
# 지금은 아래 LOT_PATTERN_WEIGHTS 기반 로트 모델만 쓴다.

# [1번 요청사항] From-To 범주별 비율. EQ<->EQ/EQ<->STB/STB<->EQ 세 범주만
# 쓴다(STK는 이 비율에 없음 - "처음 자재를 받을 때"와 "작업 완료 시"에만
# 관여한다는 게 사용자 확인 내용이라, 5번 요청사항(작업 완료 로직)에서
# 다룬다). 범주 안에서 From/To 포트는 EQ_PORTS/STB_PORTS 안에서 완전
# 균등 무작위로 뽑는다(포트별 빈도 가중치 없음 - 사용자 확인).
# [STB의 정의 - 사용자 확인] 여기서 "STB"는 순수 버퍼(STB_PORTS, kind가
# STB/UTB인 것)만 가리킨다. 스토커(STK)는 포함하지 않는다 - STB는 단순
# 버퍼 역할이고, processing time이 실제로 걸리는(=작업이 이뤄지는) 곳은
# EQ뿐이라는 게 사용자 설명이다.
# [5번 요청사항] 자재는 이제 단발성 job이 아니라 "로트" 단위로 움직인다.
#   STK -> [STB] -> EQ(가공, 딱 1번) -> [STB] -> STK
# EQ는 항상 정확히 1번 방문(가공이 실제로 일어나는 곳은 EQ뿐이라는 사용자
# 설명). STB는 생산 전/후/둘다/전혀없음 중 하나인데, 사용자가 "둘다"
# 패턴은 빼고 아래 세 패턴 비율이 1번 요청사항의 EQ-EQ/EQ-STB/STB-EQ
# 비율(10/50/40)을 그대로 만족하도록 정했다:
#   NONE   (STB 없음, STK->EQ->STK)         : 10%  (1번의 EQ_EQ에 대응)
#   AFTER  (STB가 가공 후, STK->EQ->STB->STK): 50%  (1번의 EQ_STB에 대응 -
#                                              가공 후 EQ->STB로 나가는 leg)
#   BEFORE (STB가 가공 전, STK->STB->EQ->STK): 40%  (1번의 STB_EQ에 대응 -
#                                              STB->EQ로 들어가는 leg)
LOT_PATTERN_WEIGHTS = {
    "NONE": 0.10,
    "AFTER": 0.50,
    "BEFORE": 0.40,
}

# [1번 요청사항] 작업 긴급도(W_priority 입력값) 생성 분포 - 사용자 확인.
# 대부분(90%)은 0이고, 나머지 10%만 30~50 사이 무작위값을 받는다.
JOB_PRIORITY_ZERO_PROB = 0.9    # 이 확률로 priority=0
JOB_PRIORITY_RANGE = (30.0, 50.0)  # 나머지 확률에서 이 범위 균등 무작위

# [정리] 실측 투입 간격 재현 기능(real_job_interarrival_gaps.json 기반)도
# 폐기됐다 - 신규 JOB 파일엔 애초에 타임스탬프가 없어 파일 자체가 존재한
# 적이 없다. 로트 생성은 항상 SPAWN_INTERVAL 고정 간격이다.

SCALE = 0.0045          # 도면 단위(mm) -> 화면 픽셀 배율. 신규 맵 도면이
                        # 321,900 x 99,900mm 라 이 배율에서 1,478 x 480px.
MARGIN = 30

MAX_OHT = 1000
SPAWN_INTERVAL = 0.5          # 초, 신규 자재 투입 간격
                              # [2번 요청사항] 10초 배정 주기마다 20건 생성
                              # (10/0.5 = 20건/주기, 사용자 확정값).
                              # [실측 참고] MAX_OHT=1000 기준으로는 PENDING이
                              # 거의 안 쌓이고 가동률 28%에 그쳤다(유휴 44%).
                              # Little's Law 역산상 목표가동률 82.5%엔
                              # ~69건/10초가 필요하지만, 사용자가 대수와
                              # 무관하게 이 값으로 확정했다.
PROCESS_TIME = 30           # 초, 설비에서 처리(가공) 시간. 아래 OHT_LOAD_UNLOAD_TIME
                            # (OHT가 포트에서 자재를 싣고내리는 시간)과는 별개 개념.

UNIT_PER_METER = 1000.0     # 신규 맵의 좌표/Length 단위는 Rev7.1 정의상 mm.

# --- H/W Spec (Simulation용) ---
# 아래 속도/가감속/치수/차간거리/Load-Unload 시간은 전부 공식 H/W Spec
# 문서 값을 그대로 반영한다(이전에는 udp_logs 실측 분석으로 보정한 값을
# 썼으나, 이제 공식 스펙이 주어졌으므로 그 값으로 교체).
#   직선주행 최고 속도 : H/W 스펙상 8 m/s, "Simulation 반영" 값은 3.5 m/s
#                        -> 실제 차량 최고 성능이 아니라 시뮬레이션에 쓰라고
#                        지정된 값(3.5)을 그대로 쓴다.
#   곡선주행 최고 속도 : H/W 스펙상 1 m/s, "Simulation 반영" 값은 0.7 m/s
#   가속도/감속도      : 2 / 3 m/s^2 (H/W 스펙과 Simulation 반영값이 동일)
OHT_SPEED_MPS = 3.5         # 직선 최고속도 (m/s) - H/W Spec의 "Simulation 반영" 값
OHT_CURVE_SPEED_MPS = 0.7   # 커브 통과속도 (m/s) - H/W Spec의 "Simulation 반영" 값
OHT_ACCEL_MPS2 = 2.0        # 가속도 (m/s^2) - H/W Spec
OHT_DECEL_MPS2 = 3.0        # 감속도 (m/s^2) - H/W Spec

# --- OHT 차체 규격 (H/W Spec) ---
OHT_LENGTH_MM = 850.0       # OHT 길이(mm, 주행방향 치수) - H/W Spec
OHT_WIDTH_MM = 450.0        # OHT 폭(mm, 주행방향에 직각인 치수) - H/W Spec
OHT_STOP_GAP_MM = 300.0     # 정지 시 OHT 간(범퍼-범퍼 기준) 최소 거리(mm) - H/W Spec

# [차간 최소거리(follow-distance)] [사용자 확정] H/W Spec의 정지 시 OHT 간
# 최소 이격거리(OHT_STOP_GAP_MM)만 그대로 쓴다. 시뮬레이션이 OHT를 트랙
# 위의 "점"(노드 위치)으로 취급해서, 차체 길이(OHT_LENGTH_MM)까지 더하지
# 않으면 점 모델에서는 850mm짜리 차체 2대가 이론상 겹칠 수 있는 여지가
# 있다는 점은 참고로 남겨둔다 - 다만 이 값으로 가는 것으로 확정됨.
#
# [수정] 아래 RELEASE_PREV_AT_SAFE_DISTANCE(주행 중 출발 노드 조기 반납)를
# 켜면 뒤차가 이 거리만큼만 떨어져도 출발 노드로 진입한다. 그러면 위에서
# "이론상"이라고 남겨둔 차체 겹침이 상시 발생한다(간격 300mm에 차체 850mm
# -> 범퍼 기준 -550mm). 그래서 차체 길이를 포함한 값으로 올린다.
# 원문 값으로 되돌리려면 OHT_STOP_GAP_MM / 1000.0 으로 바꾸면 된다.
MIN_FOLLOW_DISTANCE_M = (OHT_LENGTH_MM + OHT_STOP_GAP_MM) / 1000.0  # = 1.15m

# --- Load/Unload 시간 (H/W Spec) ---
OHT_LOAD_UNLOAD_TIME = 8.0  # 초. OHT가 포트(버퍼/설비)에서 자재를 싣거나
                            # 내리는 데 걸리는 시간. 설비 자체의 가공시간
                            # (PROCESS_TIME)과는 별개이며, 자재를 픽업/드롭
                            # 할 때마다(포트 도착 시) 한 번씩 발생한다.

OHT_MAX_SPEED = OHT_SPEED_MPS * UNIT_PER_METER    # 도면단위 / 초
OHT_CURVE_SPEED = OHT_CURVE_SPEED_MPS * UNIT_PER_METER
OHT_ACCEL = OHT_ACCEL_MPS2 * UNIT_PER_METER       # 도면단위 / 초^2
OHT_DECEL = OHT_DECEL_MPS2 * UNIT_PER_METER
MIN_FOLLOW_DISTANCE = MIN_FOLLOW_DISTANCE_M * UNIT_PER_METER

# [차간거리 검사 범위 수정] 아래 _path_clear_ahead()의 lookahead 루프는 원래
# 링크 길이를 "먼저 더하고" 그 끝 노드의 점유를 검사했다. 그래서 기준 거리가
# 300mm여도 링크 길이(중앙값 1,700mm)만큼 건너뛴 뒤의 노드까지 비어 있어야
# 전진할 수 있었다 - 즉 선언된 300mm가 아니라 "다음 링크 하나를 통째로"
# 보는 것과 같았다. True이면 기준 거리를 넘어서는 노드는 검사 대상에서
# 제외한다(선언된 값대로 동작). False이면 기존(원문) 동작을 그대로 쓴다.
# 팀원 브랜치(dongwoo/distance-fixed)와 같은 이름을 쓴다.
FOLLOW_DISTANCE_CHECK_BEFORE_OCCUPANCY = True

# [주행 중 출발 노드 조기 반납] 원문은 링크를 다 건너 도착한 뒤에야 출발
# 노드의 락을 놓았다(_travel_path 끝의 release(prev_node)). 그래서 주행 중인
# OHT 한 대가 항상 노드 2칸(출발+도착)을 점유했고, 뒤차는 앞차가 완전히
# 도착할 때까지 출발조차 못 했다. 3.5m/s, 링크 1,700mm 기준으로 0.49초를
# 통째로 기다린다. 무리가 붙어서 함께 움직이는 tailgate group이 구조적으로
# 생길 수 없었다.
# True이면 안전거리(MIN_FOLLOW_DISTANCE)만큼 전진한 시점에 출발 노드를
# 반납해서, 주행 중 점유를 1칸으로 줄이고 뒤차가 바로 따라붙게 한다.
RELEASE_PREV_AT_SAFE_DISTANCE = True

# [막힘 정지 시 사전감속] 원문은 앞이 막혀 설 때 self._cur_speed = 0.0 으로
# 속도를 즉시 0으로 대입한다. H/W Spec 감속도(3.0 m/s^2) 기준으로 3.5m/s
# 에서 정지까지 2,042mm, 약 1.17초가 필요한데 그 거리와 시간이 모델에
# 없다. 원문에도 같은 목적의 기구가 있지만(_reroute_stop_limit), 재탐색
# 때문에 서야 하는 분기점만 보고 앞차 때문에 서는 경우는 보지 않는다.
# True이면 제동거리 안에 다른 OHT가 점유한 노드가 있을 때 그 직전 노드에
# 설 수 있도록 미리 감속한다.
PREDECEL_FOR_BLOCKED_NODE = True

# [길 비켜주기 - 근본원인 탐색 범위] MIN_FOLLOW_DISTANCE와는 목적이 다르다.
# MIN_FOLLOW_DISTANCE는 "실제로 이만큼 떨어져야 주행 가능"이라는 물리적
# 안전거리(H/W Spec 기반)라, 이 값을 키우면 주행 물리 자체가 바뀐다(실측:
# 2.3m로 늘렸더니 대기시간 +26%, 정체 +24%). YIELD_SCAN_DISTANCE는 그와
# 별개로 "막혀 있을 때 근본 원인이 누구인지 더 멀리까지 찾아보는" 시야
# 범위다 - 물리적 차간거리는 그대로 두고, 비켜달라는 요청을 보낼 대상만
# 더 멀리서 찾는다.
YIELD_SCAN_DISTANCE_M = 10.0   # [사용자 확정] 링크 6~7개 수준
YIELD_SCAN_DISTANCE = YIELD_SCAN_DISTANCE_M * UNIT_PER_METER

# [맵 교체] 신규 맵은 링크 종류(S/L/R/U/N)를 갖고 있어서 어느 구간이
# 커브인지 알 수 있다. True면 커브 링크(L/R/U) 진입 속도를
# OHT_CURVE_SPEED로 제한한다(구 맵에는 이 정보가 없어 전 구간 직선속도로
# 돌렸고, 그래서 커브가 많은 베이 진입부의 소요시간이 과소평가됐다).
USE_LINK_CURVE_SPEED = True

# [H/W Spec 반영] 삼각형 OHT 표시 크기를 임의 상수 대신 실제 차체 규격
# (OHT_LENGTH_MM 850 x OHT_WIDTH_MM 450, 위 "OHT 차체 규격" 참고)으로
# 정한다. heading=0일 때 앞(꼭짓점)이 +y, 뒤(밑변 두 점)가 -y를 향하도록
# 정의하고, 앞뒤 각각 길이의 절반(_TRI_HALF_LEN)만큼, 좌우 각각 폭의
# 절반(_TRI_HALF_WID)만큼 떨어뜨린다. 값이 바뀌지 않는 한 고정값이라
# 모듈 로드 시 한 번만 계산해둔다.
_TRI_HALF_LEN = OHT_LENGTH_MM * 0.5 * SCALE   # 길이(진행방향)의 절반
_TRI_HALF_WID = OHT_WIDTH_MM * 0.5 * SCALE    # 폭(진행방향 직각)의 절반
TRI_LOCAL_PTS = ((0.0, _TRI_HALF_LEN), (-_TRI_HALF_WID, -_TRI_HALF_LEN),
                  (_TRI_HALF_WID, -_TRI_HALF_LEN))

MAX_WINDOW_SCALE = 1.6      # 창 배율의 상한(화면이 아주 크면 이 이상은 안 키움)
SCREEN_FIT_MARGIN = 0.9     # 화면 해상도의 이 비율 안에 들어오도록 자동 맞춤
TARGET_FPS = 30             # pygame 화면 갱신 목표 fps
SIM_SPEED_FACTOR = 3.0      # 시뮬레이션 배속. 기존 salabim env.speed(3)과 같은 의미
                             # (실시간 1초당 시뮬레이션 시간 3초를 진행)

# --- 줌/팬(화면 확대·축소/이동) 설정 ---
# main()(생성형 시뮬레이션)과 replay_main()(실측 재생) 둘 다 같은 Viewport
# 클래스를 공유해서 쓴다(마우스 휠/드래그/+-/R 로직을 두 번 따로 관리하면
# 서로 어긋날 위험이 있어서).
# [축소 지원] zoom=1.0은 "창에 맞춘 기본 배율(fit_scale)"을 뜻한다. 예전엔
# ZOOM_MIN도 1.0이라 축소가 전혀 안 됐다(Viewport.visible_size()가 항상
# img_w/img_h로 clamp 돼 있어서, zoom을 1.0 밑으로 내려도 보이는 영역이
# 안 늘어났다 - 사실상 무효였음). ZOOM_MIN을 1.0 밑으로 낮추고
# visible_size()의 clamp를 제거해서, zoom<1일 때는 지도 전체가 창 안에서
# 더 작게(여백을 두고) 보이도록 했다(자세한 내용은 Viewport 클래스 참고).
ZOOM_MIN = 0.3
ZOOM_MAX = 25.0
ZOOM_STEP = 1.15

# --- OHT 번호 표시 ---
# [요청 반영] OHT별 운행률/병목률/위치 로그를 나중에 뽑아볼 수 있게, 각
# OHT는 생성 순서대로 0부터 oht_id를 갖는다(OHT.setup() 참고, STALL_EVENTS
# 에도 이 id로 기록됨). SHOW_OHT_ID=True면 화면에도 그 번호를 보여주되,
# OHT 수백~수천 대가 한 화면에 작게 몰려있을 땐 숫자가 다 겹쳐 오히려
# 안 보이므로 OHT_ID_LABEL_MIN_SCALE 이상으로 확대했을 때만 그린다.
# [토글] 이 초기값과 별개로, 실행 중 N 키로 언제든 켜고 끌 수 있다
# (toggle_show_oht_id() 참고, L/T 키와 동일한 방식).
SHOW_OHT_ID = True
OHT_ID_LABEL_MIN_SCALE = 3.0    # cam.scale(=fit_scale*zoom)이 이 값 이상일 때만 번호 표시
# [성능] OHT 위치는 매 프레임 바뀌므로 번호도 매 프레임 다시 그려야 해서
# (EQ/STK 포트처럼 캐시할 수 없음) PIL 텍스트 렌더링 자체가 대수가 많을 때
# 눈에 띄게 느려진다(1000대 다 보이는 화면에서 실측 시 프레임당 0.5초
# 가까이 소요). 화면에 동시에 보이는 라벨 수를 이 값으로 제한해서, 아무리
# 넓게/많이 겹쳐 보이는 화면이라도 프레임당 비용이 일정 이상 늘지 않게
# 막는다(화면 밖은 애초에 세지 않고, 화면 안에 보이는 것 중 처음 N개만).
OHT_ID_LABEL_MAX_COUNT = 150

# --- STK/EQ 포트 번호 표시 ---
# [요청 반영] equipment.json에 이미 "port_id"(예: "EQ54_1", "STK1_1")가 실제
# 설비/스토커 포트 이름으로 들어있으므로, OHT처럼 새로 번호를 매기지 않고
# 그 값을 그대로 보여준다(실제 설비 도면/DB와 바로 대조 가능). 포트 수가
# 훨씬 많아서(EQ 수천 개) OHT보다 더 확대했을 때만 켜지도록 임계값을
# 따로 둔다.
SHOW_PORT_ID = True
PORT_ID_LABEL_MIN_SCALE = 5.0

# --- 구역(베이/중앙통로) 색 틴트 표시 ---
# [요청 반영] 첨부 이미지처럼 베이(세로 구역)는 파란 계열, 중앙통로(가로
# 중심대)는 초록 계열의 반투명 색으로 깔아서 화면에서 바로 구분되게 한다.
# 기본은 꺼둔다(SHOW_REGION_TINT=False) - 레일 정체 색(초록/노랑/빨강)과
# 동시에 켜면 시각적으로 복잡해질 수 있어서, 필요할 때 T 키로 켜고 끄는
# 방식을 기본으로 한다.
SHOW_REGION_TINT = False

# [노드 점 표시] 모든 주행노드를 점으로 찍어 보여준다. 색 우선순위는:
#   1) 합류 지점 자체(MERGE_POINT_NODES, in-degree>1)   - 주황
#   2) 합류대기 진입측(MERGE_WAIT_NODES, 꺾여 들어오는 쪽) - 빨강
#   3) 우선(직진) 진입측(MERGE_PRIORITY_NODES)           - 파랑
#   4) N갈래 이상 분기점(out-degree, [ ] 키로 조절)       - 보라
#   5) 일반 노드                                          - 검정
# (agv_fab_sim_mini.py의 SHOW_NODE_DOTS 방식을 이 파일 구조에 맞게 옮긴
#  것. mini판은 합류부 앞뒤 역할(1~3)만 구분했고, 4는 4단계 재탐색이
#  재탐색 지점으로 삼는 JUNCTION_NODES(out-degree>=2)와 같은 기준을
#  추가로 얹은 것이다.)
SHOW_NODE_DOTS = False
SHOW_RING_OVERLAY = False   # [고리 진입 제어] G 키: 베이 고리 표시(파랑=여유/주황=거의 참/빨강=만석)
RING_OVERLAY_WARN = 0.75    # 점유/용량이 이 비율 이상이면 주황
# [실측] 이 맵(Map_LargeFab_CCW_v1_8.txt)은 전부 2갈래 포인트 전환기
# 구조라 out-degree 최대값이 2다(3갈래 이상 분기 자체가 없다). 그래서
# 기본값은 2 - JUNCTION_NODES(out-degree>=2)와 같은 집합이 표시된다.
# 나중에 더 복잡한 맵이 들어오면 [ ] 키로 3, 4...로 올려 실제 다갈래
# 분기만 골라 볼 수 있다.
JUNCTION_DOT_MIN_BRANCHES = 2   # 이 값 이상으로 나눠진 노드만 분기점 색으로
NODE_DOT_COLOR = (0, 0, 0)          # 일반 노드: 검은 점
JUNCTION_DOT_COLOR = (160, 32, 240)  # N개 이상 분기점(out-degree): 보라 점
MERGE_POINT_COLOR = (255, 100, 0)   # 합류 지점(허가권 노드, in-degree>1): 주황
MERGE_WAIT_COLOR = (220, 0, 0)      # 합류대기 노드(비우선 진입측): 빨강
MERGE_PRIORITY_COLOR = (0, 90, 220)  # 우선(직진) 진입측 노드: 파랑
NODE_DOT_R = 1.5                    # 일반 노드 점 반지름(아래 EQ_R과 동일)
JUNCTION_DOT_R = 1.5                # 분기/합류 점 반지름(크기는 같고 색만 구분)
REGION_BAY_COLOR = (70, 130, 220, 70)      # 베이: 파란 계열, 반투명(RGBA)
REGION_CENTER_COLOR = (60, 179, 90, 70)    # 중앙통로: 초록 계열, 반투명(RGBA)
REGION_MERGE_COLOR = (170, 40, 160, 110)   # 합류부: 보라 계열, 더 진하게(병목 강조 - 베이/
                                            # 중앙통로 색 위에 덧칠해도 눈에 띄도록 알파를 높임)
REGION_TINT_WIDTH = 5   # 틴트 띠의 기본 폭(ref-pixel 단위 - cam.scale 곱해서 화면 픽셀 폭 결정)
    # [폭 축소] 기존 10이었을 때, 같은 그룹 안에서 1000 도면단위 간격으로
    # 촘촘히 붙은 라인들(중앙통로/베이 모두 이런 구조)은 잘 이어붙었지만,
    # 그 그룹의 맨 바깥쪽 라인 기준으로는 선 폭의 절반(중심에서 ±폭/2)만큼
    # 실제 레일보다 훨씬 바깥까지 밴드가 삐져나가 보이는 문제가 있었다
    # (실측 스크린샷 대조 결과 화면상 ~36px씩 여백 - 사용자가 지적한
    # "범위가 넓게 잡힘"의 실제 원인). 5로 줄이면 1000 단위 간격 라인은
    # 여전히 끊김 없이 이어지면서(각 선이 자기 중심에서 ±2.5 ref-px씩
    # 칠해 이웃 선과 겹침), 그룹 가장 바깥쪽 라인의 삐져나온 여백은
    # 절반 수준(~18px)으로 줄어든다.
REGION_ID_LABEL_MIN_SCALE = 3.0  # 이 배율 이상으로 확대했을 때만 베이/중앙통로 번호 표시

# --- 실측 재생(replay) 모드 설정 ---
# extract_oht_replay.py로 OHT_log_only.log를 미리 파싱해둔 결과. oht_graph
# 등과 달리 이건 --replay 모드에서만 필요하므로, 아래서 항상 읽지 않고
# replay_main() 안에서 그때 가서 로드한다(생성형 모드/--analyze 실행할 때마다
# 240MB짜리 파일을 매번 읽는 낭비를 피하려고).
REPLAY_JSON = "oht_replay_trajectories.json"

# extract_eq_status.py로 JOB_log_only.log를 미리 파싱해둔 설비(EQ/STK) 상태
# 타임라인. "reserved"(OHT가 그 포트로 실어가려고 배정된 시각~픽업 완료
# 시각)와 "done"(다음 픽업 전까지)은 실측 타임스탬프 그대로다.
# "processing"(가공중, 빨강)만 예외 - 이 로그엔 실제 가공 구간 정보가
# 없어서, extract_eq_status.py가 하차 완료 시각부터 고정 길이(기본 1초,
# --processing-seconds로 조절)만큼 임의로 끼워넣은 값이다. 실측이 아니라는
# 점에 유의.
REPLAY_EQ_JSON = "oht_replay_eq_status.json"

RAIL_COLOR = (150, 150, 150)
RAIL_WIDTH = 2
STK_COLOR = (101, 67, 33)
STB_COLOR = (150, 130, 105)   # [맵 교체] 트랙 사이드 버퍼(STB). 스토커보다
                               # 연한 색으로 구분한다.
BG_COLOR = (245, 246, 248)

# [맵 교체] 신규 맵의 버퍼 포트는 STB 15,777개 + STK 54개다. 구 맵의
# 스토커 232개와 달리 개수가 두 자릿수 배라, 전부 그리면 정적 레이어를
# 다시 굽는 순간(줌/팬 할 때)마다 원 1.5만 개를 그리게 된다. 기본은 켜두되
# 화면 밖은 건너뛰고(컬링), 무거우면 SHOW_STB=False로 끄면 된다.
SHOW_STB = True
STB_R = 0.8   # STB 화면 표시 반경(px). 스토커(STK_R)보다 작게.

# --- 설비(EQ) 상태 색상 ---
MACHINE_STATUS_COLOR = {
    "empty":      "#1C6B08",   # 비어있음 (알파 채널은 tkinter 미지원이라 제거)
    "reserved":   "#26D8A6",   # 예약(OHT 배송중)
    "processing": "#FF0000",   # 가공중
    "done":       "#E29118",   # 가공완료(픽업 대기)
}
EQ_R = 1.5  # 화면 표시 반경(px)
assert NODE_DOT_R == EQ_R == JUNCTION_DOT_R, "노드 점 크기는 EQ 점과 같아야 함"
STK_R = 1.2   # 스토커(STK) 화면 표시 반경(px). render_frame()/render_replay_frame()에서
            # STK를 매 프레임 맨 위에 다시 그릴 때도 재사용하므로 전역 상수로 둔다.

# --- OHT 상태 색상 ---
IDLE_COLOR = "#5B65D6"     # 미배정(배회중) - 아직 job 없음
EMPTY_COLOR = "#00297A83"    # 빈 차(배정됨, 픽업하러 이동중)
LOADED_COLOR = "#AF29D8"   # 적재 차(스테이션 순회중)

# --- 미배정(Idle) OHT 로직 ---
# 실제 OCS는 Job이 없는 OHT에게 두 종류의 Route 명령을 내린다.
#   (1) 길 비켜주기 : 작업 중인 OHT의 진로를 막고 서 있으면 비켜준다.
#   (2) 존 분산     : Idle 차량이 한곳에 뭉치지 않도록 Zone마다 N등분한
#                     지점으로 흩뿌린다.
#
# [Zone 정의 - 사용자 확인] Zone = 베이 89개. 중앙통로는 간선 통로라
# Idle 차량이 거기 서 있으면 작업 차량을 막으므로 대기 금지 구역으로 둔다
# (이동 중 통과는 당연히 허용).
# [N등분] 각 베이를 세로 방향으로 IDLE_ZONE_DIVISIONS등분하고, 각 구간의
# 대표 노드 하나를 대기지점으로 삼는다. 베이가 N개로 못 쪼개질 만큼
# 작으면(노드 4개짜리 베이도 있다) 있는 만큼만 만든다.
# 실측상 2~4가 가장 좋았고(150대 기준 wait 9.1~9.6s), 1로 줄이면 13.2s,
# 8로 늘리면 11.0s로 나빠진다 - 너무 적으면 분산이 거칠고, 너무 많으면
# 대기지점이 촘촘해져 유휴 차량끼리 다시 간섭한다.
IDLE_ZONE_DIVISIONS = 4      # 베이 하나를 몇 등분할지
IDLE_REBALANCE_INTERVAL = 10.0   # 분산 재계산 주기(초)
IDLE_STANDBY_HOLD_MIN = 1.0  # 대기지점에 도착한 뒤 다음 재확인까지
IDLE_STANDBY_HOLD_MAX = 3.0
# [버그 수정] 비켜주기(_yield_step)가 갈 곳이 없어 실패했을 때 재시도까지
# 쉬는 시간. 이게 없으면 request_yield는 딱 한 번만 오고(노드락 대기측은
# 재시도가 없음), 그 한 번을 주변이 꽉 차서 놓치면 영원히 못 비켜줘서
# 뒤차가 무한 대기하는 교착상태(deadlock)가 생긴다.
YIELD_RETRY_INTERVAL = 1.0
# (구 명칭 호환 - 예전 배회 로직에서 쓰던 상수)
WANDER_HOLD_MIN = IDLE_STANDBY_HOLD_MIN
WANDER_HOLD_MAX = IDLE_STANDBY_HOLD_MAX

# --- 부가 시각화 on/off ---
SHOW_STATS_TEXT = True
OHT_DRAW_OUTLINE = False   # True면 삼각형에 검은 테두리 추가. 벤치마크상 outline
                           # 그리기 자체가 render_frame 시간의 ~35%를 차지해서
                           # 2500대 규모에선 기본 off. 삼각형이 화면상 몇 px
                           # 밖에 안 돼서 outline 유무 차이는 육안으로 크지 않음.

# --- 레일 정체(congestion) 시각화 설정 ---
# [요청 반영] 레일 하나하나(분기점~분기점 사이 구간)마다 그 위에 지금
# OHT가 얼마나 몰려있는지를 초록(널널)->노랑(50~80%)->빨강(80%+) 3단계로
# 색칠해 한눈에 병목 위치를 볼 수 있게 한다. "레일 하나"의 정의는 개별
# 노드/엣지가 아니라, agv_graph.json의 위상 구조에서 분기점(연결된 이웃이
# 정확히 2개가 아닌 노드 - 막다른 길=1개, 분기/합류=3개 이상)을 기준으로
# 나눈 사슬이다. 실제 분할/렌더링 코드는 아래 "레일 구간 분할" 섹션 참고.
SHOW_RAIL_CONGESTION = True
RAIL_CONGESTION_LOW = 0.5       # 이 비율 미만 -> 초록(널널)
RAIL_CONGESTION_HIGH = 0.8      # 이 비율 이상 -> 빨강(매우 막힘). 그 사이는 노랑
RAIL_CONGESTION_REFRESH_SEC = 1.0  # 정체 색상 재계산 주기(시뮬레이션 시간 기준,
                                    # 초). 정체는 초 단위로 서서히 바뀌는 지표라
                                    # 매 프레임(30fps) 다시 계산할 필요가 없어서
                                    # (EQ 상태 갱신과 같은 원칙) 이 주기마다만
                                    # 갱신해 성능을 아낀다.
RAIL_LINE_WIDTH_NORMAL = 1      # 널널(초록) 상태일 때 색선 폭 - 원래 레일(RAIL_WIDTH=2)
                                 # 보다 얇게 그려서, 밑에 깔린 회색 레일 윤곽이 살짝
                                 # 비치도록 한다(색선이 레일 전체를 뒤덮지 않게).
RAIL_LINE_WIDTH_CONGESTED = 1    # 정체(노랑/빨강) 레일도 널널 상태와 같은 1px로 통일.
                                  # 색 자체(초록/노랑/빨강)만으로 구분하고, 굵기 차이는 두지 않는다.


RAIL_COLOR_GREEN = (60, 179, 75)
RAIL_COLOR_YELLOW = (230, 200, 30)
RAIL_COLOR_RED = (220, 40, 40)


def congestion_color(ratio):
    """레일 점유율(0~1)을 초록(널널)->노랑(50~80%)->빨강(80%+) 그라데이션
    RGB 색상으로 변환한다. 임계값(RAIL_CONGESTION_LOW/HIGH)만 다른 상수로
    독립돼 있어 색 구간을 조정하고 싶으면 이 두 값만 바꾸면 된다."""
    ratio = max(0.0, min(ratio, 1.0))
    green, yellow, red = RAIL_COLOR_GREEN, RAIL_COLOR_YELLOW, RAIL_COLOR_RED
    if ratio < RAIL_CONGESTION_LOW:
        return green
    elif ratio < RAIL_CONGESTION_HIGH:
        t = (ratio - RAIL_CONGESTION_LOW) / (RAIL_CONGESTION_HIGH - RAIL_CONGESTION_LOW)
        return tuple(int(green[i] + (yellow[i] - green[i]) * t) for i in range(3))
    else:
        span = 1.0 - RAIL_CONGESTION_HIGH
        t = (ratio - RAIL_CONGESTION_HIGH) / span if span > 1e-9 else 1.0
        return tuple(int(yellow[i] + (red[i] - yellow[i]) * t) for i in range(3))


# ----------------------------------------------------------------------
# 데이터 로드
# ----------------------------------------------------------------------
with open(GRAPH_JSON, encoding="utf-8") as f:
    graph_data = json.load(f)

NODE_XY = {nid: tuple(xy) for nid, xy in graph_data["nodes"].items()}
EDGES = [tuple(e) for e in graph_data["edges"]]

# [맵 교체] 신규 맵은 링크마다 실측 주행거리(Length)와 종류(S/L/R/U/N)를
# 갖고 있다. 라우팅 가중치와 구간 이동거리는 유클리드 거리 대신 이 Length를
# 쓰고(특히 커브에서 직선거리와 최대 2.9m 차이), 커브 링크(L/R/U)에서는
# 최고속도를 OHT_CURVE_SPEED로 낮춰 실제 OHT 거동에 맞춘다.
EDGE_LENGTH = {}   # (a,b) -> 주행거리(mm)
EDGE_IS_CURVE = {}  # (a,b) -> 커브 여부
EDGE_SPEED = {}    # (a,b) -> 링크 지정 최고속도(mm/s) 또는 None
for _lk in graph_data.get("links", []):
    _key = (_lk["a"], _lk["b"])
    EDGE_LENGTH[_key] = float(_lk["length"])
    EDGE_IS_CURVE[_key] = bool(_lk.get("curve"))
    _sp = _lk.get("speed")
    EDGE_SPEED[_key] = float(_sp) if _sp else None


def edge_length(a, b):
    """a->b 링크의 주행거리. 그래프에 없는 조합(방어적)이면 직선거리."""
    d = EDGE_LENGTH.get((a, b))
    if d is not None:
        return d
    xa, ya = NODE_XY[a]
    xb, yb = NODE_XY[b]
    return math.hypot(xb - xa, yb - ya)


with open(TRACK_JSON, encoding="utf-8") as f:
    track_data = json.load(f)

SEGMENTS = track_data["segments"]
PAGE_W = track_data["page_size"]["width"]
PAGE_H = track_data["page_size"]["height"]

with open(EQUIP_JSON, encoding="utf-8") as f:
    EQUIPMENT = json.load(f)

# Rev7.1 Port.Type: EQ / STK / STB / UTB / TL / CONV (+ 현장맵 MLUD).
# EQ = 가공이 일어나는 곳 (로트당 정확히 1번 방문)
# STK = 자재의 출발/도착 지점 역할
# STB/UTB = "버퍼" 역할
# CONV / MLUD / TL = OTHER_PORTS로 분류만 되어 있고, 로트 흐름 로직에는 관여하지 않음
EQ_PORTS = [e for e in EQUIPMENT if e["kind"] == "EQ"]
STB_PORTS = [e for e in EQUIPMENT if e["kind"] in ("STB", "UTB")]
STK_PORTS = [e for e in EQUIPMENT if e["kind"] == "STK"]
OTHER_PORTS = [e for e in EQUIPMENT
               if e["kind"] in ("CONV", "MLUD", "TL")]
BUFFER_PORTS = STB_PORTS + STK_PORTS     # 자재 출발/도착 지점(설비가 아닌 곳) - 포트 라벨 렌더링용
EQ_NODE_SET = {e["node"] for e in EQ_PORTS}  # reserved/processing/done 색상은 EQ 노드에만

# 라우팅 warmup의 출발지 후보 = 모든 포트의 접근노드(중복 제거).
# PORT 19,960개가 접근노드 3,432개를 공유하므로, 포트 단위로 warmup하면
# 같은 노드를 6배 가까이 중복 계산하게 된다.
PORT_ACCESS_NODES = sorted({e["node"] for e in EQUIPMENT})

print(f"nodes={len(NODE_XY)} edges={len(EDGES)} "
      f"EQ={len(EQ_PORTS)} STB={len(STB_PORTS)} STK={len(STK_PORTS)} "
      f"기타(CONV/MLUD/TL)={len(OTHER_PORTS)} 접근노드={len(PORT_ACCESS_NODES)}")



# ----------------------------------------------------------------------
# 구역(Section/Bay/Corridor/Merge) 매핑 - 정체가 "어디서" 많이 발생하는지
# 구역 단위로 구분하기 위한 분류.
# [맵 교체] 구 버전은 사용자가 별도 엑셀(section_link_coordinates.xlsx)로
# 준 731개 섹션 정의를 그대로 썼다. 신규 맵에는 Node.ZoneID가 전부 비어
# 있고 대응하는 엑셀도 없어서, build_map_data.py가 트랙 기하만으로 아래
# 규칙에 따라 자동 분류한 결과를 굽는다:
#   섹션(Section)     : 분기점~분기점 사이 사슬(2,406개).
#   베이(Bay)         : 세로 링크를 X열로 묶고(허용오차 1.5m) 중앙통로
#                       밴드로 잘라낸 구간(89개). 왼쪽->오른쪽, 위->아래 순 번호.
#   중앙통로(Corridor): 가로 링크를 Y밴드로 묶은 간선 통로(5개). 위->아래 순 번호.
#   합류부(Merge)     : 진입차수 2 이상인 합류 노드 군집 중 "베이와
#                       중앙통로가 함께 닿는" 곳만(295개). 트랙 전체의
#                       합류점은 773곳이지만 대부분은 베이 내부의 짧은
#                       분기/재합류라 병목 지표로서 의미가 약해 제외했다.
# [커버리지] 6,983개 노드 중 베이 3,813 / 중앙통로 2,938 / 미분류 232개.
# 커버 안 되는 노드는 NODE_TO_SECTIONS에 키가 아예 없으므로, 조회하는 쪽
# (compute_region_congestion() 등)은 반드시 .get(node_id, []) 형태로
# 안전하게 처리하고 "미분류"로 집계한다.
# ----------------------------------------------------------------------
NODE_TO_SECTIONS = {}   # node_id -> [section_id, ...] (분기점 노드는 여러 섹션에 걸침)
SECTION_INFO = {}       # section_id(int) -> {"orientation","is_bay","is_center_bay",...}
BAY_SECTION_IDS = set()
CENTER_BAY_SECTION_IDS = set()
BAY_TINT_NODES = set()      # 화면 틴트용: 베이로 칠할 노드 전체(섹션 소속 + 갭 메우기 연결부)
CENTER_TINT_NODES = set()   # 화면 틴트용: 중앙통로로 칠할 노드 전체(위와 동일한 원리)
NODE_TO_BAY_ID = {}      # node_id -> 베이 번호(1~NUM_BAYS). 세로 열을 중앙통로로
                         # 잘라낸 구간 단위 - "베이1"..."베이89" 단위 리포트용.
NODE_TO_CORRIDOR_ID = {}  # node_id -> 중앙통로 번호(1=맨 위 라인, 커질수록 아래)
NODE_TO_MERGE_ID = {}    # node_id -> [합류부 번호, ...] (한 노드가 여러 곳에 속할 수 있음)
MERGE_ZONE_BAY_IDS = {}  # merge_id -> [베이 번호, ...] (그 합류부에 닿는 베이들)
NUM_BAYS = 0
NUM_CORRIDORS = 0
NUM_MERGE_ZONES = 0
try:
    with open(SECTION_REGIONS_JSON, encoding="utf-8") as f:
        _section_regions = json.load(f)
    NODE_TO_SECTIONS = _section_regions["node_to_sections"]
    SECTION_INFO = {int(sid): info for sid, info in _section_regions["sections"].items()}
    BAY_SECTION_IDS = {sid for sid, info in SECTION_INFO.items() if info["is_bay"]}
    CENTER_BAY_SECTION_IDS = {sid for sid, info in SECTION_INFO.items() if info["is_center_bay"]}
    # [갭 메우기] build_section_regions.py가 "베이 양 끝의 짧은 U턴 연결부"처럼
    # 어느 섹션에도 안 속하지만 실제로는 베이/중앙통로에 낀 노드들을 찾아
    # bay_nodes/center_nodes에 이미 합쳐뒀다. 화면 틴트(draw_region_tint)는
    # 섹션 단위 폴리라인이 아니라 이 "노드 집합"과 실제 그래프 간선(EDGES)을
    # 대조해서 그리므로, 사용자가 스크린샷으로 짚어준 끊긴 부분들이 메워진다.
    BAY_TINT_NODES = set(_section_regions.get("bay_nodes", []))
    CENTER_TINT_NODES = set(_section_regions.get("center_nodes", []))
    NODE_TO_BAY_ID = {n: int(v) for n, v in _section_regions.get("node_to_bay_id", {}).items()}
    NODE_TO_CORRIDOR_ID = {n: int(v) for n, v in _section_regions.get("node_to_corridor_id", {}).items()}
    NODE_TO_MERGE_ID = {n: [int(v) for v in ids] for n, ids in _section_regions.get("node_to_merge_id", {}).items()}
    MERGE_ZONE_BAY_IDS = {int(k): v for k, v in _section_regions.get("merge_zone_bay_ids", {}).items()}
    NUM_BAYS = max(NODE_TO_BAY_ID.values()) if NODE_TO_BAY_ID else 0
    NUM_CORRIDORS = max(NODE_TO_CORRIDOR_ID.values()) if NODE_TO_CORRIDOR_ID else 0
    NUM_MERGE_ZONES = max(MERGE_ZONE_BAY_IDS.keys()) if MERGE_ZONE_BAY_IDS else 0
    print(f"section_regions: 섹션 {len(SECTION_INFO)}개 "
          f"(베이 {len(BAY_SECTION_IDS)}개, 중앙통로 {len(CENTER_BAY_SECTION_IDS)}개), "
          f"매핑 노드 {len(NODE_TO_SECTIONS)}개, 틴트 노드 "
          f"{len(BAY_TINT_NODES)}(베이)/{len(CENTER_TINT_NODES)}(중앙통로)개, "
          f"베이 번호 1~{NUM_BAYS}, 중앙통로 번호 1~{NUM_CORRIDORS}, "
          f"합류부 번호 1~{NUM_MERGE_ZONES}")
except FileNotFoundError:
    print(f"[경고] {SECTION_REGIONS_JSON} 을(를) 찾을 수 없어 구역(베이/중앙통로) "
          f"분석 기능이 비활성화됩니다 (build_map_data.py로 생성 가능).")

# [베이/중앙통로 번호 라벨 위치] 각 베이/중앙통로마다 소속 노드
# 좌표의 평균("무게중심")을 대표 위치로 미리 계산해둔다. 베이는 중앙통로~
# 바깥벽 사이를 왕복하는 대체로 곧은 세로줄이라 평균 위치가 그 줄 중간
# 어딘가에 자연스럽게 떨어지고, 중앙통로는 폭 전체에 걸쳐 있어 평균 위치가
# 대략 가로 중앙 부근에 온다. T로 구역 틴트를 켰을 때 이 위치에 "베이12",
# "중앙통로1" 같은 번호를 같이 그려서 어느 구역이 몇 번인지 바로 알 수
# 있게 한다(draw_region_tint 참고).
_bay_id_coord_sum = {}
_bay_id_count = {}
for _n, _bid in NODE_TO_BAY_ID.items():
    if _n not in NODE_XY:
        continue
    _x, _y = NODE_XY[_n]
    _s = _bay_id_coord_sum.setdefault(_bid, [0.0, 0.0])
    _s[0] += _x
    _s[1] += _y
    _bay_id_count[_bid] = _bay_id_count.get(_bid, 0) + 1
BAY_ID_LABEL_POS = {
    _bid: (_s[0] / _bay_id_count[_bid], _s[1] / _bay_id_count[_bid])
    for _bid, _s in _bay_id_coord_sum.items()
}

_corridor_id_coord_sum = {}
_corridor_id_count = {}
for _n, _cid in NODE_TO_CORRIDOR_ID.items():
    if _n not in NODE_XY:
        continue
    _x, _y = NODE_XY[_n]
    _s = _corridor_id_coord_sum.setdefault(_cid, [0.0, 0.0])
    _s[0] += _x
    _s[1] += _y
    _corridor_id_count[_cid] = _corridor_id_count.get(_cid, 0) + 1
CORRIDOR_ID_LABEL_POS = {
    _cid: (_s[0] / _corridor_id_count[_cid], _s[1] / _corridor_id_count[_cid])
    for _cid, _s in _corridor_id_coord_sum.items()
}

# [합류부 라벨 위치] 한 노드가 여러 합류부에 동시에 속할
# 수 있으므로, 그 노드의 좌표를 자신이 속한 합류부 "전부"의 무게중심
# 계산에 반영한다(베이/중앙통로와 달리 리스트라 반복문 한 겹 더 필요).
_merge_id_coord_sum = {}
_merge_id_count = {}
for _n, _mids in NODE_TO_MERGE_ID.items():
    if _n not in NODE_XY:
        continue
    _x, _y = NODE_XY[_n]
    for _mid in _mids:
        _s = _merge_id_coord_sum.setdefault(_mid, [0.0, 0.0])
        _s[0] += _x
        _s[1] += _y
        _merge_id_count[_mid] = _merge_id_count.get(_mid, 0) + 1
MERGE_ID_LABEL_POS = {
    _mid: (_s[0] / _merge_id_count[_mid], _s[1] / _merge_id_count[_mid])
    for _mid, _s in _merge_id_coord_sum.items()
}

# [틴트용 간선 미리 계산] 매 정적 레이어 리빌드마다 EDGES 12,397개를 다시
# 훑지 않도록, "양 끝이 모두 베이(또는 중앙통로) 틴트 노드인" 간선만 모듈
# 로드 시 한 번 걸러서 ref-pixel 좌표 쌍으로 캐싱해둔다. 실제 트랙 간선을
# 그대로 쓰기 때문에(섹션의 node_ids 순서에 의존하지 않음) 두 틴트 노드가
# 그래프상 어떻게 이어져 있든 항상 끊김 없이 이어진다.
# [경계 램프 처리] 베이가 중앙통로로 합류/진입하는 지점은 대각선으로 살짝
# 꺾인 짧은 램프 구간이라, 그 램프의 두 끝 노드가 "한쪽은 베이 전용,
# 반대쪽은 중앙통로 전용"으로 서로 다르게 분류돼 있는 경우가 있다(실측
# 스크린샷으로 확인됨 - 114개 간선). 이런 "양쪽 다른 구역" 경계 간선은
# 위 조건 어디에도 안 걸려서 색이 아예 안 칠해지는 흰 틈으로 보였는데,
# 사용자 요청에 따라 이런 진입 램프는 베이 쪽으로 편입시켜 파란색으로
# 칠한다(베이에서 중앙통로로 들어가는 진입부이므로).
BAY_TINT_SEGMENTS = []
CENTER_TINT_SEGMENTS = []
if BAY_TINT_NODES or CENTER_TINT_NODES:
    for _a, _b in EDGES:
        _a_bay, _a_ctr = _a in BAY_TINT_NODES, _a in CENTER_TINT_NODES
        _b_bay, _b_ctr = _b in BAY_TINT_NODES, _b in CENTER_TINT_NODES
        if _a_bay and _b_bay:
            BAY_TINT_SEGMENTS.append((_a, _b))
        elif _a_ctr and _b_ctr:
            CENTER_TINT_SEGMENTS.append((_a, _b))
        elif _a_bay or _b_bay:
            # 베이 <-> (중앙통로 또는 바깥쪽 외곽 퍼리미터/미분류) 경계 진입
            # 램프: 베이 쪽 끝에서 뻗어나온 간선이므로 전부 베이 색으로
            # 편입한다. 중앙통로 쪽 램프뿐 아니라(이전 수정) 베이 맨 위/
            # 아래가 바깥 외곽 통로로 이어지는 램프(이번 요청)도 여기 포함.
            BAY_TINT_SEGMENTS.append((_a, _b))
        elif _a_ctr or _b_ctr:
            # 대칭 처리: 중앙통로 쪽 끝에서 뻗어나온 간선(반대편이 베이도
            # 아니고 중앙통로도 아닌 미분류)도 중앙통로 색으로 편입.
            CENTER_TINT_SEGMENTS.append((_a, _b))

# [합류부용 간선 미리 계산] 위와 같은 원리로, "양 끝이 모두 같은 합류부
# 번호에 속한" 간선만 걸러서 캐싱해둔다. 한 노드가 여러 합류부에 동시에
# 속할 수 있어서(문제 열은 왼쪽/오른쪽 두 합류부에 다 속함), 두 노드가
# 공유하는 합류부가 하나라도 있으면 그 번호로 그린다.
MERGE_TINT_SEGMENTS = {}  # merge_id -> [(a,b), ...]
if NODE_TO_MERGE_ID:
    for _a, _b in EDGES:
        _a_ids = set(NODE_TO_MERGE_ID.get(_a, ()))
        _b_ids = set(NODE_TO_MERGE_ID.get(_b, ()))
        _shared = _a_ids & _b_ids
        if _shared:
            for _mid in _shared:
                MERGE_TINT_SEGMENTS.setdefault(_mid, []).append((_a, _b))
        elif _a_ids or _b_ids:
            # [경계 간선] 한쪽 끝만 합류부인 간선(199개). 이걸 안 그리면
            # 합류부 구역 중간중간에 흰 틈이 생겨 "포함돼야 할 곳이 빠진
            # 것처럼" 보인다(사용자 스크린샷의 빨간 박스들이 전부 이 경우 -
            # 노드 자체는 이미 100% 합류부에 들어가 있었고, 그 사이를 잇는
            # 간선만 안 그려지고 있었다). 합류부 쪽 끝의 번호로 그린다.
            _mid = next(iter(_a_ids or _b_ids))
            MERGE_TINT_SEGMENTS.setdefault(_mid, []).append((_a, _b))

# ----------------------------------------------------------------------
# 그래프 인덱싱 + scipy sparse (Dijkstra용)
# ----------------------------------------------------------------------
NODE_LIST = list(NODE_XY.keys())
NODE_IDX = {nid: i for i, nid in enumerate(NODE_LIST)}
N = len(NODE_LIST)

# ----------------------------------------------------------------------
# OHT 최초 배치 위치 고정 (0번 요청사항)
# ----------------------------------------------------------------------
# [왜 고정하는가] 예전에는 매 실행마다 random.sample(NODE_LIST, num_oht)로
# 시작 위치를 새로 뽑았다. 그러면 실행할 때마다 OHT들이 다른 자리에서
# 출발해서, 같은 조건을 여러 번 돌려도 "정말 로직이 달라져서 결과가
# 바뀐 건지, 그냥 시작 위치가 달라서 바뀐 건지" 구분이 안 됐다. 이제는
# MAX_OHT(1000)대 분량의 시작 노드를 한 번 뽑아 OHT_PARK_JSON에 저장해두고,
# 그 뒤로는 항상 이 파일을 그대로 읽어 쓴다. num_oht대로 돌릴 때는 이
# 순서대로 앞에서부터 num_oht개를 쓴다 - 즉 "OHT 0번은 항상 이 자리에서
# 시작한다"가 대수를 바꿔 돌려도 유지된다(적은 대수는 항상 앞쪽 부분집합).
# [재생성] 파일이 있으면 무조건 그대로 쓴다(사용자 확인). 위치를 다시
# 뽑고 싶으면 이 파일을 수동으로 지우고 다시 실행하면 된다 - 코드가
# 스스로 재생성하지 않는다.
# [seed와의 관계] random.seed(seed)는 이제 시작 위치에 영향을 주지 않는다
# (파일에서 읽으므로). job 생성/배정 등 나머지 무작위성에만 적용된다.
OHT_PARK_JSON = "oht_park_positions.json"


def _load_or_create_park_positions():
    try:
        with open(OHT_PARK_JSON, encoding="utf-8") as f:
            positions = json.load(f)
        bad = [nid for nid in positions if nid not in NODE_XY]
        if bad:
            print(f"[경고] {OHT_PARK_JSON}에 현재 맵에 없는 노드 ID가 "
                  f"{len(bad)}개 있습니다(맵이 바뀐 뒤 파일을 안 지운 걸로 "
                  f"보입니다). 그래도 있는 그대로 씁니다 - 그 노드가 걸린 "
                  f"OHT는 배치 시 에러가 납니다. 필요하면 파일을 지우고 "
                  f"다시 실행해 새로 뽑으세요.")
        print(f"[OHT 배치] {OHT_PARK_JSON}에서 고정 시작위치 {len(positions)}개 로드")
        return positions
    except FileNotFoundError:
        rng = random.Random(0)  # 최초 생성은 항상 같은 결과가 나오도록 고정 시드
        positions = rng.sample(NODE_LIST, min(MAX_OHT, len(NODE_LIST)))
        with open(OHT_PARK_JSON, "w", encoding="utf-8") as f:
            json.dump(positions, f)
        print(f"[OHT 배치] {OHT_PARK_JSON} 없어서 새로 생성 - "
              f"MAX_OHT={MAX_OHT}대 기준 시작위치 {len(positions)}개")
        return positions


OHT_PARK_POSITIONS = _load_or_create_park_positions()


def get_park_nodes(num_oht):
    """num_oht대의 고정 시작 노드를 앞에서부터 순서대로 돌려준다.
    OHT_PARK_POSITIONS보다 많이 요청하면(=파일을 만든 뒤 MAX_OHT를 올렸거나
    그보다 큰 대수로 돌리는 경우) 여기서 막는다 - 조용히 무작위로 채워
    넣으면 '고정 위치' 취지가 깨지므로, 파일을 지우고 다시 생성하라고
    명확히 알려준다."""
    if num_oht > len(OHT_PARK_POSITIONS):
        raise ValueError(
            f"요청한 OHT {num_oht}대가 {OHT_PARK_JSON}에 저장된 고정 시작위치 "
            f"{len(OHT_PARK_POSITIONS)}개보다 많습니다. {OHT_PARK_JSON}을 "
            f"지우고 다시 실행하면 현재 MAX_OHT 기준으로 새로 생성됩니다.")
    return list(OHT_PARK_POSITIONS[:num_oht])


# ----------------------------------------------------------------------
# Job 스케줄 고정 (0번 요청사항 연장 - "위치도 job도 항상 똑같이 고정")
# ----------------------------------------------------------------------
# [왜 고정하는가] OHT 시작위치처럼, Job의 내용(범주/From/To 포트/긴급도)도
# 매 실행마다 그 자리에서 random 모듈로 새로 뽑으면 실행할 때마다 결과가
# 달라진다. 게다가 그 난수는 시뮬레이션 안의 다른 무작위성(예: 노드락
# 경합 시 어느 순서로 처리되는지 등)과 같은 전역 random 스트림을 공유하고
# 있어서, seed를 고정해도 무관한 로직 변경 하나로 job 내용이 밀려버릴 수
# 있다. 그래서 OHT_PARK_JSON과 똑같은 방식으로, Job 내용도 독립된 난수
# 스트림(random.Random(1) - 시작위치의 Random(0)과 다른 시드)으로 미리
# JOB_SCHEDULE_SIZE(20,000)건을 뽑아 JOB_SCHEDULE_JSON에 저장해두고, 그
# 뒤로는 항상 이 파일 그대로 순서대로 꺼내 쓴다.
# [재생성] 파일이 있으면 무조건 그대로 쓴다. 다시 뽑고 싶으면 파일을
# 수동으로 지우고 다시 실행하면 된다.
# [순서 재현] 매 시뮬레이션 실행(run_headless_sim/main 호출)마다 이
# 스케줄을 처음(0번)부터 다시 순서대로 소비한다(reset_sim_state()가
# 인덱스를 리셋) - 그래야 "같은 조건으로 두 번 돌리면 완전히 같은 job이
# 같은 순서로 나온다"가 성립한다.
JOB_SCHEDULE_JSON = "oht_lot_schedule.json"   # [5번] 파일명 변경 - 이전
    # oht_job_schedule.json(단발성 flat job)과 스키마가 달라 혼동 방지용.
    # 예전 파일이 폴더에 남아있어도 이 이름으로는 안 읽으니 안전하다.
JOB_SCHEDULE_SIZE = 20000   # [사용자 확정] 로트 20,000개 분량


def _make_lot_entry(rng):
    """독립 rng로 로트 1개의 사양을 뽑는다(포트/패턴/긴급도만 - leg 순서는
    runtime에 build_lot_legs()가 코드로 재구성한다. 결정론적이라 항상
    똑같은 leg 시퀀스가 나온다)."""
    pattern = rng.choices(
        list(LOT_PATTERN_WEIGHTS.keys()),
        weights=list(LOT_PATTERN_WEIGHTS.values()), k=1,
    )[0]
    stk = rng.choice(STK_PORTS)
    eq = rng.choice(EQ_PORTS)
    stb = rng.choice(STB_PORTS) if pattern != "NONE" else None

    priority = 0.0
    if rng.random() >= JOB_PRIORITY_ZERO_PROB:
        priority = rng.uniform(*JOB_PRIORITY_RANGE)

    return dict(pattern=pattern, stk_port=stk["port_id"], stk_node=stk["node"],
                eq_port=eq["port_id"], eq_node=eq["node"],
                stb_port=(stb["port_id"] if stb else None),
                stb_node=(stb["node"] if stb else None),
                priority=priority)


def build_lot_legs(entry):
    """로트 사양(entry) -> leg 딕셔너리 리스트. 각 leg는 Job 생성자에
    그대로 넘길 수 있는 형태(from_node/to_node/from_port/to_port/
    from_kind/to_kind). 순서:
        NONE  : STK->EQ, EQ->STK
        AFTER : STK->EQ, EQ->STB, STB->STK   (가공은 leg0 완료 직후)
        BEFORE: STK->STB, STB->EQ, EQ->STK   (가공은 leg1 완료 직후)
    EQ가 도착지인 leg가 끝나면 항상 PROCESS_TIME 가공이 끼어든다
    (advance_lot()에서 처리 - to_kind=="EQ"만 보면 되므로 패턴별로 어느
    leg인지 따로 표시해둘 필요가 없다)."""
    stk = dict(node=entry["stk_node"], port_id=entry["stk_port"], kind="STK")
    eq = dict(node=entry["eq_node"], port_id=entry["eq_port"], kind="EQ")

    def _leg(a, b):
        return dict(from_node=a["node"], to_node=b["node"],
                    from_port=a["port_id"], to_port=b["port_id"],
                    from_kind=a["kind"], to_kind=b["kind"])

    if entry["pattern"] == "NONE":
        return [_leg(stk, eq), _leg(eq, stk)]
    stb = dict(node=entry["stb_node"], port_id=entry["stb_port"], kind="STB")
    if entry["pattern"] == "AFTER":
        return [_leg(stk, eq), _leg(eq, stb), _leg(stb, stk)]
    return [_leg(stk, stb), _leg(stb, eq), _leg(eq, stk)]   # "BEFORE"


def _load_or_create_job_schedule():
    try:
        with open(JOB_SCHEDULE_JSON, encoding="utf-8") as f:
            schedule = json.load(f)
        bad = sum(1 for e in schedule if e["stk_node"] not in NODE_XY
                  or e["eq_node"] not in NODE_XY
                  or (e["stb_node"] is not None and e["stb_node"] not in NODE_XY))
        if bad:
            print(f"[경고] {JOB_SCHEDULE_JSON}에 현재 맵에 없는 노드가 걸린 "
                  f"항목이 {bad}건 있습니다(맵이 바뀐 뒤 파일을 안 지운 걸로 "
                  f"보입니다). 그래도 그대로 씁니다 - 해당 로트는 경로탐색 "
                  f"실패로 처리됩니다. 필요하면 파일을 지우고 다시 실행해 "
                  f"새로 뽑으세요.")
        print(f"[Job 스케줄] {JOB_SCHEDULE_JSON}에서 고정 로트 스케줄 {len(schedule)}건 로드")
        return schedule
    except FileNotFoundError:
        rng = random.Random(1)   # 시작위치(Random(0))와 다른 독립 시드
        schedule = [_make_lot_entry(rng) for _ in range(JOB_SCHEDULE_SIZE)]
        with open(JOB_SCHEDULE_JSON, "w", encoding="utf-8") as f:
            json.dump(schedule, f)
        print(f"[Job 스케줄] {JOB_SCHEDULE_JSON} 없어서 새로 생성 - "
              f"로트 {JOB_SCHEDULE_SIZE}건")
        return schedule


JOB_SCHEDULE = _load_or_create_job_schedule()
_JOB_SCHEDULE_IDX = [0]   # 다음에 꺼낼 스케줄 인덱스 (리스트로 감싸 reset에서 갈아끼움)

# [결정론 보강] OHT 시작위치/Job 내용은 파일로 고정했지만, 유휴 대기시간
# (아래 IDLE_STANDBY_HOLD_MIN/MAX 사이 무작위값, 락 경합 완화용 지터)은
# 여전히 매 호출마다 뽑아야 한다 - 없애면 유휴 차량들이 완전히 같은
# 타이밍에 몰려 움직여 경합이 오히려 나빠진다. 문제는 이걸 전역 random
# 모듈로 뽑으면 run_headless_sim(seed=...)의 seed에 따라 값이 달라지고,
# 로트 모델에서는 유휴 발생 빈도가 높아서(작업 대비 공급 과잉 시) 이
# 지터가 이벤트 처리 순서 자체를 바꿔버려 seed가 다르면 최종 결과까지
#달라지는 문제가 있었다(실측: leg 완료 277 vs 283, 로트완료 6 vs 4).
# 그래서 이 지터만 독립된 고정 시드 RNG로 분리한다 - "무작위이긴 하지만
# 항상 같은 순서로 소비"되어 seed 파라미터와 무관하게 재현된다.
_SIM_RNG = random.Random(2)   # 시작위치(0)/Job스케줄(1)과 다른 독립 시드


def next_lot_entry():
    """스케줄에서 다음 로트 사양을 순서대로 꺼낸다. 다 썼으면(스케줄보다
    긴 시뮬레이션) 조용히 무작위로 채우지 않고 명확한 에러로 막는다 -
    OHT 시작위치와 같은 원칙."""
    idx = _JOB_SCHEDULE_IDX[0]
    if idx >= len(JOB_SCHEDULE):
        raise ValueError(
            f"Job 스케줄(로트) {len(JOB_SCHEDULE)}건을 다 썼습니다(이번 실행이 "
            f"더 긴 시뮬레이션입니다). {JOB_SCHEDULE_JSON}을 지우고 "
            f"JOB_SCHEDULE_SIZE를 늘린 뒤 다시 실행하면 더 많이 생성됩니다.")
    _JOB_SCHEDULE_IDX[0] = idx + 1
    return JOB_SCHEDULE[idx]


# [맵 교체] 가중치 = 맵의 Link.Length(실측 주행거리). 유클리드 거리로
# 재계산하면 커브 링크에서 최대 2.9m까지 짧게 잡혀 최단경로가 실제보다
# 커브를 선호하게 된다. Length가 결측인 링크만 직선거리로 보정된다
# (보정은 build_map_data.py에서 이미 처리됨).
rows, cols, weights = [], [], []
ADJ = {nid: [] for nid in NODE_LIST}  # out-adjacency (실제 주행용)
for a, b in EDGES:
    rows.append(NODE_IDX[a])
    cols.append(NODE_IDX[b])
    weights.append(max(edge_length(a, b), 1e-6))
    ADJ[a].append(b)

GRAPH_CSR = csr_matrix((weights, (rows, cols)), shape=(N, N))


def validate_graph():
    """강연결/막다른 노드 여부를 빠르게 점검(런타임 안전 확인용)."""
    from scipy.sparse.csgraph import connected_components
    n_comp, _ = connected_components(GRAPH_CSR, directed=True, connection="strong")
    dead_ends = sum(1 for nid in NODE_LIST if not ADJ[nid])
    print(f"[validate_graph] strongly-connected components={n_comp}, dead-end nodes={dead_ends}")
    return n_comp == 1 and dead_ends == 0


validate_graph()

# ----------------------------------------------------------------------
# 시간 기반 라우팅 (pair-state A*)
# ----------------------------------------------------------------------
# [왜 거리가 아니라 시간인가]
# 회사 방법론 2번 "최단 소요시간 경로 탐색 - 실제 물리적 주행시간을 간선
# 가중치로 대입, 링크별 최대 허용속도 및 물리 가감속 반영"을 구현한 것이다.
# 이전 구현은 Link.Length(mm)를 가중치로 쓰는 최단"거리" 탐색이었고,
# 가감속은 경로를 고른 뒤 실제로 달릴 때(_segment_profile)만 반영됐다.
#
# [왜 노드가 아니라 링크쌍(pair-state)인가 - 이 맵의 실측 근거]
# 이 맵의 링크 길이 중앙값은 1,700mm인데 가감속에 필요한 거리는 그보다 길다.
#     정지 -> 3.5m/s        : 3,062mm  (한 링크 안에 도달 가능한 직선링크는
#                                       6,717개 중 22개뿐)
#     3.5 -> 0.7m/s (커브진입): 1,960mm  (직전 링크 1개로 감속 가능한 경우는
#                                       커브진입 877건 중 192건뿐)
#     0.7 -> 3.5m/s (커브탈출): 2,940mm  (한 링크로 회복 가능 3건 / 877건)
# 즉 "링크 하나를 정상속도로 통과"하는 근사(길이/속도상한)는 이 맵에서
# 성립하지 않는다. 링크의 통과시간은 "어느 링크에서 어떤 속도로 진입했는가"에
# 좌우되므로, 상태를 노드가 아니라 (직전링크 -> 다음링크) 쌍으로 확장해야
# 진입속도가 확정된다.
# 노드 단위로 "통과속도 = 인접 링크 속도상한의 최소값"이라고 뭉뚱그리는
# 근사도 쓸 수 없다. 분기노드 802개 중 587개(73%)가 후속 링크의 속도상한이
# 서로 다르기 때문에(한쪽은 커브, 한쪽은 직진), 그렇게 하면 직진하는 차량까지
# 커브속도로 끌어내리게 되고 하필 그게 경로 선택이 일어나는 지점이다.
# 다행히 이 맵은 진출입 차수가 최대 2라서 pair-state가 8,587개뿐이다
# (노드 6,983개와 같은 자릿수).
#
# [왜 캐시가 아니라 온디맨드 A*인가]
# 이전에는 출발가능 노드 3,408곳 전부에 대해 전역 dijkstra를 미리 계산해
# 캐시했다(warmup). 하지만 방법론 2-3번의 "실시간 교통 혼잡도에 따른 동적
# 가중치"가 들어오면 가중치가 주기적으로 바뀌므로 그런 정적 캐시는 통째로
# 무효가 된다. pair-state A*는 실측 평균 2.3ms / p95 6.7ms 라서 매 질의마다
# 새로 계산해도 충분하다. 그래서 캐시 구조를 걷어내고 온디맨드로 간다.
# ----------------------------------------------------------------------

# --- 링크를 인덱스 배열로 (pair-state 계산은 dict보다 배열이 훨씬 빠르다) ---
LINK_A = []      # 링크 i의 시작노드
LINK_B = []      # 링크 i의 끝노드
LINK_LEN = []    # 링크 i의 주행거리(mm)
LINK_CAP = []    # 링크 i의 속도상한(mm/s)
EDGE_LINK_IDX = {}   # (a,b) -> 링크 인덱스

for _lk in graph_data.get("links", []):
    _a, _b = _lk["a"], _lk["b"]
    if _a not in NODE_XY or _b not in NODE_XY:
        continue
    _i = len(LINK_A)
    EDGE_LINK_IDX[(_a, _b)] = _i
    LINK_A.append(_a)
    LINK_B.append(_b)
    LINK_LEN.append(max(float(_lk["length"]), 1e-6))
    # 속도상한: 커브(L/R/U)면 커브속도, 아니면 직선 최고속도.
    # [주의] 현재 맵(Map_LargeFab_CCW_v1_8.txt)은 Link.Speed가 7,785개
    # 전부 비어 있어서, 실질적으로 커브/직선 2단계로만 갈린다. 나중에
    # 구간별 속도제한 테이블이 생기면 이 자리에 그대로 들어온다.
    _c = OHT_CURVE_SPEED if (USE_LINK_CURVE_SPEED and _lk.get("curve")) else OHT_MAX_SPEED
    _sp = _lk.get("speed")
    if _sp:
        _c = min(_c, float(_sp))
    LINK_CAP.append(_c)

NUM_LINKS = len(LINK_A)
OUT_LINKS = {nid: [] for nid in NODE_LIST}   # 노드 -> 나가는 링크 인덱스
IN_LINKS = {nid: [] for nid in NODE_LIST}    # 노드 -> 들어오는 링크 인덱스
for _i in range(NUM_LINKS):
    OUT_LINKS[LINK_A[_i]].append(_i)
    IN_LINKS[LINK_B[_i]].append(_i)

# 링크 j 다음에 갈 수 있는 링크들(= pair-state의 후속)
SUCC_LINKS = [OUT_LINKS[LINK_B[_j]] for _j in range(NUM_LINKS)]

# ----------------------------------------------------------------------
# 합류부 노드 분류 (agv_fab_sim_mini.py의 MERGE_POINT/WAIT/PRIORITY 이식)
# ----------------------------------------------------------------------
# 합류부는 두 레일이 하나로 합쳐지는 지점이라 역할이 서로 다른 노드로
# 나뉜다. 노드 점 표시(D 키)에서 색을 구분하는 데 쓴다.
#   MERGE_POINT_NODES    : 합류 지점 자체(허가권 노드). in-degree(진입
#                           링크 수) > 1인 노드.
#   MERGE_WAIT_NODES     : 합류 지점 바로 앞, 꺾여 들어오며 허가권을
#                           기다리는 진입측 노드.
#   MERGE_PRIORITY_NODES : 같은 합류 지점으로 직진(우선) 들어오는 진입측
#                           노드.
# 판정: 이 맵의 합류 지점은 전부 진입 2개/진출 1개 구조라서, 각 진입
# 링크와 진출 링크 방향벡터의 코사인 유사도가 1.0(직선)에 가장 가까운
# 쪽을 우선(직진) 진입으로, 나머지를 합류대기 진입으로 판정한다.
MERGE_POINT_NODES = {n for n in NODE_LIST if len(IN_LINKS[n]) > 1}


def _unit_vec(a, b):
    xa, ya = NODE_XY[a]
    xb, yb = NODE_XY[b]
    dx, dy = xb - xa, yb - ya
    d = math.hypot(dx, dy)
    return (dx / d, dy / d) if d > 0 else (0.0, 0.0)


MERGE_WAIT_NODES = set()
MERGE_PRIORITY_NODES = set()
for _n in MERGE_POINT_NODES:
    _preds = [LINK_A[_li] for _li in IN_LINKS[_n]]
    _succs = [LINK_B[_li] for _li in OUT_LINKS[_n]]
    if len(_preds) < 2 or len(_succs) < 1:
        continue   # 방어적 처리(실측상 전부 진입2/진출1이라 원래는 발생 안 함)
    _vout = _unit_vec(_n, _succs[0])
    _scored = sorted(
        _preds,
        key=lambda _p: -(_unit_vec(_p, _n)[0] * _vout[0]
                         + _unit_vec(_p, _n)[1] * _vout[1]),
    )
    MERGE_PRIORITY_NODES.add(_scored[0])      # 코사인 유사도 최대(직진) = 우선
    for _p in _scored[1:]:
        MERGE_WAIT_NODES.add(_p)              # 나머지 = 합류 대기

print(f"[합류부] 허가권 노드(합류지점) {len(MERGE_POINT_NODES)}개, "
      f"합류대기 노드 {len(MERGE_WAIT_NODES)}개, "
      f"우선진입 노드 {len(MERGE_PRIORITY_NODES)}개")

# ----------------------------------------------------------------------
# Idle OHT 대기지점 (Zone = 베이, 베이마다 N등분)
# ----------------------------------------------------------------------
# 베이는 중앙통로에서 바깥벽 쪽으로 뻗은 세로줄이라, 노드를 y좌표 순으로
# 정렬해 N등분하면 그 줄을 따라 고르게 흩어진 지점들이 나온다. 각 구간의
# 가운데 노드 하나를 그 구간의 대기지점으로 쓴다.
# 중앙통로 노드는 대기 금지이므로 후보에서 제외한다.
CORRIDOR_NODE_SET = set(NODE_TO_CORRIDOR_ID)

_bay_nodes = {}
for _n, _bid in NODE_TO_BAY_ID.items():
    if _n in CORRIDOR_NODE_SET or _n not in NODE_XY:
        continue
    _bay_nodes.setdefault(_bid, []).append(_n)

IDLE_STANDBY_POINTS = []      # [node_id, ...] 전체 대기지점
IDLE_POINT_BAY = []           # 같은 인덱스의 대기지점이 속한 베이 번호
for _bid in sorted(_bay_nodes):
    _nodes = sorted(_bay_nodes[_bid], key=lambda n: (NODE_XY[n][1], NODE_XY[n][0]))
    _k = min(IDLE_ZONE_DIVISIONS, len(_nodes))
    if _k <= 0:
        continue
    for _s in range(_k):
        _lo = _s * len(_nodes) // _k
        _hi = (_s + 1) * len(_nodes) // _k
        if _hi <= _lo:
            continue
        _mid = _nodes[(_lo + _hi) // 2]      # 구간 가운데 노드를 대표로
        IDLE_STANDBY_POINTS.append(_mid)
        IDLE_POINT_BAY.append(_bid)

# 대기지점 -> 지금 그 자리를 배정받은 OHT (1지점 1대)
STANDBY_ASSIGNED = {}

IDLE_STATS = {
    "yield_requests": 0,   # 유휴차량에게 '비켜달라'고 통보한 횟수
    "yield_moves": 0,      # 그 요청을 받아 실제로 비켜준 횟수
    "assigns": 0,          # 대기지점 배정 횟수
}

print(f"[Idle 분산] Zone=베이 {len(_bay_nodes)}개 x {IDLE_ZONE_DIVISIONS}등분 "
      f"-> 대기지점 {len(IDLE_STANDBY_POINTS)}곳 "
      f"(중앙통로 {len(CORRIDOR_NODE_SET)}노드는 대기 금지)")


def _junction_speeds():
    """pair-state (i -> j)마다 "링크 i를 마치고 j에 진입하는 순간의 속도"를
    구한다.

    1차 근사는 min(cap_i, cap_j) - 직선(3.5)에서 커브(0.7)로 들어가면 커브
    상한이 걸리고, 커브에서 직선으로 나오면 커브 속도에서 시작한다.

    여기에 방법론 2-2번의 "사전 감속"을 backward fixpoint로 얹는다. 링크 j를
    지나 그 다음에 더 느린 구간이 온다면, j의 길이만으로는 감속이 안 끝날 수
    있다(이 맵에서는 커브진입 877건 중 685건이 그렇다). 그때는 j에 진입하는
    속도 자체를 미리 낮춰야 한다:

        v(i,j) <= sqrt( v_out(j)^2 + 2 * 감속도 * len(j) )

    v_out(j)는 j 이후 상태들의 진입속도 중 최소값이다(어느 분기로 갈지 모르니
    보수적으로 최소). 이 부등식을 수렴할 때까지 반복하면 여러 링크에 걸친
    다중홉 사전감속이 자연스럽게 전파된다. 이 맵에서는 3회 반복이면 수렴한다.
    """
    vj = {}
    for j in range(NUM_LINKS):
        for i in IN_LINKS[LINK_A[j]]:
            vj[(i, j)] = min(LINK_CAP[i], LINK_CAP[j])

    # --- 후방 전파: 앞으로 느려질 구간을 위해 미리 감속 ---
    # --- 전방 전파: 방금 느린 구간에서 나왔으면 아직 그 속도를 못 낸다 ---
    # 후방만 돌리면 v(i,j)는 "감속 제약만 반영한 상한"이라, 커브를 막 빠져나온
    # 직후처럼 가속으로는 도달할 수 없는 속도가 남는다(이 맵에서 전이
    # 9,476건 중 932건, 부족분 중앙값 808mm/s). 그 상태로 두면 라우팅이
    # 실제로 낼 수 없는 속도를 전제로 시간을 계산해 실행결과와 어긋난다.
    #     v(i,j) <= sqrt( v(h,i)^2 + 2 * 가속도 * len(i) )
    # h는 i의 선행 링크다. 어느 쪽에서 왔는지 모르니 후보 중 최대값을 쓴다
    # (가장 빠르게 진입할 수 있었던 경우). 둘 다 "값을 낮추는" 방향이라
    # 번갈아 돌려도 단조 감소해서 반드시 수렴한다.
    for _ in range(64):
        changed = 0
        for (i, j), v in vj.items():
            ks = SUCC_LINKS[j]
            if ks:
                v_out = min([vj.get((j, k), LINK_CAP[j]) for k in ks])
            else:
                v_out = 0.0    # 막다른 링크: 끝에서 정지해야 함
            lim = math.sqrt(v_out * v_out + 2.0 * OHT_DECEL * LINK_LEN[j])

            hs = IN_LINKS[LINK_A[i]]
            if hs:
                v_prev = max([vj.get((h, i), 0.0) for h in hs])
            else:
                v_prev = 0.0   # 선행 링크가 없으면 정지에서 출발
            acc_lim = math.sqrt(v_prev * v_prev + 2.0 * OHT_ACCEL * LINK_LEN[i])

            nv = min(LINK_CAP[i], LINK_CAP[j], lim, acc_lim)
            if nv < v - 1e-6:
                vj[(i, j)] = nv
                changed += 1
        if not changed:
            break
    return vj


_t0 = time.time()
JUNCTION_SPEED = _junction_speeds()
print(f"[라우팅] pair-state {len(JUNCTION_SPEED)}개 / 사전감속 전파 "
      f"{time.time() - _t0:.2f}s (링크 {NUM_LINKS}, 노드 {N})")


def link_travel_time(v_in, v_out, cap, dist):
    """길이 dist인 링크를 v_in으로 진입해 v_out으로 빠져나가는 최소 소요시간과
    실제 탈출속도를 (시간, 탈출속도)로 돌려준다.

    가속도 OHT_ACCEL로 올리고 상한 cap에서 순항하다 OHT_DECEL로 낮추는
    사다리꼴(거리가 짧으면 삼각형) 프로파일이다.

    [핵심] 요청된 v_out을 먼저 "이 링크 안에서 물리적으로 가능한 범위"
    [v_floor, v_ceil]로 자른다. 이 클램프가 없으면 도달 불가능한 탈출속도를
    그대로 받아 삼각형 공식이 실제보다 긴 시간을 내놓는다(라우팅이 실행보다
    느리게 예측하던 원인). 범위 안으로만 들어오면 아래 공식은 정확하다."""
    v_in = min(max(v_in, 0.0), cap)
    v_out = min(max(v_out, 0.0), cap)
    # 최대로 감속해도 이 밑으로는 못 내려간다 / 최대로 가속해도 이 위로는 못 간다
    v_floor = math.sqrt(max(v_in * v_in - 2.0 * OHT_DECEL * dist, 0.0))
    v_ceil = min(math.sqrt(v_in * v_in + 2.0 * OHT_ACCEL * dist), cap)
    v_out = min(max(v_out, v_floor), v_ceil)

    d_acc = max(cap * cap - v_in * v_in, 0.0) / (2.0 * OHT_ACCEL)
    d_dec = max(cap * cap - v_out * v_out, 0.0) / (2.0 * OHT_DECEL)
    if d_acc + d_dec <= dist:
        # 상한속도까지 올라가서 순항할 여유가 있다(사다리꼴)
        return (((cap - v_in) / OHT_ACCEL + (cap - v_out) / OHT_DECEL
                 + (dist - d_acc - d_dec) / cap), v_out)
    # 상한에 못 미치고 꼭짓점에서 바로 감속으로 꺾인다(삼각형)
    v_pk = math.sqrt(max(
        (2.0 * OHT_ACCEL * OHT_DECEL * dist
         + OHT_DECEL * v_in * v_in + OHT_ACCEL * v_out * v_out)
        / (OHT_ACCEL + OHT_DECEL), 0.0))
    v_pk = max(v_pk, v_in, v_out)
    return ((v_pk - v_in) / OHT_ACCEL + (v_pk - v_out) / OHT_DECEL, v_out)


def link_time(v_in, v_out, li):
    """경로 탐색이 쓰는 링크 통과 '비용'.

    물리 통과시간에 실시간 혼잡 페널티(LINK_PENALTY)를 곱한 값이다. 혼잡이
    없으면 계수가 1.0이라 순수 물리시간과 같다. A*의 휴리스틱(직선거리/최고속)
    은 계수가 항상 1 이상이라 여전히 하한이므로 최적성이 유지된다.

    물리 그대로가 필요한 곳(실제 주행, path_travel_time)은 link_travel_time()
    을 직접 쓴다."""
    return (link_travel_time(v_in, v_out, LINK_CAP[li], LINK_LEN[li])[0]
            * LINK_PENALTY[li])


def shortest_path(src, dst, avoid=None, from_link=None):
    """src -> dst 최소 "소요시간" 경로(노드 id 리스트). 도달 불가면 None.

    avoid: 이번 탐색에서만 임시로 제외할 링크 인덱스 집합. 분기점 정체
    (트리거 2)에서 "지금 막혀 있는 그 링크만 빼고 우회로를 찾아라"에 쓴다.
    BLOCKED_LINKS(장애/유지보수 차단)는 항상 제외된다.

    from_link: 지금 src에 "달려서" 도착한 경우 그 진입 링크. 주행 중
    재탐색(트리거 1/2)에서 반드시 넘겨야 한다. 이걸 안 넘기면 차량이
    정지 상태라고 가정하고 계산하는데, 실제로는 분기점을 3.5m/s로
    통과하는 중이라 커브 분기의 상대 비용이 실제보다 싸게 나온다.
    그러면 최적 경로의 접미부를 다시 구했을 뿐인데도 매번 다른 가지를
    고르게 되어(실측 재탐색의 47.6%가 첫 링크부터 달라졌다) 차량들이
    경로를 계속 갈아타며 차간거리 정체를 키운다.

    상태 (i, j) = "링크 i 주행을 끝내고 노드 LINK_B[i](=LINK_A[j])에 서서,
    다음으로 링크 j에 진입하려는 시점". 이렇게 두면 링크 j의 진입속도가
    JUNCTION_SPEED[(i,j)]로 확정되어 통과시간을 정확히 계산할 수 있다.
    출발은 정지상태(v=0), 도착은 정지상태(v=0)로 둔다 - OHT는 포트에서
    Load/Unload를 위해 반드시 멈추기 때문이다.

    휴리스틱은 "남은 직선거리 / 최고속도"로, 실제 소요시간을 절대 넘지 않아
    A*의 최적성이 보장된다(admissible).
    """
    if src == dst:
        return [src]
    if src not in OUT_LINKS or dst not in NODE_XY:
        return None
    if avoid:
        banned = BLOCKED_LINKS | set(avoid)
    else:
        banned = BLOCKED_LINKS
    dx_goal, dy_goal = NODE_XY[dst]

    dist_g = {}
    prev = {}
    pq = []

    def _push(state, g, parent):
        if g < dist_g.get(state, math.inf):
            dist_g[state] = g
            prev[state] = parent
            if state[0] == "S":       # 종료상태는 휴리스틱 0
                heapq.heappush(pq, (g, g, state))
            else:
                bx, by = NODE_XY[LINK_B[state[1]]]
                h = math.hypot(dx_goal - bx, dy_goal - by) / OHT_MAX_SPEED
                heapq.heappush(pq, (g + h, g, state))

    if from_link is not None and from_link not in banned:
        # 주행 중 재탐색: 지금 이 순간의 상태(from_link로 진입 완료)에서
        # 그대로 이어서 탐색한다. 상태 (from_link, k)의 진입속도가
        # JUNCTION_SPEED에 이미 들어 있으므로 비용 0으로 심으면 된다.
        for k in SUCC_LINKS[from_link]:
            if k not in banned and (from_link, k) in JUNCTION_SPEED:
                _push((from_link, k), 0.0, None)
    else:
        # 정지상태에서 첫 링크로 진입
        for j in OUT_LINKS[src]:
            if j in banned:
                continue
            if LINK_B[j] == dst:
                _push(("S", j), link_time(0.0, 0.0, j), None)
            for k in SUCC_LINKS[j]:
                if k not in banned and (j, k) in JUNCTION_SPEED:
                    _push((j, k),
                          link_time(0.0, JUNCTION_SPEED[(j, k)], j), None)

    goal = None
    while pq:
        _f, g, state = heapq.heappop(pq)
        if g > dist_g.get(state, math.inf):
            continue           # 더 좋은 값으로 이미 갱신된 낡은 항목
        if state[0] == "S":
            goal = state
            break
        i, j = state
        v_in = JUNCTION_SPEED[(i, j)]
        if LINK_B[j] == dst:   # 링크 j를 달려 목적지에서 정지
            _push(("S", j), g + link_time(v_in, 0.0, j), state)
        for k in SUCC_LINKS[j]:
            nxt = (j, k)
            if k not in banned and nxt in JUNCTION_SPEED:
                _push(nxt, g + link_time(v_in, JUNCTION_SPEED[nxt], j), state)

    if goal is None:
        return None

    states = []
    cur = goal
    while cur is not None:
        states.append(cur)
        cur = prev.get(cur)
    states.reverse()

    # 각 상태에서 "주행을 마친" 링크는 상태의 첫 원소(종료상태는 둘째 원소).
    path = [src]
    for st in states:
        li = st[1] if st[0] == "S" else st[0]
        if LINK_B[li] != path[-1]:
            path.append(LINK_B[li])
    return path


# ======================================================================
# 경로 재탐색 (회사 방법론 3번)
# ======================================================================
# 배정/탐색 계산은 10초 주기(DISPATCH_INTERVAL)지만, 이미 달리고 있는 개별
# OHT의 경로 재탐색은 그와 별개로 아래 4가지 이벤트에서 일어난다.
#   (1) 분기점 접근 시      - 최소 5초 간격
#   (2) 분기점 정체 10초    - 우회로 탐색
#   (3) 전방 경로 차단      - 즉시
#   (4) 경로 탐색 실패      - 5초 쿨타임 후 재시도
# [측정 결과 주의] (1)번은 "전방 교통 상황을 재확인"하는 트리거인데, 지금은
# 간선 가중치가 정적이라 재확인할 교통 정보가 아직 없다. 실측해보면 재탐색
# 결과가 기존 경로와 같은 경우가 99.7%(같은 목적지 기준)라 라우팅 이득이
# 사실상 0인 반면, 경로를 5초마다 끊었다 잇는 것만으로 차간거리 정체가
# 늘어 평균 대기가 27s -> 49s, 처리량이 약 3% 떨어졌다(경로를 그대로 재사용
# 하고 재진입만 시켜도 같은 저하가 재현되므로 원인은 경로 변경이 아니라
# 주행을 끊는 것 자체다).
# 5단계에서 혼잡도 기반 동적 가중치가 들어오면 비로소 "전방 교통 상황"이
# 생겨 이 트리거가 제 역할을 하므로, 그때 다시 벤치마크해야 한다.
ENABLE_JUNCTION_REROUTE = True   # (1)번 트리거 사용 여부
JUNCTION_REROUTE_MIN_GAP = 5.0   # (1) 연속 분기점에서의 시스템 부하 방지
JUNCTION_STALL_REROUTE = 10.0    # (2) 같은 분기점에서 이만큼 못 지나가면 우회
# [누적 정체 강제 우회] 트리거2는 재시도마다 "방금 막힌 링크 하나"만 피한다.
# 근처가 전반적으로 혼잡하면 매번 다른 링크에 또 걸리면서 사실상 같은
# 동네를 못 벗어날 수 있다. 그래서 한 번의 10초 정체가 아니라, 여러 번의
# 재탐색을 거치며 누적된 정체시간을 따로 추적한다. 이 값을 넘으면 그동안
# 걸렸던 링크 전부를 회피 목록에 쌓아 완전히 다른(더 멀더라도) 경로를
# 강제로 찾게 한다. [사용자 확정] 60초(트리거2 재시도 6회 수준).
FORCE_DETOUR_SECONDS = 60.0
PATH_RETRY_COOLDOWN = 5.0        # (4) 탐색 실패 후 재시도까지 쿨타임
MAX_PATH_RETRY = 12              # 그래도 계속 실패하면 작업을 풀에 반납

# 분기 노드(Stop-Node): 나가는 링크가 2개 이상이라 경로 선택이 일어나는 곳.
# 재탐색을 여기서만 하는 이유는, 갈림길이 없는 곳에서 경로를 다시 잡아봐야
# 결과가 같기 때문이다.
JUNCTION_NODES = {n for n in NODE_LIST if len(OUT_LINKS[n]) >= 2}

# 장애/유지보수로 통행이 막힌 링크. block_link()로만 조작한다.
BLOCKED_LINKS = set()

# ---- 실시간 교통 혼잡도 (방법론 2-3번) ----------------------------------
# "해당 링크의 주행 차량 수 및 대기 차량 밀집도에 따라 링크의 예상 주행속도가
#  감속 보정되어 가중치소요시간에 반영"
#
# [왜 순간 점유가 아니라 누적인가] 순간 스냅샷으로 링크별 차량 수를 세보면
# 150대 기준 링크의 94.7%가 0이라 라우팅 신호가 되지 못한다(차량 150대가
# 링크 7,785개에 흩어져 있으니 당연하다). 실제 혼잡은 "특정 지점에서 반복해서
# 막힌다"는 형태로 나타나므로, 한 주기 동안의 통과량(주행 차량 수)과 진입
# 대기시간(대기 차량 밀집도)을 링크별로 모아서 쓴다.
LINK_FLOW = np.zeros(NUM_LINKS)        # 이번 주기에 이 링크를 통과한 차량 수
LINK_STALL = np.zeros(NUM_LINKS)       # 이번 주기에 이 링크로 진입하려다 기다린 시간(초)
LINK_PENALTY = np.ones(NUM_LINKS)      # 라우팅 가중치에 곱할 계수(>=1.0)
LINK_DELAY = np.zeros(NUM_LINKS)       # 차량 1대당 평균 진입 대기시간(초), EWMA

CONGESTION_INTERVAL = 10.0    # 혼잡도 갱신 주기(초)
CONGESTION_ALPHA = 0.15       # EWMA 계수. 시정수 약 65초.
                              # 0(=혼잡도 미반영)/0.15/0.3/0.6을 비교했을 때
                              # 0.15가 가장 좋았다(평균 대기 51.6 -> 39.2s).
                              # 더 크게 잡으면 한 주기의 튀는 관측에 과민해져
                              # 차량들이 우르르 몰려다니며 다시 나빠진다.
CONGESTION_MAX_PENALTY = 10.0  # 한 번의 극단적 관측이 지도를 지배하지 않게 상한

# 링크를 막힘 없이 통과할 때의 대략적 소요시간. 페널티를 "몇 배 느려졌나"로
# 환산하는 기준이 된다.
LINK_BASE_TIME = np.array([LINK_LEN[i] / LINK_CAP[i] for i in range(NUM_LINKS)])


def update_congestion():
    """한 주기 동안 모은 통과량/대기시간으로 링크별 감속 보정계수를 갱신한다.

    차량 1대가 이 링크에 진입하려고 평균 몇 초를 기다렸는지(delay)를 구해,
    순수 통과시간(base) 대비 몇 배가 되는지를 페널티로 삼는다.

        penalty = (base + delay) / base = 1 + delay / base

    즉 "예상 주행속도를 그만큼 감속 보정"한 것과 같다(시간은 속도의 역수).
    한 주기 관측만 쓰면 값이 튀므로 EWMA로 눌러준다.

    [주의] 이 페널티는 '경로를 고를 때'만 쓴다. 실제 주행 물리(_segment_profile)
    와 순수 주행시간(path_travel_time)은 건드리지 않는다. 실제 지연은
    시뮬레이션이 노드락/차간거리로 이미 만들어내고 있으므로, 물리에까지
    반영하면 같은 지연을 두 번 세게 된다."""
    raw = LINK_STALL / np.maximum(LINK_FLOW, 1.0)
    LINK_DELAY[:] = (1.0 - CONGESTION_ALPHA) * LINK_DELAY + CONGESTION_ALPHA * raw
    np.clip(1.0 + LINK_DELAY / LINK_BASE_TIME, 1.0, CONGESTION_MAX_PENALTY,
            out=LINK_PENALTY)
    LINK_FLOW[:] = 0.0
    LINK_STALL[:] = 0.0




# 통계
REROUTE_COUNTS = {"junction": 0, "stall": 0, "blocked": 0, "fail": 0,
                   "no_alt": 0}   # [진단] avoid로 찾았는데 대안이 없어 원래 경로로 복귀한 횟수


def _on_blocked_changed():
    """차단 목록이 바뀌면 W_movetime용 주행시간 행렬(CSR)을 다시 굽는다.
    차단은 드문 이벤트라 통째로 재구축해도 0.1초면 끝난다."""
    global TIME_CSR, TIME_NODE_IDX, TIME_STOP_OFF
    if "_build_time_csr" in globals():
        TIME_CSR, TIME_NODE_IDX, TIME_STOP_OFF = _build_time_csr()


def block_link(li, notify=True):
    """링크 li를 차단한다. 그 링크를 지나갈 예정이던 OHT는 (3)번 트리거로
    즉시 재탐색한다."""
    if li in BLOCKED_LINKS:
        return
    BLOCKED_LINKS.add(li)
    _on_blocked_changed()
    if notify:
        notify_blocked_ahead()


def unblock_link(li):
    if li not in BLOCKED_LINKS:
        return
    BLOCKED_LINKS.discard(li)
    _on_blocked_changed()


def notify_blocked_ahead():
    """앞으로 갈 경로에 차단 링크가 걸린 OHT들을 깨워 즉시 재탐색시킨다."""
    if not BLOCKED_LINKS:
        return
    for oht in OHT_LIST:
        path = oht._leg_path
        if not path or oht._leg_idx >= len(path) - 1:
            continue
        remain = path[oht._leg_idx:]
        for a, b in zip(remain, remain[1:]):
            if EDGE_LINK_IDX.get((a, b)) in BLOCKED_LINKS:
                oht._replan = True
                REROUTE_COUNTS["blocked"] += 1
                if oht.ispassive():
                    oht.activate()
                break


def path_link_indices(path):
    """노드 경로 -> 링크 인덱스 리스트. 인접하지 않은 구간이 있으면 None."""
    out = []
    for a, b in zip(path, path[1:]):
        li = EDGE_LINK_IDX.get((a, b))
        if li is None:
            return None
        out.append(li)
    return out


def path_travel_cumtime(path):
    """path_travel_time()과 같은 계산을 하되, "경로의 i번째 노드까지 걸리는
    누적시간" 리스트를 함께 돌려준다 -> (총시간, [0, t1, t2, ...]).

    후보 선별 1-③ "하차지점 10초 이내"를 매번 다시 계산하지 않기 위한 것이다.
    구간(leg) 주행을 시작할 때 한 번만 만들어두면, 그 뒤로는 지금 몇 번째
    노드에 있는지만 알면 남은 예상 주행시간이 뺄셈 한 번으로 나온다.
    (노드락 대기로 지연되면 도착 시각 자체는 밀리지만 '남은 주행시간'은
     그대로이므로, 추정치로 쓰기에 맞는 값이다.)"""
    links = path_link_indices(path)
    if not links:
        return 0.0, [0.0]
    cum = [0.0]
    total = 0.0
    v = 0.0
    for idx, j in enumerate(links):
        v_out = 0.0 if idx == len(links) - 1 else \
            JUNCTION_SPEED.get((j, links[idx + 1]), 0.0)
        dt, v = link_travel_time(v, v_out, LINK_CAP[j], LINK_LEN[j])
        total += dt
        cum.append(total)
    return total, cum


def path_travel_time(path):
    """주어진 노드 경로를 주행했을 때의 순수 주행시간(초). 혼잡/대기 제외.
    3단계의 W_movetime이 이 값을 쓴다.

    A*의 간선비용은 진입속도를 JUNCTION_SPEED로 "가정"하고 계산하지만(그래야
    상태별로 비용이 분해된다), 실제 차량은 정지에서 출발해 속도를 앞으로
    물려받는다. 그래서 여기서는 실행부(_travel_path)와 똑같이 앞에서 뒤로
    실제 속도를 전파한다 - 이 값이 실측 주행시간과 일치하는 쪽이다."""
    return path_travel_cumtime(path)[0]


# ----------------------------------------------------------------------
# 화면 좌표 변환 (배경 PIL 굽기 + salabim 애니메이션 공용)
# ----------------------------------------------------------------------
def sx(x):
    return MARGIN + x * SCALE


def sy_screen(y):
    """salabim 캔버스는 y-up. 원본 데이터도 y-up이라 추가 반전 없이 그대로."""
    return MARGIN + y * SCALE


def sy_pil(y):
    """PIL 래스터는 항상 row0=top 이므로 굽는 시점에 1회만 뒤집는다.
    (원본 데이터는 y-up 이므로 이 변환이 곧 '위가 위'로 보이게 하는 유일한 반전)"""
    img_h = int(PAGE_H * SCALE) + 2 * MARGIN
    return img_h - sy_screen(y)


def screen_xy(node_id):
    x, y = NODE_XY[node_id]
    return sx(x), sy_screen(y)


IMG_W = int(PAGE_W * SCALE) + 2 * MARGIN
IMG_H = int(PAGE_H * SCALE) + 2 * MARGIN

# ----------------------------------------------------------------------
# 폰트 로딩
# ----------------------------------------------------------------------
# [한글 폰트 수정] 기존에는 DejaVuSans.ttf 하나만 시도했는데, 이 폰트는
# 한글 글리프가 없어서 "[재생/REPLAY]", "적재중" 같은 한글 텍스트가 전부
# 네모(□) 박스로 깨져 보였다. 이제는 윈도우(맑은 고딕) -> 리눅스(나눔고딕)
# -> 마지막 fallback(DejaVuSans, 한글은 깨지지만 최소한 프로그램은 죽지
# 않음) 순서로 시도한다. 이 함수는 모듈 로드 시(아래 if not HEADLESS: 블록
# 안에서) 딱 한 번만 호출되고, 결과는 FONT_STATS/FONT_LEGEND 전역 변수에
# 저장되어 render_frame()/render_replay_frame()이 매 프레임 재사용한다
# (프레임마다 폰트 파일을 다시 여는 건 불필요한 디스크 I/O이므로 render
# 함수들 안에서는 절대 이 함수를 다시 호출하지 않는다).
def _load_font(size):
    for path in (
        "C:/Windows/Fonts/malgun.ttf",                         # 윈도우: 맑은 고딕
        "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",      # 리눅스: 나눔고딕
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",      # 최후 fallback(한글 없음)
    ):
        try:
            return ImageFont.truetype(path, size)
        except Exception:
            continue
    return ImageFont.load_default()


# ----------------------------------------------------------------------
# 레일 구간(segment) 분할 - 분기점 기준 + 정체(congestion) 오버레이
# ----------------------------------------------------------------------
# [요청 반영] "레일에 색을 넣어 막히는 부분을 표시"하되, 개별 노드/엣지가
# 아니라 "레일 하나" 단위(분기점~분기점 사이)로 묶어서 보여달라는 요청.
# agv_graph.json의 노드 연결을 방향 무시(undirected)로 보고, 연결된 이웃이
# 정확히 2개가 아닌 노드(막다른 길=1개, 분기/합류점=3개 이상)를 "분기점"
# 으로 정의한다. 분기점과 분기점 사이를 잇는, 이웃이 2개뿐인 노드들의
# 사슬을 레일 1개(segment)로 묶는다. 분기점이 전혀 없는 고립 순환 루프
# (전부 degree=2)는 그 루프 전체를 레일 1개로 예외 처리한다.
RAIL_SEGMENTS = []          # [[node_id, ...], ...] 분기점~분기점 사이 레일들
NODE_TO_SEGMENTS = {}       # node_id -> [segment_idx, ...] (분기점은 여러 레일에 걸침)
NODE_WEIGHT = {}            # node_id -> 그 노드 점유량 1을 걸쳐있는 레일 수만큼 나눈 몫
RAIL_CAPACITY = []          # segment_idx -> 그 레일의 "정밀" 수용량(노드 가중치 합)


# --- Idle(평상시) 설비 표시 설정 ---
# [요청 반영] Idle 설비를 아예 안 그리면 레일은 잘 보이지만 설비 위치
# 자체가 안 보이고, 불투명하게 그리면 레일을 가린다. 그래서 두 방식을
# SHOW_EQ_IDLE 토글 하나로 선택할 수 있게 했다.
#   True  -> Idle 설비를 EQ_IDLE_ALPHA 투명도의 반투명 원으로 표시.
#            레일 색과 설비 위치가 동시에 보인다(값을 낮출수록 레일이
#            더 잘 비치고, 높일수록 설비가 더 또렷해진다. 0~255).
#   False -> Idle 설비는 아예 그리지 않는다. 실제로 뭔가 진행 중인
#            (예약/가공/완료) 설비만 표시되므로 레일 색이 가장 또렷하게
#            보이지만, 지금 어디에 설비가 있는지는 알 수 없다.
SHOW_EQ_IDLE = False
EQ_IDLE_ALPHA = 10



def _build_rail_segments():
    undirected_adj = {nid: set() for nid in NODE_LIST}
    for a, b in EDGES:
        undirected_adj[a].add(b)
        undirected_adj[b].add(a)
    junction_nodes = {n for n in NODE_LIST if len(undirected_adj[n]) != 2}

    segments = []
    visited_edges = set()  # frozenset({a, b}) 로 무방향 엣지 방문 표시(중복 레일 방지)

    def walk_chain(start, first_next):
        chain = [start, first_next]
        prev, cur = start, first_next
        while cur not in junction_nodes:
            nbrs = undirected_adj[cur] - {prev}
            if not nbrs:
                break
            nxt = next(iter(nbrs))
            chain.append(nxt)
            prev, cur = cur, nxt
            if cur == start:  # 방어적: 분기점 없이 시작점으로 돌아온 루프
                break
        return chain

    for j in junction_nodes:
        for nb in undirected_adj[j]:
            key = frozenset((j, nb))
            if key in visited_edges:
                continue
            chain = walk_chain(j, nb)
            for i in range(len(chain) - 1):
                visited_edges.add(frozenset((chain[i], chain[i + 1])))
            segments.append(chain)

    # 분기점이 하나도 없는 고립 순환 루프(전부 degree=2) 처리
    covered = set()
    for seg in segments:
        covered.update(seg)
    seen = set()
    for n in NODE_LIST:
        if n in covered or n in seen:
            continue
        loop = [n]
        seen.add(n)
        prev, cur = None, n
        while True:
            nbrs = undirected_adj[cur] - ({prev} if prev is not None else set())
            nxt = next(iter(nbrs), None)
            if nxt is None or nxt == n:
                break
            loop.append(nxt)
            seen.add(nxt)
            prev, cur = cur, nxt
        if len(loop) > 1:
            segments.append(loop)

    return segments, len(junction_nodes)


if not HEADLESS and SHOW_RAIL_CONGESTION:
    RAIL_SEGMENTS, _n_junctions = _build_rail_segments()
    for _si, _seg in enumerate(RAIL_SEGMENTS):
        for _n in _seg:
            NODE_TO_SEGMENTS.setdefault(_n, []).append(_si)

    # [정밀화] 분기점 노드 1개는 여러 레일(자신이 끝점인 레일 전부)에
    # 동시에 걸쳐 있다. 예전에는 그 노드에 OHT가 있으면 걸쳐있는 레일
    # 전부에 "1대 전부"로 중복 카운트해서, 실측 검증 결과 점유율이 150%
    # 까지 나오는 문제가 있었다(분기점 노드 하나를 3개 레일이 공유하면
    # 실제로는 그 3개 레일에 나눠 배분돼야 할 "자리 1개"가 3배로 잡힘).
    # 지금은 그 노드의 점유량 1을 걸쳐있는 레일 수만큼 정확히 나눠(가중치
    # 1/n) 각 레일에 배분하고, 분모(레일 수용량)도 똑같은 가중치로 계산
    # 한다 - 분자·분모의 단위를 맞추면, 실제 시뮬레이션처럼 "한 노드에
    # OHT 1대만 있다"는 조건에서는 비율이 수학적으로 절대 100%를 넘을 수
    # 없다(30회 무작위 검증으로 확인됨). 분기점이 아닌 일반 노드는
    # 어차피 레일 1개에만 속하므로 가중치가 항상 1이라 기존과 동일하다.
    NODE_WEIGHT = {n: 1.0 / len(sis) for n, sis in NODE_TO_SEGMENTS.items()}
    RAIL_CAPACITY = [0.0] * len(RAIL_SEGMENTS)
    for _si, _seg in enumerate(RAIL_SEGMENTS):
        for _n in _seg:
            RAIL_CAPACITY[_si] += NODE_WEIGHT[_n]
    RAIL_CAPACITY = [max(c, 1e-6) for c in RAIL_CAPACITY]  # 0 나눗셈 방지(사실상 발생 안 함)

    # [뷰포트 기준 렌더링] 예전에는 여기서 RAIL_SCREEN_PTS(고정 저해상도
    # 픽셀 좌표)를 미리 구워뒀지만, 지금은 build_static_layer(cam, ...)가
    # 매번 현재 카메라 배율로 sx()/sy_pil() 결과를 직접 변환해서 그리므로
    # 더 이상 필요 없다.
    print(f"[레일 분할] 분기점 {_n_junctions}개 기준으로 레일 {len(RAIL_SEGMENTS)}개 구성")


# ----------------------------------------------------------------------
# 정적 레이어(레일 + 레일정체 오버레이 + EQ idle 오버레이 + STK) 캐시
# ----------------------------------------------------------------------
# [뷰포트 기준 렌더링] 예전에는 이 레이어들을 고정 저해상도(IMG_W x IMG_H,
# SCALE 기준)로 딱 한 번 구워두고 화면에 낼 때만 crop 후 확대(pygame.
# transform.scale)했다. 그러면 확대할수록 이미 래스터화된 픽셀을 그냥
# 늘리는 셈이라 계단현상이 생긴다.
# 지금은 build_static_layer(cam, ...)가 "현재 카메라(줌/팬) 배율"로 선/원을
# 매번 그 배율에 맞춰 다시 그린다 - 즉 항상 창 해상도 그대로(1픽셀=1픽셀)
# 그려지므로 아무리 확대해도 흐려지지 않는다. 다만 이 레이어는 트랙
# 7,785개 선분처럼 무거운 정적 지오메트리를 포함하므로, 매 애니메이션
# 프레임마다 다시 굽지 않고 "카메라가 바뀌었을 때" 또는 "정체/idle 상태
# 갱신 주기(RAIL_CONGESTION_REFRESH_SEC)가 됐을 때"만 다시 굽고 캐시해서
# 재사용한다(줌/팬 중이 아니면 시뮬레이션이 아무리 오래 돌아도 재계산 없음).
_STATIC_CACHE = {"img": None, "key": None, "t": -1e18}


def draw_region_tint(img, cam):
    """SHOW_REGION_TINT=True일 때, 베이(파란)/중앙통로(초록)/합류부(보라)
    구역을 반투명 띠로 깔아준다. 레일보다 먼저(아래에) 그려서 레일/정체
    색이 항상 위에 또렷하게 보이게 한다. 합류부는 베이 색 위에 다시
    덧칠하는 형태라(같은 노드가 베이이면서 동시에 합류부이기도 함),
    "여기가 특히 병목 구간"이라는 걸 강조하기 위해 맨 나중에 그린다.
    [끊김 없음] 섹션의 node_ids 순서를 따라가는 대신, 실제 그래프 간선
    (BAY_TINT_SEGMENTS/CENTER_TINT_SEGMENTS - 양 끝이 모두 해당 구역 노드인
    간선만 모듈 로드 시 미리 걸러둔 것)을 하나씩 그린다. 사용자 제공
    섹션 데이터(731개)가 커버 못 하는 노드(베이 양 끝의 짧은 U턴 연결부 등,
    build_section_regions.py의 "갭 메우기" 참고)도 BAY_TINT_NODES/
    CENTER_TINT_NODES에 이미 포함돼 있으므로, 실제 트랙이 이어진 그대로
    빈틈없이 칠해진다."""
    if not BAY_TINT_SEGMENTS and not CENTER_TINT_SEGMENTS and not MERGE_TINT_SEGMENTS:
        return
    overlay = Image.new("RGBA", img.size, (0, 0, 0, 0))
    odraw = ImageDraw.Draw(overlay)
    width = max(2, round(REGION_TINT_WIDTH * cam.scale))
    for a, b in BAY_TINT_SEGMENTS:
        p1 = cam.pt(sx(NODE_XY[a][0]), sy_pil(NODE_XY[a][1]))
        p2 = cam.pt(sx(NODE_XY[b][0]), sy_pil(NODE_XY[b][1]))
        odraw.line([p1, p2], fill=REGION_BAY_COLOR, width=width, joint="curve")
    for a, b in CENTER_TINT_SEGMENTS:
        p1 = cam.pt(sx(NODE_XY[a][0]), sy_pil(NODE_XY[a][1]))
        p2 = cam.pt(sx(NODE_XY[b][0]), sy_pil(NODE_XY[b][1]))
        odraw.line([p1, p2], fill=REGION_CENTER_COLOR, width=width, joint="curve")
    merge_width = max(3, round(REGION_TINT_WIDTH * 1.4 * cam.scale))  # 살짝 더 굵게 - 강조용
    for _mid, _segs in MERGE_TINT_SEGMENTS.items():
        for a, b in _segs:
            p1 = cam.pt(sx(NODE_XY[a][0]), sy_pil(NODE_XY[a][1]))
            p2 = cam.pt(sx(NODE_XY[b][0]), sy_pil(NODE_XY[b][1]))
            odraw.line([p1, p2], fill=REGION_MERGE_COLOR, width=merge_width, joint="curve")
    img.paste(overlay, (0, 0), overlay)


def draw_buffer_ports(draw, cam):
    """버퍼 포트(STK 54개 + STB 15,777개)를 현재 카메라 기준으로 그린다.
    [맵 교체] 개수가 커서 화면 밖 포트는 좌표 변환 후 즉시 건너뛴다.
    build_static_layer()와 render_frame()/render_replay_frame() 세 곳에서
    같은 로직을 쓰므로 함수로 뺐다(예전엔 세 곳에 같은 루프가 복붙돼
    있어서 표시 방식을 바꿀 때마다 어긋날 위험이 있었다)."""
    stk_r = max(1.0, STK_R * cam.scale)
    for e in STK_PORTS:
        px, py = cam.pt(sx(e["x"]), sy_pil(e["y"]))
        if -20 <= px <= cam.w + 20 and -20 <= py <= cam.h + 20:
            draw.ellipse((px - stk_r, py - stk_r, px + stk_r, py + stk_r),
                         fill=STK_COLOR)
    if not SHOW_STB:
        return
    stb_r = max(1.0, STB_R * cam.scale)
    for e in STB_PORTS:
        px, py = cam.pt(sx(e["x"]), sy_pil(e["y"]))
        if -20 <= px <= cam.w + 20 and -20 <= py <= cam.h + 20:
            draw.ellipse((px - stb_r, py - stb_r, px + stb_r, py + stb_r),
                         fill=STB_COLOR)


def draw_node_dots(draw, cam):
    """SHOW_NODE_DOTS=True일 때 모든 주행노드를 점으로 찍는다. 색 우선순위는:
      1) 합류 지점 자체(MERGE_POINT_NODES, in-degree>1)  - 주황
      2) 합류대기 진입측(MERGE_WAIT_NODES, 꺾여 들어오는 쪽) - 빨강
      3) 우선(직진) 진입측(MERGE_PRIORITY_NODES)          - 파랑
      4) 그 외 N갈래 이상 분기점(out-degree, 방법론 3-1번 "분기점")  - 보라
      5) 일반 노드 - 검정
    1~3은 agv_fab_sim_mini.py의 합류부 앞뒤 역할 구분을 그대로 옮긴 것이고,
    4는 4단계 재탐색이 재탐색 지점으로 삼는 JUNCTION_NODES와 같은 기준이다."""
    node_r = max(1.0, NODE_DOT_R * cam.scale)
    junction_r = max(1.0, JUNCTION_DOT_R * cam.scale)
    for node_id in NODE_LIST:
        if node_id in MERGE_POINT_NODES:
            color, r = MERGE_POINT_COLOR, junction_r
        elif node_id in MERGE_WAIT_NODES:
            color, r = MERGE_WAIT_COLOR, junction_r
        elif node_id in MERGE_PRIORITY_NODES:
            color, r = MERGE_PRIORITY_COLOR, junction_r
        elif len(OUT_LINKS.get(node_id, ())) >= JUNCTION_DOT_MIN_BRANCHES:
            color, r = JUNCTION_DOT_COLOR, junction_r
        else:
            color, r = NODE_DOT_COLOR, node_r
        x, y = NODE_XY[node_id]
        px, py = cam.pt(sx(x), sy_pil(y))
        draw.ellipse((px - r, py - r, px + r, py + r), fill=color)


def build_static_layer(cam, occupied_nodes, idle_nodes):
    """현재 카메라(cam) 배율로 레일 + 레일정체 오버레이 + EQ idle
    반투명 오버레이 + STK를 처음부터 다시 그려 반환한다. 선 굵기/원 반지름도
    cam.scale에 비례해서 키우기 때문에, 확대해도 트랙 두께 같은 비율로
    보인다(벡터 확대와 동일한 효과)."""
    img = Image.new("RGB", (cam.w, cam.h), BG_COLOR)

    # [구역 틴트] 레일을 그리기 전에 먼저 깔아서, 레일/정체 색이 항상
    # 이 위에 또렷하게 보이도록 한다.
    if SHOW_REGION_TINT:
        draw_region_tint(img, cam)

    draw = ImageDraw.Draw(img)

    rail_w = max(1, round(RAIL_WIDTH * cam.scale))
    for seg in SEGMENTS:
        pts = [cam.pt(sx(x), sy_pil(y)) for x, y in seg]
        draw.line(pts, fill=RAIL_COLOR, width=rail_w, joint="curve")

    if SHOW_RAIL_CONGESTION and RAIL_SEGMENTS:
        # [정밀화] 노드 하나의 점유량(1)을 그 노드가 걸쳐있는 레일 수만큼
        # NODE_WEIGHT로 나눠서 더한다(분기점 중복 카운트 방지).
        counts = [0.0] * len(RAIL_SEGMENTS)
        for n in occupied_nodes:
            w = NODE_WEIGHT.get(n)
            if w is None:
                continue
            for si in NODE_TO_SEGMENTS.get(n, ()):
                counts[si] += w

        rail_w_congested = max(1, round(RAIL_LINE_WIDTH_CONGESTED * cam.scale))
        rail_w_normal = max(1, round(RAIL_LINE_WIDTH_NORMAL * cam.scale))
        for si, seg in enumerate(RAIL_SEGMENTS):
            if len(seg) < 2:
                continue
            ratio = counts[si] / RAIL_CAPACITY[si]
            color = congestion_color(ratio)
            width = rail_w_congested if ratio >= RAIL_CONGESTION_LOW else rail_w_normal
            pts = [cam.pt(sx(NODE_XY[n][0]), sy_pil(NODE_XY[n][1])) for n in seg]
            draw.line(pts, fill=color, width=width, joint="curve")

    if SHOW_EQ_IDLE:
        idle_color = MACHINE_STATUS_COLOR["empty"]
        idle_rgb = ImageColor.getrgb(idle_color) if isinstance(idle_color, str) else idle_color
        fill = tuple(idle_rgb[:3]) + (EQ_IDLE_ALPHA,)
        overlay = Image.new("RGBA", (cam.w, cam.h), (0, 0, 0, 0))
        odraw = ImageDraw.Draw(overlay)
        eq_r = max(1.0, EQ_R * cam.scale)
        for node_id in idle_nodes:
            x, y = NODE_XY[node_id]
            px, py = cam.pt(sx(x), sy_pil(y))
            odraw.ellipse((px - eq_r, py - eq_r, px + eq_r, py + eq_r), fill=fill)
        img.paste(overlay, (0, 0), overlay)

    draw_buffer_ports(draw, cam)

    if SHOW_NODE_DOTS:
        draw_node_dots(draw, cam)

    # [성능] EQ/STK 포트 번호(라벨)도 여기서, 즉 카메라가 바뀌거나 갱신
    # 주기가 됐을 때만 다시 그려서 캐시에 포함시킨다. 이전엔 render_frame()/
    # render_replay_frame()에서 "확대 중이면 매 애니메이션 프레임마다"
    # EQ 3,944개 + 버퍼 15,831개를 순회하며 텍스트를 다시 그렸는데, 이게
    # 확대 상태에서 렉이 걸리던 실제 원인이었다. 이제는 레일과 똑같이
    # "카메라가 안 바뀌면 다시 그리지 않는" 캐시 대상이라 시뮬레이션이
    # 돌아가는 동안(줌 고정 상태)에는 추가 비용이 전혀 없다.
    if SHOW_PORT_ID and cam.scale >= PORT_ID_LABEL_MIN_SCALE:
        draw_port_labels(draw, cam)

    # [요청 반영] T로 구역 틴트를 켰을 때, 어느 베이가 몇 번인지/중앙통로가
    # 위쪽(1)인지 아래쪽(2)인지 바로 알 수 있게 번호를 같이 표시한다.
    # 틴트가 꺼져있으면(SHOW_REGION_TINT=False) 라벨도 같이 숨긴다 - 틴트
    # 없이 번호만 둥둥 떠있으면 오히려 헷갈리므로.
    if SHOW_REGION_TINT and cam.scale >= REGION_ID_LABEL_MIN_SCALE:
        draw_region_id_labels(draw, cam)

    return img


def get_static_layer(cam, now, occupied_nodes_fn, idle_nodes_fn):
    """캐시된 정적 레이어를 돌려주되, 카메라가 바뀌었거나(줌/팬/창크기
    변경) 정체/idle 갱신 주기(RAIL_CONGESTION_REFRESH_SEC)가 지났거나
    표시 토글(SHOW_PORT_ID 등)이 바뀌었으면 새로 굽는다."""
    # [토글 지원] SHOW_PORT_ID는 런타임에 키보드로 켜고 끌 수 있다(L 키,
    # main()/replay_main() 참고). 이 값이 카메라 키에는 안 잡히므로, 토글
    # 직후에도 캐시가 그대로 재사용되지 않도록 별도로 키에 포함시킨다.
    key = (cam.key(), SHOW_PORT_ID, SHOW_REGION_TINT,
           SHOW_NODE_DOTS, JUNCTION_DOT_MIN_BRANCHES)
    stale = (now - _STATIC_CACHE["t"]) >= RAIL_CONGESTION_REFRESH_SEC
    if _STATIC_CACHE["img"] is None or _STATIC_CACHE["key"] != key or stale:
        occ = occupied_nodes_fn() if SHOW_RAIL_CONGESTION else []
        idle = idle_nodes_fn() if SHOW_EQ_IDLE else []
        _STATIC_CACHE["img"] = build_static_layer(cam, occ, idle)
        _STATIC_CACHE["key"] = key
        _STATIC_CACHE["t"] = now
    return _STATIC_CACHE["img"]


# ----------------------------------------------------------------------
# 통계/범례 패널 (줌의 영향을 받지 않는 고정 크기 영역)
# ----------------------------------------------------------------------
FONT_STATS = FONT_LEGEND = FONT_OHT_ID = FONT_PORT_ID = FONT_REGION_ID = None
STATS_H = (96 if SHOW_RAIL_CONGESTION else 76) if SHOW_STATS_TEXT else 0

if not HEADLESS:
    FONT_STATS = _load_font(14)
    FONT_LEGEND = _load_font(11)
    FONT_OHT_ID = _load_font(13)
    FONT_PORT_ID = _load_font(10)
    FONT_REGION_ID = _load_font(20)  # 베이/중앙통로 번호는 넓은 구역 위에 크게 표시


def draw_region_id_labels(draw, cam):
    """T로 구역 틴트가 켜져 있을 때, 각 베이의 무게중심 위치에
    "베이N", 각 중앙통로의 무게중심 위치에 "중앙통로N", 각 합류부
    의 무게중심 위치에 "합류부N"을 그린다. build_static_layer()
    안에서만 호출되므로(정적 레이어 캐시에 포함됨) render_frame()처럼
    매 애니메이션 프레임마다 다시 불릴 일이 없다."""
    for bid, (wx, wy) in BAY_ID_LABEL_POS.items():
        px, py = cam.pt(sx(wx), sy_pil(wy))
        if -60 <= px <= cam.w + 60 and -20 <= py <= cam.h + 20:
            draw.text((px, py), f"베이{bid}", font=FONT_REGION_ID,
                       fill=(30, 60, 140), stroke_width=2, stroke_fill="white",
                       anchor="mm")
    for cid, (wx, wy) in CORRIDOR_ID_LABEL_POS.items():
        px, py = cam.pt(sx(wx), sy_pil(wy))
        if -100 <= px <= cam.w + 100 and -20 <= py <= cam.h + 20:
            draw.text((px, py), f"중앙통로{cid}", font=FONT_REGION_ID,
                       fill=(20, 110, 60), stroke_width=2, stroke_fill="white",
                       anchor="mm")
    # [합류부] 병목 강조용이라 베이/중앙통로 라벨보다 살짝 위(오프셋)에
    # 그려서 겹칠 때도 둘 다 읽을 수 있게 한다.
    for mid, (wx, wy) in MERGE_ID_LABEL_POS.items():
        px, py = cam.pt(sx(wx), sy_pil(wy))
        if -80 <= px <= cam.w + 80 and -40 <= py <= cam.h + 40:
            draw.text((px, py - 24), f"합류부{mid}", font=FONT_REGION_ID,
                       fill=(140, 20, 130), stroke_width=2, stroke_fill="white",
                       anchor="mm")


def draw_stats_strip(frame, vp, stats_text):
    """frame(창 전체 크기) 하단 vp.stats_h 픽셀에 숫자 통계 1줄 + 범례를
    그린다. 지도 부분과 달리 항상 고정 픽셀 크기라 줌해도 안 흐려지고,
    확대해서 지도를 봐도 이 패널은 계속 원래 크기로 보인다."""
    if not SHOW_STATS_TEXT:
        return
    draw = ImageDraw.Draw(frame)
    y0 = vp.map_win_h
    draw.rectangle((0, y0, vp.win_w, vp.win_h), fill=BG_COLOR)
    draw.text((MARGIN, y0 + 6), stats_text, font=FONT_STATS, fill="#222222")

    _sw = 8
    _y1 = y0 + 34  # OHT 줄
    _y2 = y0 + 54  # Machine 줄

    _x = MARGIN
    draw.text((_x, _y1 - 7), "OHT:", font=FONT_LEGEND, fill="#333333")
    _x += 34
    _oht_legend = [(IDLE_COLOR, "Idle"), (EMPTY_COLOR, "UnLoaded"), (LOADED_COLOR, "Loaded")]
    for _color, _label in _oht_legend:
        draw.rectangle((_x, _y1 - _sw // 2, _x + _sw, _y1 + _sw // 2), fill=_color)
        draw.text((_x + _sw + 3, _y1 - 7), _label, font=FONT_LEGEND, fill="#333333")
        # [간격] 라벨 길이를 고정 숫자로 어림잡으면 "UnLoaded"처럼 길어진
        # 텍스트에서 다음 항목과 겹치는 문제가 생긴다. textlength()로 실제
        # 렌더링 폭을 재서 간격을 계산하면 라벨이 바뀌어도 안 겹친다.
        _label_w = draw.textlength(_label, font=FONT_LEGEND)
        _x += _sw + 3 + _label_w + 15

    _machine_legend = [("empty", "Idle"), ("reserved", "Reserved"),
                        ("processing", "Processing"), ("done", "Done")]
    _x = MARGIN
    draw.text((_x, _y2 - 7), "Machine:", font=FONT_LEGEND, fill="#333333")
    _x += 54
    for _status, _label in _machine_legend:
        draw.rectangle((_x, _y2 - _sw // 2, _x + _sw, _y2 + _sw // 2),
                        fill=MACHINE_STATUS_COLOR[_status])
        _x += _sw + 3
        draw.text((_x, _y2 - 7), _label, font=FONT_LEGEND, fill="#333333")
        _label_w = draw.textlength(_label, font=FONT_LEGEND)
        _x += _label_w + 15

    if SHOW_RAIL_CONGESTION:
        _y3 = y0 + 74  # Rail 정체 줄
        _x = MARGIN
        draw.text((_x, _y3 - 7), "Rail:", font=FONT_LEGEND, fill="#333333")
        _x += 34
        _rail_legend = [(RAIL_COLOR_GREEN, "널널"),
                         (RAIL_COLOR_YELLOW, "보통"),
                         (RAIL_COLOR_RED, "정체")]
        for _color, _label in _rail_legend:
            draw.rectangle((_x, _y3 - _sw // 2, _x + _sw, _y3 + _sw // 2), fill=_color)
            _x += _sw + 3
            draw.text((_x, _y3 - 7), _label, font=FONT_LEGEND, fill="#333333")
            _label_w = draw.textlength(_label, font=FONT_LEGEND)
            _x += _label_w + 15

# ----------------------------------------------------------------------
# 노드 예약(lock) 시스템 - 충돌회피
# [2500대 대응] busy-poll(0.2s) 대신 release() 시점에 대기 큐의 첫 OHT만
# 깨우는 event-driven 방식. NODE_WAITERS[node] 는 그 노드를 기다리는
# OHT들의 FIFO 큐.
# ----------------------------------------------------------------------
NODE_HOLDER = {nid: None for nid in NODE_LIST}   # node_id -> OHT or None
NODE_WAITERS = {nid: [] for nid in NODE_LIST}    # node_id -> [OHT, ...] (대기 큐)

# ----------------------------------------------------------------------
# 베이 고리(Bay Ring) 진입 제어 - 순환 교착(ring deadlock) 사전 방지
# ----------------------------------------------------------------------
# [문제] 노드락 + 차간거리만으로는 "그래프의 작은 순환 구간이 차량으로
# 가득 차서 모두가 앞차를 기다리는" 교착을 막을 수 없다. 축소 팹 180대
# 실험에서 실제로 발생했고, 기존 트리거(2) 강제우회는 "막힌 뒤" 대응이라
# 소용이 없었다 - 고리 안 차량 대부분은 나가는 링크가 하나뿐인 직선구간에
# 있어 우회 선택지가 없고, 역주행은 불가하다.
#
# [방법론 - 사용자 확정]
#   1) 관리 대상 = 베이 고리만. 설비 베이의 "평행한 세로 차선 2개 + 양 끝
#      U턴/차선변경"으로 닫힌 순환 구간(도면상 길쭉한 육각형 모양)이다.
#      중앙통로/외곽 순환은 관리하지 않는다.
#   2) 자동 탐지: 모든 링크 (u->v)에 대해 "v에서 u로 돌아오는 최단경로 +
#      그 링크" = 그 링크를 포함하는 최단 순환을 구하고(RING_SEARCH_MAX_
#      LENGTH_M 이내), 노드집합이 같은 순환은 하나로 합친 뒤 아래 조건을
#      모두 만족하는 것만 베이 고리로 채택한다.
#        - 양 끝(BAY_RING_END_ZONE_M) 구간을 뺀 노드들의 x좌표가 정확히
#          2개 차선 열로 묶임(허용오차 BAY_RING_LANE_TOL_M) - 차선이 3개
#          이상이거나 사선이 섞인 구간은 제외
#        - 두 차선 간격(고리 폭) BAY_RING_WIDTH_M 범위 - 외곽 순환처럼
#          폭이 다른 구간 제외
#        - 고리 높이 BAY_RING_HEIGHT_M 범위 - 중앙통로~차선변경 한 구간
#        - 중앙통로 노드 BAY_RING_MAX_CORRIDOR_NODES개 이하 - 통로와 맞닿는
#          끝점만 허용
#      [검증] MiniFab에서 이 조건으로 사용자가 지정한 16개 구간과 정확히
#      일치하는 16개가 탐지된다(임계값은 이 맵 기하에 맞춘 값).
#   3) 고리 용량 = 노드 수 // 2 - RING_CAP_MARGIN.
#      [근거] 차간거리 체크(_path_clear_ahead)가 진입 노드뿐 아니라 그
#      다음 노드까지 비어 있어야 통과시키므로, 움직이는 차량 1대는 사실상
#      노드 2개를 쓴다. 노드 수의 절반이 차면 모든 차량이 "두 칸 앞이
#      막혔다"며 동시에 멈춘다(실측: 26노드 고리 13대에서 교착). 그래서
#      절반에서 여유분을 뺀다.
#   4) 고리 점유 대수 = 그 고리의 노드 락을 1개라도 쥔 OHT 수. 락 획득/
#      해제 시점에 증감하므로 판정과 예약이 같은 이벤트 안에서 일어나,
#      두 입구로 동시에 들어와 용량을 넘는 일이 없다.
#   5) 진입 판정: 다음 노드가 "아직 들어가 있지 않은 베이 고리"에 속하고
#      그 고리가 용량에 도달했으면 진입 불가.
#        - 지금 위치가 분기점이고 꽉 찬 고리로 가지 않는 다른 갈래가 있으면
#          -> 그 링크를 제외하고 재탐색해 다른 갈래로 강제 우회
#        - 아니면(분기점이 아니거나 모든 갈래가 막힘, 우회경로 없음)
#          -> 그 자리에서 대기(RING_WAIT_POLL 간격 재확인)
#      같은 노드에서 이미 우회를 시도했는데 재탐색 결과가 다시 같은 링크로
#      오면(목적지가 그 고리 안 등) 그 노드에서는 대기로 넘어간다(0초
#      재탐색 무한반복 방지).
#   6) 길 비켜주기(_yield_step)의 한 칸 이동은 막힌 작업 차량을 풀어주는
#      동작이라 판정에서 예외(대신 꽉 차지 않은 쪽 칸을 우선 고름).
#      고리 대기 차량은 정체 감시(트리거 2)가 건드리지 않는다 - 용량 제어에
#      의한 의도된 정지라 재탐색해도 같은 결과이고 계산만 낭비된다.
ENABLE_RING_CONTROL = True       # 끄면 기존과 완전히 같은 동작(비교 실험용)
RING_CAP_MARGIN = 2              # 고리 용량 = 노드 수 // 2 - 이 값
RING_WAIT_POLL = 0.5             # 진입 대기 시 재확인 간격(초)
RING_COUNT_IDLE = True           # 유휴(작업 없는) 차량도 점유 대수에 셈
RING_SEARCH_MAX_LENGTH_M = 80.0  # 순환 탐색 길이 상한(베이 고리는 40~50m)
BAY_RING_END_ZONE_M = 3.0        # 차선 판정에서 뺄 양 끝(U턴/차선변경) 구간
BAY_RING_LANE_TOL_M = 0.3        # 같은 차선으로 볼 x 허용오차
BAY_RING_WIDTH_M = (1.8, 2.6)    # 두 차선 간격 범위
BAY_RING_HEIGHT_M = (15.0, 25.0) # 고리 높이 범위
BAY_RING_MAX_CORRIDOR_NODES = 1  # 고리에 섞여도 되는 중앙통로 노드 수


def _is_bay_ring(node_set):
    xs = [NODE_XY[n][0] for n in node_set]
    ys = [NODE_XY[n][1] for n in node_set]
    width = (max(xs) - min(xs)) / UNIT_PER_METER
    height = (max(ys) - min(ys)) / UNIT_PER_METER
    if not (BAY_RING_WIDTH_M[0] <= width <= BAY_RING_WIDTH_M[1]):
        return False
    if not (BAY_RING_HEIGHT_M[0] <= height <= BAY_RING_HEIGHT_M[1]):
        return False
    if sum(1 for n in node_set if n in NODE_TO_CORRIDOR_ID) > BAY_RING_MAX_CORRIDOR_NODES:
        return False
    end = BAY_RING_END_ZONE_M * UNIT_PER_METER
    lo, hi = min(ys), max(ys)
    mid = sorted(NODE_XY[n][0] for n in node_set if lo + end < NODE_XY[n][1] < hi - end)
    if not mid:
        return False
    tol = BAY_RING_LANE_TOL_M * UNIT_PER_METER
    lanes = 1
    for a, b in zip(mid, mid[1:]):
        if b - a > tol:
            lanes += 1
    return lanes == 2


def _detect_rings():
    """베이 고리 자동 탐지. 반환: [{nodes, links, length, cap}] (왼->오, 위->아래)"""
    rings = {}
    limit_mm = RING_SEARCH_MAX_LENGTH_M * UNIT_PER_METER
    for li in range(len(LINK_A)):
        a, b = LINK_A[li], LINK_B[li]
        rest = limit_mm - LINK_LEN[li]
        if rest <= 0:
            continue
        ia, ib = NODE_IDX[a], NODE_IDX[b]
        dist, pred = dijkstra(GRAPH_CSR, indices=ib, return_predecessors=True,
                              limit=rest)
        if not np.isfinite(dist[ia]):
            continue
        seq = [ia]
        cur = ia
        while cur != ib:
            cur = pred[cur]
            seq.append(cur)
        seq.reverse()                      # b -> ... -> a
        node_seq = [NODE_LIST[i] for i in seq]
        key = frozenset(node_seq)
        if key in rings or not _is_bay_ring(key):
            continue
        links = [EDGE_LINK_IDX[(node_seq[k], node_seq[k + 1])]
                 for k in range(len(node_seq) - 1)] + [li]
        rings[key] = {"nodes": key, "links": links,
                      "length": float(dist[ia]) + LINK_LEN[li]}
    out = sorted(rings.values(),
                 key=lambda r: (round(min(NODE_XY[n][0] for n in r["nodes"]), -3),
                                -max(NODE_XY[n][1] for n in r["nodes"])))
    for r in out:
        r["cap"] = max(1, len(r["nodes"]) // 2 - RING_CAP_MARGIN)
    return out


RINGS = _detect_rings() if ENABLE_RING_CONTROL else []
NODE_RINGS = {nid: () for nid in NODE_LIST}
for _ri, _r in enumerate(RINGS):
    for _n in _r["nodes"]:
        NODE_RINGS[_n] = NODE_RINGS[_n] + (_ri,)
RING_MEMBERS = [set() for _ in RINGS]      # 고리 -> 그 고리 노드락을 쥔 OHT
HELD_NODES = {}                            # OHT -> 쥐고 있는 노드 집합
RING_STATS = {"detour": 0, "wait_events": 0, "wait_time": 0.0,
              "yield_exempt": 0}
if RINGS:
    _caps = [r["cap"] for r in RINGS]
    _ns = [len(r["nodes"]) for r in RINGS]
    print(f"[베이 고리 진입 제어] {'ON' if ENABLE_RING_CONTROL else 'OFF'} - "
          f"베이 고리 {len(RINGS)}개 탐지 (노드 {min(_ns)}~{max(_ns)}개, "
          f"용량 {min(_caps)}~{max(_caps)}대 = 노드수//2-{RING_CAP_MARGIN})")
else:
    print(f"[베이 고리 진입 제어] {'ON' if ENABLE_RING_CONTROL else 'OFF'} - 관리 고리 없음")


def _ring_occupancy(ri):
    if RING_COUNT_IDLE:
        return len(RING_MEMBERS[ri])
    return sum(1 for o in RING_MEMBERS[ri] if o.job is not None)


def ring_entry_blocked(oht, node_id):
    """oht가 node_id로 들어가면 새로 진입하게 되는 베이 고리 중 이미 용량에
    도달한 고리가 있으면 True."""
    if not ENABLE_RING_CONTROL:
        return False
    for ri in NODE_RINGS.get(node_id, ()):
        if oht in RING_MEMBERS[ri]:
            continue
        if _ring_occupancy(ri) >= RINGS[ri]["cap"]:
            return True
    return False


def _ring_on_acquire(node_id, oht):
    HELD_NODES.setdefault(oht, set()).add(node_id)
    for ri in NODE_RINGS.get(node_id, ()):
        RING_MEMBERS[ri].add(oht)


def _ring_on_release(node_id, oht):
    held = HELD_NODES.get(oht)
    if held is not None:
        held.discard(node_id)
    for ri in NODE_RINGS.get(node_id, ()):
        if not held or not any(n in RINGS[ri]["nodes"] for n in held):
            RING_MEMBERS[ri].discard(oht)


def reset_ring_state():
    for s in RING_MEMBERS:
        s.clear()
    HELD_NODES.clear()
    for k in RING_STATS:
        RING_STATS[k] = 0 if k != "wait_time" else 0.0


# ----------------------------------------------------------------------
# 합류부 진입 우선권 (Adaptive Time-based Priority)
# ----------------------------------------------------------------------
# 회사 슬라이드 "합류 제어 - 진입경로의 합류부 진입 우선권 점유 시간 가변
# 설정"을 그대로 옮긴 것. Time Window(미래 슬롯 예약)도 Platoon(차량군
# 결합주행)도 아니고, "방금 그 합류점을 통과한 OHT의 진입경로(링크)가
# 일정 시간 동안 우선권을 그대로 물려받는" 반응적 방식이다.
#   - 기본은 대기노드 도착순(FCFS) - NODE_WAITERS의 자연스러운 순서.
#   - 단, 우선권을 쥔 링크에서 대기 중인 OHT가 있으면, 그 OHT가 먼저
#     도착한 다른 링크의 OHT보다 먼저 통과한다(release()에서 구현).
#   - 점유시간 안에 그 링크로 후속 OHT가 없으면 우선권은 그냥 안 쓰이고
#     사라진다(따로 리셋할 필요 없음 - 만료시각을 지나면 자동으로 무효).
MERGE_HELD_LINK = {}       # merge_node -> 우선권을 쥔 진입 링크 인덱스
MERGE_HOLD_EXPIRE = {}     # merge_node -> 그 우선권이 소멸하는 시각(sim time)
MERGE_PRIORITY_WINDOW = 5.0   # 우선권 점유 시간(초) - 슬라이드의 "가변 설정" 대상
ENABLE_MERGE_PRIORITY = True  # 끄면 순수 FCFS(도착순)로 동작 - 효과 비교용

# priority_grants: 우선권으로 통과 승인된 횟수
# overtakes      : 그 중 먼저 도착한 다른 경로 차량을 실제로 앞지른 횟수
# fcfs_grants    : 우선권 없이 도착순으로 승인된 횟수
MERGE_STATS = {"priority_grants": 0, "overtakes": 0, "fcfs_grants": 0}

# [용량 산정용] 어떤 노드에서 락 대기가 자주 발생하는지 세는 카운터.
# 값이 큰 노드일수록 트래픽이 몰리는 병목 후보 지점이다.
WAIT_COUNTS = Counter()

# [정체(stall) 추적] OHT가 다음 노드에 못 들어가서 그 자리에 계속 멈춰있는
# 구간(락 대기 + follow-distance 대기 통틀어)이 STALL_THRESHOLD_SECONDS
# 이상이면 "정체 이벤트" 1건으로 기록한다. 좌표(node)와 (EQ/STK 포트면)
# zone까지 같이 남겨서 나중에 노드/베이 단위로 집계할 수 있게 한다.
STALL_THRESHOLD_SECONDS = 10.0
STALL_EVENTS = []  # [{"oht": id, "node": str, "zone": str|None,
                   #   "t_start": float, "duration": float}, ...]
NODE_ZONE = {e["node"]: e.get("zone") for e in EQUIPMENT}  # EQ/STK 노드만 존재

# [구역별 정체 누적] STALL_EVENTS는 STALL_THRESHOLD_SECONDS 이상인 "굵직한"
# 정체만 이벤트로 남기지만, 구역(베이/중앙통로/합류부) 지표는 짧고 잦은
# 정체까지 다 반영해야 실제 병목 위치가 드러난다. 그래서 _travel_path()가
# 정체를 겪을 때마다 그 위치의 구역에 시간을 직접 누적한다.
# 각 dict: region_key -> {"stall": 초, "lock": 초, "follow": 초, "events": 건수}
def _new_region_stat():
    return {"stall": 0.0, "lock": 0.0, "follow": 0.0, "events": 0}


BAY_STALL_STATS = {}       # bay_id -> stat
CORRIDOR_STALL_STATS = {}  # corridor_id -> stat
MERGE_STALL_STATS = {}     # merge_id -> stat
UNASSIGNED_STALL_STATS = _new_region_stat()  # 어느 구역에도 안 속한 노드에서의 정체


def _accumulate_region_stall(node_id, stall_dur, lock_dur, follow_dur):
    """정체가 일어난 노드(node_id)가 속한 베이/중앙통로/합류부에 정체 시간을
    누적한다. 한 노드가 베이이면서 동시에 합류부일 수 있으므로(합류부는 베이
    위에 겹쳐 정의됨) 해당되는 곳에 각각 더한다 - 즉 합류부 집계는 베이
    집계를 대체하는 게 아니라 "그 중 합류부에 해당하는 몫"을 따로 보는 뷰다."""
    bid = NODE_TO_BAY_ID.get(node_id)
    cid = NODE_TO_CORRIDOR_ID.get(node_id)
    mids = NODE_TO_MERGE_ID.get(node_id, ())

    matched = False
    if bid is not None:
        st = BAY_STALL_STATS.setdefault(bid, _new_region_stat())
        st["stall"] += stall_dur; st["lock"] += lock_dur
        st["follow"] += follow_dur; st["events"] += 1
        matched = True
    if cid is not None:
        st = CORRIDOR_STALL_STATS.setdefault(cid, _new_region_stat())
        st["stall"] += stall_dur; st["lock"] += lock_dur
        st["follow"] += follow_dur; st["events"] += 1
        matched = True
    for mid in mids:
        st = MERGE_STALL_STATS.setdefault(mid, _new_region_stat())
        st["stall"] += stall_dur; st["lock"] += lock_dur
        st["follow"] += follow_dur; st["events"] += 1
    if not matched:
        UNASSIGNED_STALL_STATS["stall"] += stall_dur
        UNASSIGNED_STALL_STATS["lock"] += lock_dur
        UNASSIGNED_STALL_STATS["follow"] += follow_dur
        UNASSIGNED_STALL_STATS["events"] += 1

# [용량 산정용] 완료된 작업(자재 1건) 단위 통계. 각 원소는
# {"cycle": 총 운반시간, "wait": 노드락+자원대기 시간,
#  "travel": 실주행 시간, "process": 설비 가공 시간, "t_end": 완료 시각}
# (시간류는 초, t_end는 시뮬레이션 시작 기준 경과초). from-to는
# Manager._assign_job()이 매 건 랜덤으로 뽑으므로 이 리스트는 여러 무작위
# 표본에 대한 통계 산출용 데이터가 된다.
JOB_STATS = []


def merge_priority_link(node_id, now):
    """지금 이 합류점에서 우선권을 쥐고 있는 진입 링크. 점유시간이 지났으면
    None(=우선권 소멸). 따로 리셋하지 않고 만료시각만 보면 되므로, 슬라이드의
    "후속 OHT가 없으면 우선권은 사라짐"이 자동으로 성립한다."""
    if MERGE_HOLD_EXPIRE.get(node_id, -1.0) <= now:
        return None
    return MERGE_HELD_LINK.get(node_id)


def grant_merge_priority(node_id, link_idx, now):
    """합류부 통과를 승인받은 OHT의 진입경로(link_idx)에 우선권을 준다.
    이미 그 링크가 우선권을 쥐고 있었다면 점유시간이 갱신되어, 같은 경로로
    연속해서 들어오는 OHT들이 계속 먼저 통과하게 된다."""
    if not ENABLE_MERGE_PRIORITY or link_idx is None:
        return
    if node_id not in MERGE_POINT_NODES:
        return
    MERGE_HELD_LINK[node_id] = link_idx
    MERGE_HOLD_EXPIRE[node_id] = now + MERGE_PRIORITY_WINDOW


def try_acquire(node_id, oht):
    if NODE_HOLDER[node_id] is None:
        NODE_HOLDER[node_id] = oht
        _ring_on_acquire(node_id, oht)   # [고리 진입 제어] 점유 대수 갱신
        return True
    return False


def release(node_id, oht, now=None):
    """노드 락을 놓고, 대기 중인 다음 OHT 하나를 깨운다.

    [합류부 우선권] 이 노드가 합류점이고 아직 살아있는 우선권이 있으면,
    그 진입경로에서 기다리는 OHT를 먼저 깨운다(먼저 도착한 다른 경로의
    OHT보다 앞선다). 우선권이 없거나 만료됐거나 그 경로에 대기차량이
    없으면 원래대로 도착순(FCFS)으로 깨운다."""
    if NODE_HOLDER.get(node_id) is oht:
        NODE_HOLDER[node_id] = None
        _ring_on_release(node_id, oht)   # [고리 진입 제어] 점유 대수 갱신
        waiters = NODE_WAITERS.get(node_id)
        if not waiters:
            return

        if now is not None and node_id in MERGE_POINT_NODES:
            held = merge_priority_link(node_id, now)
            if held is not None:
                for i, w in enumerate(waiters):
                    if (w.ispassive()
                            and EDGE_LINK_IDX.get((w.node, node_id)) == held):
                        MERGE_STATS["priority_grants"] += 1
                        if i > 0:
                            # 먼저 온 다른 경로 차량을 앞질렀다
                            MERGE_STATS["overtakes"] += 1
                        waiters.pop(i).activate()
                        return

        while waiters:
            nxt = waiters.pop(0)
            # 대기 중 다른 이유로 이미 활성화/종료된 OHT는 건너뜀
            if nxt.ispassive():
                MERGE_STATS["fcfs_grants"] += 1
                nxt.activate()
                break


def cancel_wait(node_id, oht):
    """대기 큐에서 빠진다(재탐색으로 이 노드를 더 이상 노리지 않을 때).
    남겨두면 release()가 이 OHT에게 깨우기 슬롯을 낭비한다."""
    waiters = NODE_WAITERS.get(node_id)
    if waiters and oht in waiters:
        waiters.remove(oht)


def _find_front_idle(node_id, max_depth=64):
    """node_id를 쥐고 있는 유휴 차량에서 출발해, 그 앞을 또 막고 있는
    유휴 차량의 사슬을 계속 따라가 맨 앞(더 이상 다른 유휴 차에 막히지
    않는) 차량을 찾는다.

    체인 중간 차량은 비키려 해도 바로 앞이 또 다른 유휴 차로 막혀있어서
    실패하고(YIELD_RETRY_INTERVAL만큼 허비), 그게 풀리기까지 사슬 길이만큼
    재시도가 반복되는 게 느렸다. 맨 앞 차부터 먼저 비키게 하면 한 번에
    사슬 전체를 풀 수 있다."""
    cur = NODE_HOLDER.get(node_id)
    if cur is None or cur.job is not None or cur.state != OHT_IDLE:
        return None
    visited = {cur.node}
    for _ in range(max_depth):
        neighbors = ADJ.get(cur.node) or []
        next_idle = None
        for n in neighbors:
            if n in visited:
                continue
            h = NODE_HOLDER.get(n)
            if h is not None and h.job is None and h.state == OHT_IDLE:
                next_idle = h
                break
        if next_idle is None:
            return cur          # 앞이 비었거나 작업차 - cur가 맨 앞
        visited.add(next_idle.node)
        cur = next_idle
    return cur                  # 깊이 한도 도달(방어적) - 마지막 찾은 것


def request_yield(node_id, blocked_oht):
    """node_id를 쥐고 있는 OHT가 '작업 없는 유휴 차량'이면 비켜달라고 한다.
    실제 OCS가 Idle 차량에게 내리는 '길 비켜주기 Route 명령'에 해당한다.
    유휴 차량은 이 요청을 받으면 대기 hold를 중단하고 즉시 한 칸 이동한다.

    [체인 릴레이] node_id의 바로 그 홀더가 아니라, _find_front_idle()로
    찾은 사슬의 맨 앞 차에게 요청을 보낸다 - 중간 차는 앞이 막혀 비킬 수
    없으므로 요청해봐야 소용없다."""
    holder = NODE_HOLDER.get(node_id)
    if holder is None or holder is blocked_oht:
        return
    if holder.job is not None or holder.state != OHT_IDLE:
        return                      # 작업 중인 차량에게는 비키라고 할 수 없다
    front = _find_front_idle(node_id) or holder
    if front._yield_request:
        return                      # 이미 요청받은 상태
    front._yield_request = True
    IDLE_STATS["yield_requests"] += 1
    if front.ispassive():
        front.activate()


def wait_for_node(node_id, oht):
    """node_id 락을 못 얻었을 때 대기 큐에 등록하고 passivate.
    release() 쪽에서 activate() 해줄 때까지 이벤트 소비 없이 잠든다."""
    request_yield(node_id, oht)     # 막고 있는 게 유휴 차량이면 비켜달라고 통보
    if oht not in NODE_WAITERS[node_id]:
        NODE_WAITERS[node_id].append(oht)
        WAIT_COUNTS[node_id] += 1  # 혼잡도 집계 (용량 산정 병목 판정용)
    return oht.passivate()


# ----------------------------------------------------------------------
# 설비 리소스 (동시 1개 자재만 처리) + 상태
# ----------------------------------------------------------------------
EQ_RESOURCE = {}  # node_id -> sim.Resource (지연 생성)
EQ_STATUS = {e["node"]: "empty" for e in EQ_PORTS}  # node_id -> status
EQ_ACTIVE = set()  # empty가 아닌 node_id 집합. [참고] STK/Machine을 화면 맨
                    # 위에 표시하기 위해 render_frame()이 이제 EQ_PORTS 전체를
                    # 순회하므로 렌더링에는 더 이상 쓰이지 않지만, 가벼운
                    # 상태 추적용으로 계속 유지한다(다른 분석 코드가 참조할
                    # 수 있어 남겨둠).


def set_eq_status(node_id, status):
    """설비 상태 변경. 여기서는 tkinter 오브젝트를 전혀 만들지 않고
    상태값 + 활성집합만 갱신한다. 실제 그리기는 render_frame()이 매 프레임
    한 번에 PIL로 합성해서 처리한다 (아래 '단일 이미지 합성 렌더링' 섹션)."""
    EQ_STATUS[node_id] = status
    if status == "empty":
        EQ_ACTIVE.discard(node_id)
    else:
        EQ_ACTIVE.add(node_id)


def get_eq_resource(node_id):
    res = EQ_RESOURCE.get(node_id)
    if res is None:
        res = sim.Resource(f"EQ@{node_id}", capacity=1, monitor=False)
        EQ_RESOURCE[node_id] = res
    return res


# ----------------------------------------------------------------------
# 진행 통계
# ----------------------------------------------------------------------
class Manager(sim.Component):
    def setup(self):
        self.injected = 0
        self.finished = 0

    @property
    def wip(self):
        return self.injected - self.finished

    def create_job(self):
        """새 로트 하나를 만든다 - STK에서 출발해 최종적으로
        STK로 돌아오는 다단계 흐름(STK->[STB]->EQ->[STB]->STK)의 첫
        leg만 지금 Job으로 만들어 배정 경쟁에 넣는다. 나머지 leg는
        LOT_REGISTRY에 넣어두고, 현재 leg가 끝날 때마다 advance_lot()이
        하나씩 꺼내 다음 Job을 만든다.
        [고정 스케줄] 로트 내용(패턴/포트/긴급도)은 매번 새로 뽑지 않고
        JOB_SCHEDULE에서 순서대로 꺼낸다 - 같은 조건으로 다시 돌리면
        완전히 같은 로트가 같은 순서로 나온다."""
        entry = next_lot_entry()
        legs = build_lot_legs(entry)
        lot_id = new_lot_id()
        now = self.env.now()
        LOT_REGISTRY[lot_id] = {
            "legs": legs[1:],            # 첫 leg를 뺀 나머지
            "priority": entry["priority"],
            "created_t": now,
            "pattern": entry["pattern"],
            "n_done": 0,
        }
        job = _spawn_next_leg(lot_id, legs[0], entry["priority"], now)
        self.injected += 1
        return job

    def process(self):
        # Manager는 이제 "작업 생성"만 한다. 배정은 Dispatcher가
        # DISPATCH_INTERVAL(10초)마다 스코어링으로 따로 수행한다.
        #
        # [실측 Assign Delay 재현] 배정될 때까지 작업은 PENDING_JOBS에
        # 남아 대기한다. 이 대기시간(Job.delay_time)이 실측 로그의
        # Assign Delay(평균 14.5s, 최대 164.5s)에 대응하며 W_delay의
        # 입력이 된다.
        while True:
            self.create_job()
            yield self.hold(SPAWN_INTERVAL)


# ----------------------------------------------------------------------
# OHT 컴포넌트 (영구 풀 - 소멸하지 않고 idle_pool과 왕복)
# ----------------------------------------------------------------------
# ======================================================================
# Job 엔티티 / OHT 상태머신
# ======================================================================
# [왜 Job을 따로 떼어내는가]
# 예전에는 Manager가 (from, stations, to) 튜플을 만들어 OHT.assign()에 넘기는
# 순간 그 작업이 해당 OHT 인스턴스 안에 갇혔다. 회사 방법론 1번은 10초마다
# "후보 OHT x 배정가능 작업"을 다시 매칭하고, 아직 픽업 전인 작업은 다른
# 차량으로 옮길 수 있어야 한다(Reassign). 그러려면 작업이 차량과 독립적으로
# 존재하면서 "지금 누가 맡고 있는지"만 참조로 물고 있어야 한다.
#
# [Load / Unload 용어] 회사 문서(및 SEMI 관례)는 포트 기준이다.
#     Unload = 포트에서 OHT로 자재를 올리는 동작   -> 픽업(FromPort)
#     Load   = OHT에서 포트로 자재를 내리는 동작   -> 하차(ToPort)
# 이 파일도 이제부터 이 기준을 따른다(예전 주석의 "적재/하차" 표현과 방향이
# 반대이므로 주의). 소요시간은 양쪽 모두 OHT_LOAD_UNLOAD_TIME으로 같다.

# --- Job 상태 ---
JOB_PENDING = "pending"      # 생성됐지만 아직 아무 OHT에도 배정되지 않음
JOB_ASSIGNED = "assigned"    # OHT가 배정돼 픽업하러 가는 중 (아직 Unload 전)
                             #  -> 이 상태의 작업만 다른 차량으로 Reassign 가능
JOB_CARRYING = "carrying"    # 픽업(Unload) 완료, 자재를 싣고 목적지로 이동 중
                             #  -> 자재가 차 위에 있으므로 Reassign 불가
JOB_DONE = "done"            # 하차(Load)까지 끝남

# --- OHT 상태 ---
OHT_IDLE = "idle"            # 유휴. 배회(wander) 중이며 후보 선별 1-(1)
OHT_TO_PICKUP = "to_pickup"  # 작업 배정됨, 빈 차로 FromPort 이동 중. 1-(2)
OHT_UNLOADING = "unloading"  # FromPort에서 자재를 싣는 중(포트 기준 Unload)
OHT_TO_DEST = "to_dest"      # 자재를 싣고 ToPort로 이동 중. 하차지점까지
                             #  남은 시간이 짧으면 후보 선별 1-(3)
OHT_LOADING = "loading"      # ToPort에서 자재를 내리는 중(포트 기준 Load)

# 후보 선별 1-(3): 하차지점까지 남은 예상 주행시간이 이 값 이내면
# "곧 작업을 마칠 차량"으로 보고 다음 작업의 후보에 넣는다.
SOON_FREE_SECONDS = 10.0


class Job:
    """운반 작업 1건(=로트 안의 leg 하나). 차량과 독립적으로 존재하며
    담당 OHT를 참조로만 문다."""

    __slots__ = ("job_id", "from_node", "to_node",
                 "from_port", "to_port", "from_kind", "to_kind",
                 "priority", "port_weight", "created_t", "assigned_t",
                 "state", "oht", "n_assign", "lot_id", "leg_index")

    def __init__(self, job_id, from_node, to_node,
                 from_port=None, to_port=None, from_kind=None, to_kind=None,
                 priority=0.0, port_weight=0.0, created_t=0.0,
                 lot_id=None, leg_index=0):
        self.job_id = job_id
        self.from_node = from_node      # 픽업(Unload) 지점의 접근노드
        self.to_node = to_node          # 하차(Load) 지점의 접근노드
        self.from_port = from_port
        self.to_port = to_port
        self.from_kind = from_kind
        self.to_kind = to_kind
        # [3단계용 자리] W_priority(상위 시스템이 주는 긴급도, 최대 50)와
        # W_port(관리자가 특정 포트에 미리 걸어둔 가산점, 기본 0).
        # 지금 JOB 데이터에는 둘 다 없어서 기본값 0으로 둔다.
        self.priority = priority
        self.port_weight = port_weight
        self.created_t = created_t      # 생성 시각 -> DelayTime 계산 기준
        self.assigned_t = None          # 마지막으로 배정된 시각
        self.state = JOB_PENDING
        self.oht = None                 # 현재 담당 OHT (없으면 None)
        self.n_assign = 0               # 배정된 횟수(Reassign 빈도 추적용)
        # [5번 요청사항 - 로트 다단계 흐름] 이 leg가 속한 로트 식별자와,
        # 그 로트 안에서 몇 번째 leg인지. 완료 시 advance_lot()이 이 값으로
        # LOT_REGISTRY에서 다음 leg를 찾는다. lot_id=None이면 로트에
        # 속하지 않는 단발성 job(예전 방식/합성 모델 호환용).
        self.lot_id = lot_id
        self.leg_index = leg_index

    def delay_time(self, now):
        """생성된 뒤 아직 픽업되지 않은 채 흐른 시간(초). W_delay의 입력."""
        return max(0.0, now - self.created_t)

    @property
    def reassignable(self):
        """아직 자재를 싣지 않았으므로 다른 차량이 가져가도 되는 작업."""
        return self.state == JOB_ASSIGNED


idle_pool = []      # 유휴 OHT 목록
OHT_LIST = []       # 생성된 모든 OHT 인스턴스 (render_frame이 매 프레임 순회)

JOB_POOL = {}       # job_id -> Job (아직 끝나지 않은 작업 전체)
PENDING_JOBS = []   # 미할당(JOB_PENDING) 작업 목록 - 생성 순서 유지
_JOB_SEQ = [0]      # job_id 발급 카운터 (리스트로 감싸 reset에서 갈아끼움)

# [5번 요청사항 - 로트 다단계 흐름] ------------------------------------
# lot_id -> {"legs": [남은 leg dict, ...], "priority": float,
#            "created_t": float, "n_done": int}
# 로트가 생성될 때 첫 leg는 바로 Job이 되어 빠지고, 나머지는 여기 "legs"에
# 남아있다가 현재 leg가 끝날 때마다 하나씩 꺼내 다음 Job으로 만든다.
LOT_REGISTRY = {}
_LOT_SEQ = [0]
LOT_STATS = []      # 로트 전체(STK->...->STK) 완료 시 1건씩: {"lot_id","cycle","n_legs","pattern","t_end"}


def new_lot_id():
    _LOT_SEQ[0] += 1
    return _LOT_SEQ[0]


def new_job_id():
    _JOB_SEQ[0] += 1
    return _JOB_SEQ[0]


def _spawn_next_leg(lot_id, leg, priority, now):
    """leg dict로 새 Job을 만들어 풀에 넣는다(로트의 다음 구간). leg가
    EQ로 들어가는 경우 도착 즉시 화면에서 '예약'으로 보이도록 미리
    표시해둔다(실제 가공 시작은 그 leg가 완료된 뒤)."""
    job = Job(new_job_id(), created_t=now, priority=priority,
              lot_id=lot_id, **leg)
    JOB_POOL[job.job_id] = job
    PENDING_JOBS.append(job)
    if leg["to_kind"] == "EQ":
        set_eq_status(leg["to_node"], "reserved")
    return job


def advance_lot(lot_id, now):
    """방금 leg 하나가 끝난 로트를 한 단계 진행시킨다.
    남은 leg가 있으면 다음 leg를 Job으로 만들어 PENDING_JOBS에 넣고
    (leg마다 다른 OHT가 맡을 수 있다 - 사용자 확인), 없으면 로트가
    완전히 끝난 것이므로 LOT_STATS에 1건 기록하고 레지스트리에서 지운다."""
    ent = LOT_REGISTRY.get(lot_id)
    if ent is None:
        return   # 방어적: 이미 정리된 로트(재시작 등)
    ent["n_done"] += 1
    if ent["legs"]:
        leg = ent["legs"].pop(0)
        _spawn_next_leg(lot_id, leg, ent["priority"], now)
    else:
        LOT_STATS.append({
            "lot_id": lot_id,
            "cycle": now - ent["created_t"],
            "n_legs": ent["n_done"],
            "pattern": ent["pattern"],
            "t_end": now,
        })
        LOT_REGISTRY.pop(lot_id, None)


class LotProcessAndAdvance(sim.Component):
    """[5번 요청사항] leg가 EQ에 도착(Load 완료)하면 스폰된다. 실제
    가공(PROCESS_TIME)은 OHT와 무관하게 여기서 흘러간다 - OHT는 이 leg가
    끝나는 순간 바로 풀려서 다른 일을 할 수 있다(로트를 끝까지 같은 OHT가
    담당하지 않는다는 사용자 확인과 일치). EQ 리소스(동시 1개)도 기존
    합성모델과 같은 방식으로 경합한다."""

    def setup(self, lot_id, eq_node):
        self.lot_id = lot_id
        self.eq_node = eq_node

    def process(self):
        set_eq_status(self.eq_node, "processing")
        res = get_eq_resource(self.eq_node)
        yield self.request(res)
        yield self.hold(PROCESS_TIME)
        set_eq_status(self.eq_node, "done")
        self.release(res)
        set_eq_status(self.eq_node, "empty")
        advance_lot(self.lot_id, self.env.now())


def assignable_ohts():
    """회사 방법론 1번의 "작업 할당이 가능한 OHT" 후보 집합.
        (1) IDLE 유휴차량
        (2) 배정은 됐지만 아직 픽업(Unload) 전 -> 지금 작업을 뺏어와도 됨
        (3) 하차(Load) 지점까지 SOON_FREE_SECONDS 이내 -> 곧 유휴가 됨
    3단계의 스코어링이 이 목록을 받아 N x M 매트릭스를 만든다."""
    out = []
    for oht in OHT_LIST:
        if oht.state == OHT_IDLE:
            out.append(oht)
        elif oht.state == OHT_TO_PICKUP:
            out.append(oht)
        elif oht.state == OHT_TO_DEST and oht.eta_to_free() <= SOON_FREE_SECONDS:
            out.append(oht)
    return out


def assignable_jobs():
    """미할당 작업 + Reassign 가능한(아직 픽업 전) 작업."""
    return [j for j in JOB_POOL.values()
            if j.state == JOB_PENDING or j.state == JOB_ASSIGNED]


# ======================================================================
# 배정 스코어링 + Greedy Matching (회사 방법론 1번, 3~4)
# ======================================================================
#   Total Score = W_movetime + W_priority + W_delay + W_port
#
# W_movetime : 현 OHT 위치에서 FromPort까지의 실제 레일 최단주행시간이
#              짧을수록 100에 수렴. 역수형 100/(1+t/t0)을 쓴다.
#              t=0이면 100, t=t0이면 50, 멀어질수록 0으로 완만히 감소한다.
#              이미 그 차량에 할당돼 있던 작업이면 10점을 더해 잦은
#              Reassign을 억제한다.
# W_priority : 상위 시스템이 준 긴급도(최대 50). 지금 JOB 데이터에 없어
#              전부 0이며, Job.priority에 값이 들어오면 그대로 반영된다.
# W_delay    : 작업이 생성된 뒤 아직 픽업되지 못한 채 흐른 시간(DelayTime).
#              120초까지는 선형으로 50까지 오르고, 그 뒤로는 완만히 계속
#              증가한다(굶주림 방지). 회사 문서에 상한이 없어 그대로 뒀다.
# W_port     : 관리자가 특정 설비/포트에 미리 걸어둔 가산점(기본 0).
DISPATCH_INTERVAL = 10.0    # 배정 계산 주기(초) - 방법론 "10초에 한번씩"

MOVETIME_T0 = 50.0          # W_movetime의 반감 기준(초).
                            # 후보 OHT x 작업 이동시간 실측 분포의 중앙값
                            # (52.7s, p10 30s / p90 80s)에서 잡은 값이라
                            # "이동시간이 중앙값이면 50점"이 된다.
MOVETIME_MAX = 100.0        # t=0일 때의 상한
REASSIGN_STICKY_BONUS = 10.0  # 이미 그 차량에 할당돼 있으면 가산
DELAY_KNEE = 120.0          # W_delay가 꺾이는 지점(초)
DELAY_BASE = 50.0           # 그 지점에서의 점수
DELAY_SLOPE = 0.1           # 꺾인 뒤 DELAY_KNEE마다 붙는 비율

# [N x M 폭발 방지] 포화 상태에서는 미할당 작업이 수천 건까지 쌓인다.
# 매 라운드 전부를 후보로 넣으면 행렬이 무의미하게 커지므로, 주행시간과
# 무관하게 미리 계산할 수 있는 점수(W_priority + W_delay + W_port)가 높은
# 순으로 이만큼만 추린 뒤 이동시간을 계산한다. 오래 기다린 작업일수록
# W_delay가 커지므로 굶주리는 작업이 잘려나가지는 않는다.
MAX_JOBS_PER_ROUND = 400
DIJKSTRA_SRC_CHUNK = 400    # 출발지를 이만큼씩 끊어 계산(피크 메모리 억제)


def w_movetime(t, already_assigned=False):
    """이동시간 t(초) -> 0~100점. 가까울수록 100에 수렴."""
    if not math.isfinite(t):
        return 0.0
    s = MOVETIME_MAX / (1.0 + max(t, 0.0) / MOVETIME_T0)
    return s + (REASSIGN_STICKY_BONUS if already_assigned else 0.0)


def w_delay(delay_time):
    """미할당 대기시간 -> 점수. 회사 문서의 구간별 식을 그대로 옮긴 것."""
    if delay_time < DELAY_KNEE:
        return DELAY_BASE * (delay_time / DELAY_KNEE)
    return DELAY_BASE * (1.0 + DELAY_SLOPE * (delay_time - DELAY_KNEE) / DELAY_KNEE)


# ----------------------------------------------------------------------
# 주행시간 행렬 (후보 OHT x 작업 FromPort)
# ----------------------------------------------------------------------
# [왜 별도 구조가 필요한가]
# W_movetime은 "실제 레일 최단주행시간"이라 후보 OHT N대 x 작업 M건 전부에
# 경로탐색이 필요하다. A*를 N*M번 부르면 150x1000 = 15만회 x 6ms = 900초로
# 한 라운드도 못 돈다. 대신 pair-state 그래프를 scipy CSR로 한 번 구워두고
# 출발지 N개에서 다중출발 다익스트라를 돌리면(C 구현) 모든 목적지까지의
# 시간이 한 번에 나온다. 실측 150대 0.35초 / 1500대 2.8초라 10초 주기 안에
# 충분히 들어간다.
#
# 상태 배치: [0, NN)          = "노드 n에서 정지상태로 출발"
#            [NN, NN+NP)      = pair-state (직전링크, 다음링크)
#            [NN+NP, 2NN+NP)  = "노드 n에 도착해 정지"
# 간선 가중치는 A*가 쓰는 link_time()과 완전히 동일하다.
def _build_time_csr():
    nidx = {n: i for i, n in enumerate(NODE_LIST)}
    pair_idx = {s: i + len(NODE_LIST) for i, s in enumerate(JUNCTION_SPEED)}
    stop_off = len(NODE_LIST) + len(pair_idx)
    total = stop_off + len(NODE_LIST)

    rows, cols, wts = [], [], []
    for n in NODE_LIST:
        si = nidx[n]
        for j in OUT_LINKS[n]:
            if j in BLOCKED_LINKS:
                continue
            for k in SUCC_LINKS[j]:
                st = (j, k)
                if k not in BLOCKED_LINKS and st in JUNCTION_SPEED:
                    rows.append(si)
                    cols.append(pair_idx[st])
                    wts.append(link_time(0.0, JUNCTION_SPEED[st], j))
            # 링크 하나만 달리고 바로 멈추는 경우
            rows.append(si)
            cols.append(stop_off + nidx[LINK_B[j]])
            wts.append(link_time(0.0, 0.0, j))

    for (i, j), si in pair_idx.items():
        if i in BLOCKED_LINKS or j in BLOCKED_LINKS:
            continue
        v = JUNCTION_SPEED[(i, j)]
        for k in SUCC_LINKS[j]:
            st = (j, k)
            if k not in BLOCKED_LINKS and st in JUNCTION_SPEED:
                rows.append(si)
                cols.append(pair_idx[st])
                wts.append(link_time(v, JUNCTION_SPEED[st], j))
        rows.append(si)
        cols.append(stop_off + nidx[LINK_B[j]])
        wts.append(link_time(v, 0.0, j))

    csr = csr_matrix((np.array(wts), (rows, cols)), shape=(total, total))
    return csr, nidx, stop_off


_t0 = time.time()
TIME_CSR, TIME_NODE_IDX, TIME_STOP_OFF = _build_time_csr()
print(f"[배정] 주행시간 행렬용 CSR {TIME_CSR.shape[0]}상태 / "
      f"{TIME_CSR.nnz}간선 ({time.time() - _t0:.2f}s)")


def travel_time_matrix(origin_nodes, dest_nodes):
    """origin_nodes에서 dest_nodes까지의 주행시간 행렬 (N x M, 초).
    도달 불가는 inf. 출발지를 청크로 끊어 피크 메모리를 억제한다."""
    if not origin_nodes or not dest_nodes:
        return np.zeros((len(origin_nodes), len(dest_nodes)))
    cols = [TIME_STOP_OFF + TIME_NODE_IDX[n] for n in dest_nodes]
    out = np.empty((len(origin_nodes), len(dest_nodes)))
    for s in range(0, len(origin_nodes), DIJKSTRA_SRC_CHUNK):
        chunk = origin_nodes[s:s + DIJKSTRA_SRC_CHUNK]
        src = [TIME_NODE_IDX[n] for n in chunk]
        dist = dijkstra(TIME_CSR, directed=True, indices=src)
        out[s:s + len(chunk)] = dist[:, cols]
    return out


def run_dispatch_round(now):
    """방법론 1번 한 라운드: 후보 선별 -> N x M 스코어 -> Greedy Matching.
    반환: (배정 성사 건수, 그중 Reassign이었던 건수)"""
    ohts = assignable_ohts()
    jobs = assignable_jobs()
    if not ohts or not jobs:
        return 0, 0

    # --- 주행시간과 무관한 점수를 먼저 계산해 작업 후보를 추린다 ---
    base = [j.priority + w_delay(j.delay_time(now)) + j.port_weight for j in jobs]
    if len(jobs) > MAX_JOBS_PER_ROUND:
        keep = sorted(range(len(jobs)), key=lambda i: base[i],
                      reverse=True)[:MAX_JOBS_PER_ROUND]
        jobs = [jobs[i] for i in keep]
        base = [base[i] for i in keep]

    # --- N x M 주행시간 -> W_movetime ---
    origins = [o.score_origin_node() for o in ohts]
    tmat = travel_time_matrix(origins, [j.from_node for j in jobs])

    with np.errstate(over="ignore", invalid="ignore"):
        score = MOVETIME_MAX / (1.0 + np.maximum(tmat, 0.0) / MOVETIME_T0)
    score[~np.isfinite(tmat)] = -np.inf          # 도달 불가 조합은 후보에서 제외
    # 이미 그 차량이 들고 있던 작업이면 10점 가산(잦은 Reassign 방지)
    held = {}
    for oi, oht in enumerate(ohts):
        if oht.job is not None:
            held[id(oht.job)] = oi
        if oht.next_job is not None:
            held[id(oht.next_job)] = oi
    for ji, job in enumerate(jobs):
        oi = held.get(id(job))
        if oi is not None:
            score[oi, ji] += REASSIGN_STICKY_BONUS
    score += np.asarray(base)[None, :]

    # --- Greedy Matching: Total Score 내림차순 ---
    flat = score.ravel()
    order = np.argsort(-flat, kind="stable")
    n_job = len(jobs)
    used_o = set()
    used_j = set()
    n_match = n_reassign = 0
    limit = min(len(ohts), n_job)
    for pos in order:
        if len(used_o) >= limit or len(used_j) >= limit:
            break
        if not math.isfinite(flat[pos]):
            break                                 # 남은 건 전부 도달 불가
        oi, ji = divmod(int(pos), n_job)
        if oi in used_o or ji in used_j:
            continue
        oht, job = ohts[oi], jobs[ji]
        used_o.add(oi)
        used_j.add(ji)
        if oht.job is job or oht.next_job is job:
            continue                              # 이미 그대로다 - 손대지 않는다
        was_assigned = job.state == JOB_ASSIGNED
        if oht.take_job(job):
            n_match += 1
            if was_assigned:
                n_reassign += 1
    return n_match, n_reassign


def release_standby_point(oht):
    """이 OHT가 잡고 있던 대기지점 예약을 반납한다."""
    idx = oht._idle_point_idx
    if idx is not None and STANDBY_ASSIGNED.get(idx) is oht:
        STANDBY_ASSIGNED.pop(idx, None)
    oht._idle_point_idx = None
    oht._idle_target = None


class IdleDistributor(sim.Component):
    """OCS의 'Idle 차량 분산 Route 명령'에 해당한다.

    IDLE_REBALANCE_INTERVAL마다 Job 없는 OHT를 모아, 아직 비어있는 대기지점
    (베이별 N등분 지점) 중 가장 가까운 곳으로 보낸다. 거리는 화면상 직선이
    아니라 실제 레일 주행시간(travel_time_matrix)으로 재기 때문에, 역주행
    불가/일방통행 구조가 그대로 반영된다.

    우선순위: 중앙통로에 서 있는 차량 -> 그다음 나머지. 중앙통로는 대기
    금지 구역이라(사용자 확인) 거기 있는 유휴 차량을 먼저 빼내야 한다."""

    def setup(self, period=None):
        self.period = period if period is not None else IDLE_REBALANCE_INTERVAL
        self.rounds = 0

    def process(self):
        while True:
            yield self.hold(self.period)
            self.rebalance()
            self.rounds += 1

    def rebalance(self):
        # 더 이상 유휴가 아닌 차량이 잡고 있던 자리는 회수
        for idx, oht in list(STANDBY_ASSIGNED.items()):
            if oht.job is not None or oht.state != OHT_IDLE:
                STANDBY_ASSIGNED.pop(idx, None)
                if oht._idle_point_idx == idx:
                    oht._idle_point_idx = None
                    oht._idle_target = None

        # 배정이 필요한 유휴 차량 (중앙통로에 서 있는 차량을 앞쪽에)
        need = [o for o in OHT_LIST
                if o.job is None and o.state == OHT_IDLE
                and o._idle_point_idx is None]
        if not need:
            return
        need.sort(key=lambda o: 0 if o.node in CORRIDOR_NODE_SET else 1)

        free_idx = [i for i in range(len(IDLE_STANDBY_POINTS))
                    if i not in STANDBY_ASSIGNED]
        if not free_idx:
            return

        origins = [o.node for o in need]
        dests = [IDLE_STANDBY_POINTS[i] for i in free_idx]
        tmat = travel_time_matrix(origins, dests)

        taken = set()
        for oi, oht in enumerate(need):
            row = tmat[oi]
            best_j = -1
            best_t = math.inf
            for j in range(len(free_idx)):
                if j in taken:
                    continue
                t = row[j]
                if t < best_t:
                    best_t, best_j = t, j
            if best_j < 0 or not math.isfinite(best_t):
                continue
            taken.add(best_j)
            idx = free_idx[best_j]
            STANDBY_ASSIGNED[idx] = oht
            oht._idle_point_idx = idx
            oht._idle_target = IDLE_STANDBY_POINTS[idx]
            IDLE_STATS["assigns"] += 1
            if oht.ispassive():
                oht.activate()


def _find_queue_head(oht, max_depth=64):
    """oht가 막혀서 기다리는 락을 따라 계속 거슬러 올라가, 실제로 "판단을
    내릴 수 있는" 지점(분기 노드에 서 있는 OHT)을 찾는다.

    단일 레인(나가는 링크가 하나뿐인 구간)에 줄줄이 밀려 있으면, 그 줄의
    맨 앞 차량만 다른 경로를 고를 수 있다 - 줄 중간 차량은 물리적으로
    갈 곳이 없어서(나가는 링크가 하나뿐) 재탐색을 해봐야 결과가 똑같다.
    그래서 그 줄을 실제로 막고 있는 분기점의 OHT를 찾아 대신 재탐색을
    요청한다. 분기점을 못 찾으면(줄이 끝나거나, 순환 대기라 계속 맴돌면)
    None - 이 경우 우회 자체가 존재하지 않는다는 뜻이다."""
    visited = {oht}
    cur = oht
    for _ in range(max_depth):
        nxt = cur._stall_next
        if nxt is None:
            return None
        holder = NODE_HOLDER.get(nxt)
        if holder is None or holder is cur:
            return None    # 지금은 안 막혀 있음(방금 풀렸을 수도)
        if holder.node in JUNCTION_NODES:
            return holder
        if holder in visited:
            return None    # 순환 대기(circular wait) - 분기점이 아니므로 우회 불가
        visited.add(holder)
        if holder._stall_next is None:
            return None    # holder는 지금 대기 중이 아님(막 지나가는 중 등)
        cur = holder
    return None


class StallWatchdog(sim.Component):
    """트리거 (2): 같은 분기점을 JUNCTION_STALL_REROUTE초 넘게 통과하지
    못하고 있는 OHT를 찾아 우회로를 탐색하게 한다.

    OHT는 노드락을 기다리는 동안 passivate()로 잠들어 있어서 스스로
    "10초 지났다"를 알 수 없다. 대기 큐 로직(release/wait_for_node)을
    건드리면 락 공정성이 틀어지므로, 밖에서 주기적으로 훑어 깨우는 방식을
    택했다. 깨우면 _travel_path의 대기 루프가 _replan을 보고 빠져나온다.

    [줄서기 릴레이] 정체된 OHT가 분기점이 아닌 단일 레인 중간에 있으면
    (예: 베이 안쪽에 줄줄이 밀려선 경우), 재탐색을 해봐야 나가는 링크가
    하나뿐이라 결과가 똑같다. 그래서 그런 경우 _find_queue_head()로 이
    줄을 실제로 막고 있는 분기점의 OHT를 찾아 "그 OHT"의 재탐색을 대신
    요청한다 - 요청 근거(대기 시작 시각)는 지금 처리 중인 OHT(줄 중간
    차량) 자신의 것을 그대로 쓴다. 그래야 "줄 중간에서 60초 넘게 기다린"
    사실이 곧바로 머리(head)의 강제우회 판단에 반영된다.

    [누적 정체 강제 우회] 매번 "방금 막힌 링크 하나"만 피하면, 근처가
    전반적으로 혼잡할 때 매번 다른 링크에 또 걸리면서 사실상 같은 동네를
    못 벗어날 수 있다. 그래서 첫 정체가 시작된 시각(_first_stall_t)부터
    누적으로 재고, FORCE_DETOUR_SECONDS를 넘으면 그동안 걸렸던 링크
    전부(_stall_avoid_links, 리셋 없이 계속 누적)를 한꺼번에 피하게 해서
    이 지역 자체를 벗어나는 - 더 멀더라도 - 경로를 강제한다."""

    def setup(self, period=1.0):
        self.period = period
        self.n_fired = 0
        self.n_forced_detour = 0   # 누적 강제 우회가 실제로 발동한 횟수
        self.n_relayed = 0         # 줄서기 릴레이로 머리(head)에 대신 요청한 횟수

    def process(self):
        while True:
            yield self.hold(self.period)
            now = self.env.now()
            handled = set()   # 이번 틱에 이미 처리한 head(id) - 중복 방지
            for oht in OHT_LIST:
                if (oht._stall_start is None or oht._replan or oht._abort_leg
                        or oht._ring_waiting):
                    continue
                if now - oht._stall_start < JUNCTION_STALL_REROUTE:
                    continue

                if oht.node in JUNCTION_NODES:
                    target, origin_stall_t, relayed = oht, oht._stall_start, False
                else:
                    head = _find_queue_head(oht)
                    if (head is None or head._replan or head._abort_leg
                            or head._ring_waiting):
                        continue      # 갈림길 자체가 없거나 이미 재탐색 중
                    target, origin_stall_t, relayed = head, oht._stall_start, True

                if id(target) in handled:
                    continue
                handled.add(id(target))

                nxt = target._stall_next
                li = EDGE_LINK_IDX.get((target.node, nxt)) if nxt else None
                if li is None:
                    continue

                # [줄서기 릴레이] target(head)의 누적 시작시각은 이 줄에서
                # 가장 오래 기다린 시점으로 갱신한다 - head 자신이 짧게
                # 짧게 통과해온 탓에 개인 기록은 얼마 안 됐어도, 뒤에 줄
                # 선 차량이 훨씬 오래 기다렸다면 그게 이 병목의 진짜 나이다.
                if target._first_stall_t is None or origin_stall_t < target._first_stall_t:
                    target._first_stall_t = origin_stall_t
                target._stall_avoid_links.add(li)

                if now - target._first_stall_t >= FORCE_DETOUR_SECONDS:
                    # 누적 정체가 기준을 넘었다 - 그동안 걸렸던 링크 전부를
                    # 피해서 완전히 다른 경로를 강제한다(멀어져도 상관없음).
                    target._avoid_links = set(target._stall_avoid_links)
                    self.n_forced_detour += 1
                else:
                    # 아직 기준 전이면 지금까지처럼 방금 걸린 링크만 피한다.
                    target._avoid_links = {li}

                target._replan = True
                REROUTE_COUNTS["stall"] += 1
                self.n_fired += 1
                if relayed:
                    self.n_relayed += 1
                if target.ispassive():
                    target.activate()


class CongestionMonitor(sim.Component):
    """방법론 2-3번: 실시간 교통 혼잡도를 주기적으로 갱신하고, 바뀐 가중치를
    W_movetime용 주행시간 행렬(CSR)에도 반영한다."""

    def setup(self, period=None):
        self.period = period if period is not None else CONGESTION_INTERVAL
        self.updates = 0

    def process(self):
        while True:
            yield self.hold(self.period)
            update_congestion()
            _on_blocked_changed()   # 새 가중치로 CSR 재구축
            self.updates += 1


class Dispatcher(sim.Component):
    """DISPATCH_INTERVAL마다 배정 라운드를 돈다."""

    def setup(self):
        self.rounds = 0
        self.n_match = 0
        self.n_reassign = 0

    def process(self):
        while True:
            yield self.hold(DISPATCH_INTERVAL)
            m, r = run_dispatch_round(self.env.now())
            self.rounds += 1
            self.n_match += m
            self.n_reassign += r


class OHT(sim.Component):
    def setup(self, park_node):
        """park_node: 최초 대기 위치(트랙 노드). 시뮬레이션 내내 재사용된다."""
        self.node = park_node
        self.job = None          # 현재 맡고 있는 Job 객체 (없으면 None)
        self.next_job = None     # 후보조건 (3)으로 미리 예약된 다음 작업.
                                 #  하차를 끝내는 즉시 이어서 착수한다.
        self.state = OHT_IDLE    # 위 OHT_* 상수 중 하나
        self.color = IDLE_COLOR
        self.heading = 0.0
        # [구간(leg) 진행상황] 후보 선별 1-(3) "하차지점 10초 이내" 판정과
        # 4단계 재탐색에서 쓴다. leg를 시작할 때 누적 소요시간표를 한 번
        # 만들어두고, 노드를 하나 지날 때마다 _leg_idx만 올린다.
        self._leg_path = None    # 지금 따라가는 노드 경로
        self._leg_cum = None     # 경로 i번째 노드까지의 누적 예상 주행시간
        self._leg_idx = 0        # 그 경로에서 지금 몇 번째 노드에 있는지
        self._abort_leg = False  # True면 _travel_path가 현재 경로를 포기한다
                                 #  (Reassign용 - leg 자체를 버린다)
        # [재탐색] _replan은 "이 경로는 접고 현 위치에서 다시 잡아라"는 뜻으로,
        # _abort_leg(작업 자체를 놓음)와 구분된다. _goto()가 이 플래그를 보고
        # 경로를 다시 계산해 이어간다.
        self._replan = False
        self._avoid_links = None    # 트리거(2)에서 우회를 위해 임시 제외할 링크
        self._ring_detour_node = None   # [고리 진입 제어] 우회를 이미 시도한 노드
        self._ring_exempt = False       # [고리 진입 제어] 길 비켜주기 한 칸 이동 중
        self._ring_waiting = False      # [고리 진입 제어] 지금 고리 진입 대기 중
        # [누적 정체 강제 우회] 여러 번의 재탐색에 걸쳐 계속 못 벗어나고
        # 있는지 추적한다. _first_stall_t가 None이 아니면 "이 스트릭이
        # 언제 시작됐는지", _stall_avoid_links는 "그동안 걸렸던 링크 전부"
        # (리셋 없이 계속 누적). 진짜로 막힘 없이 통과하면(_travel_path에서
        # 확인) 둘 다 초기화된다.
        self._first_stall_t = None
        self._stall_avoid_links = set()
        self._last_reroute_t = -1e9  # 트리거(1) 최소 5초 간격 판정용
        self._stall_start = None     # 지금 정체가 시작된 시각(감시용, 없으면 None)
        self._stall_next = None      # 진입하지 못하고 있는 다음 노드
        self._arrival_link = None    # 지금 노드에 진입할 때 타고 온 링크
        # [Idle 로직] 배정받은 대기지점(Zone N등분 지점)과 비켜주기 요청.
        self._idle_target = None     # 이동해야 할 대기지점 노드 (없으면 None)
        self._idle_point_idx = None  # 그 대기지점의 인덱스(반납용)
        self._yield_request = False  # True면 즉시 한 칸 비켜준다
        # heading=0 -> 회전 없음. 단, render_frame()이 화면(y-down) 좌표에
        # 직접 더하는 값이므로 _update_heading()과 동일하게 y부호를
        # 미리 뒤집어둔다(최초 배회 이동 전 정지 프레임에서도 꼭짓점
        # 방향이 어긋나지 않도록 하기 위함 - 아래 [진행방향 버그 수정] 참고).
        self._tri_local = tuple((lx, -ly) for lx, ly in TRI_LOCAL_PTS)

        # [번호 매기기] OHT별 운행률/병목률/위치 로그 추출 등에 쓸 수 있게
        # 생성 순서대로 0부터 순번을 매긴다. OHT_LIST에 등록되기 직전
        # 길이를 그대로 쓰면 되므로 별도 전역 카운터가 필요 없고, run이
        # 바뀔 때(reset_globals 등에서 OHT_LIST.clear())마다 자연히 0부터
        # 다시 시작한다.
        self.oht_id = len(OHT_LIST)

        px, py = screen_xy(self.node)
        self._seg_start_t = 0.0
        self._seg_start_xy = (px, py)
        self._seg_end_xy = (px, py)
        self._seg_dist = 0.0
        self._seg_profile = {"v0": 0.0, "v_peak": 0.0, "v_end": 0.0,
                              "t_acc": 0.0, "t_cruise": 0.0, "t_dec": 0.0,
                              "dur": 0.001}
        self._cur_speed = 0.0  # 현재 속도(도면단위/초), 구간 간 이어받아 가감속 계산에 사용

        # [용량 산정용 통계] job_start_t~완료까지가 이 OHT의 "가동" 구간.
        # total_busy는 그 구간 길이의 누적합으로, 가동률(=total_busy/전체
        # 경과시간) 계산에 쓴다. _job_* 4종은 "현재 진행중인 작업 1건"에
        # 대한 대기/주행/가공/적재-하차 시간을 따로 누적했다가 작업 완료 시
        # JOB_STATS에 한 건으로 기록한다(건별 분해를 위함).
        self.job_start_t = None
        self.total_busy = 0.0
        self._job_wait = 0.0
        self._job_travel = 0.0
        self._job_process = 0.0
        self._job_loadunload = 0.0  # H/W Spec의 Load/Unload 시간(OHT_LOAD_UNLOAD_TIME) 누적

        # [병목률용] 이 OHT가 지금까지 누적으로 멈춰있었던(락 대기 +
        # follow-distance 대기) 시간의 합. STALL_EVENTS는 STALL_THRESHOLD_
        # SECONDS 이상인 "굵직한" 정체만 이벤트로 남기지만, 이건 짧은
        # 정체까지 전부 더해서 "이 OHT가 전체 시간 중 몇 %를 멈춰서
        # 보냈는지(병목률)"를 바로 계산할 수 있게 해준다.
        self.total_stall_time = 0.0
        # [원인 분해] 위 total_stall_time을 "왜 멈췄는지"로 쪼갠 값.
        #   lock  : 다음 노드가 이미 점유돼 있어서(교차점/합류부 경합) 기다린 시간
        #   follow: 앞차와의 최소 차간거리(MIN_FOLLOW_DISTANCE)에 걸려 기다린 시간
        #           - 사용자가 말한 "앞 OHT에 막혀 멈춰있었던 병목시간"이 이것.
        self.total_lock_stall_time = 0.0
        self.total_follow_stall_time = 0.0
        # [주행 거리] 가동률 외에 "얼마나 많이 움직였나"를 같이 보기 위한 누적치.
        self.total_distance = 0.0

        # [2500대 대응] 예전에는 OHT마다 sim.AnimatePolygon(tkinter 캔버스
        # 오브젝트)을 하나씩 만들었다. 2500개가 되면 tkinter가 매 프레임
        # 그 개수만큼 아이템을 갱신해야 해서 이게 렉의 핵심 원인이었다.
        # 이제는 개별 오브젝트를 만들지 않고 OHT_LIST에만 등록해두면,
        # main()에서 만드는 단 하나의 AnimateImage(render_frame)가 매 프레임
        # 전체 OHT 위치를 한 번에 PIL로 그려서 이미지 한 장으로 합성한다.
        OHT_LIST.append(self)

        try_acquire(self.node, self)
        idle_pool.append(self)

    def _interp_xy(self, t):
        """구간 이동을 매 프레임 여러 hold로 쪼개는 대신, 가감속 프로파일과
        시작/도착 좌표만 저장해두고 렌더링 시점(t)에 위치를 계산한다.
        OHT가 여러 노드를 지나도 구간당 이벤트 1개(hold(dur))만 발생한다."""
        if self._seg_dist <= 0:
            return self._seg_end_xy
        te = t - self._seg_start_t
        te = max(0.0, min(te, self._seg_profile["dur"]))
        d = self._profile_distance(self._seg_profile, te)
        frac = min(d / self._seg_dist, 1.0)
        x0, y0 = self._seg_start_xy
        x1, y1 = self._seg_end_xy
        return (x0 + (x1 - x0) * frac, y0 + (y1 - y0) * frac)

    @staticmethod
    def _profile_distance(prof, te):
        """가감속 프로파일 상에서 경과시간 te까지 이동한 거리.
        가속(t_acc) -> 순항(t_cruise, v_peak) -> 감속(t_dec) 3구간."""
        v0, v_pk, v_end = prof["v0"], prof["v_peak"], prof["v_end"]
        t_acc, t_cru, t_dec = prof["t_acc"], prof["t_cruise"], prof["t_dec"]
        if te <= t_acc:
            return v0 * te + 0.5 * OHT_ACCEL * te * te
        d = v0 * t_acc + 0.5 * OHT_ACCEL * t_acc * t_acc
        if te <= t_acc + t_cru:
            return d + v_pk * (te - t_acc)
        d += v_pk * t_cru
        td = min(te - t_acc - t_cru, t_dec)
        return d + v_pk * td - 0.5 * OHT_DECEL * td * td

    @staticmethod
    def _profile_time(prof, d):
        """_profile_distance의 역함수. 거리 d까지 가는 데 걸리는 시간.
        가속/순항/감속 중 d가 어느 구간에 떨어지는지 찾아 그 구간 식을
        뒤집는다(등가속 구간은 이차방정식, 순항 구간은 일차). d가 구간
        전체 거리보다 크면 전체 소요시간을 돌려준다."""
        v0, v_pk = prof["v0"], prof["v_peak"]
        t_acc, t_cru, t_dec = prof["t_acc"], prof["t_cruise"], prof["t_dec"]
        if d <= 0.0:
            return 0.0
        # 1) 가속 구간:  d = v0*t + 0.5*a*t^2
        d_acc = v0 * t_acc + 0.5 * OHT_ACCEL * t_acc * t_acc
        if d <= d_acc:
            if OHT_ACCEL <= 0.0:
                return d / v0 if v0 > 1e-9 else 0.0
            return (math.sqrt(max(v0 * v0 + 2.0 * OHT_ACCEL * d, 0.0))
                    - v0) / OHT_ACCEL
        # 2) 순항 구간:  등속
        d_cru = v_pk * t_cru
        if d <= d_acc + d_cru:
            return t_acc + ((d - d_acc) / v_pk if v_pk > 1e-9 else 0.0)
        # 3) 감속 구간:  d_r = v_pk*t - 0.5*b*t^2
        d_r = d - d_acc - d_cru
        disc = v_pk * v_pk - 2.0 * OHT_DECEL * d_r
        if disc <= 0.0 or OHT_DECEL <= 0.0:
            return t_acc + t_cru + t_dec
        return t_acc + t_cru + (v_pk - math.sqrt(disc)) / OHT_DECEL

    @staticmethod
    def _segment_profile(v0, v_end_target, cap, dist):
        """길이 dist인 링크를 v0로 진입해 v_end_target으로 빠져나가는
        사다리꼴 프로파일. 속도상한 cap을 절대 넘지 않는다.

        [변경] 예전에는 "목표속도를 향해 한 번 가감속하고 남은 거리는 등속"
        이라는 2구간 모델이었고, 감속 거리가 링크 길이보다 길면 목표속도에
        못 미친 채로 그냥 다음 링크에 진입했다. 그래서 커브 진입 877건 중
        685건에서 커브 속도상한(0.7m/s)을 넘긴 채 커브에 들어가고 있었다
        (경로당 중앙값 3회). 이제 진입/탈출 속도는 라우팅과 동일한
        JUNCTION_SPEED 표에서 오고, 그 표가 사전감속을 이미 전파해뒀기
        때문에 감속은 항상 링크 안에서 끝난다.
        반환: {v0, v_peak, v_end, t_acc, t_cruise, t_dec, dur}"""
        v0 = min(max(v0, 0.0), cap)
        v_end = min(max(v_end_target, 0.0), cap)
        if dist < 1e-6:
            return {"v0": v_end, "v_peak": v_end, "v_end": v_end,
                    "t_acc": 0.0, "t_cruise": 0.0, "t_dec": 0.0, "dur": 0.001}

        # 목표 탈출속도를 이 링크 안에서 물리적으로 가능한 범위로 자른다.
        # link_travel_time()과 완전히 같은 클램프라, 라우팅이 예측한 시간과
        # 여기서 실제로 흐르는 시간이 같은 식에서 나온다.
        # (노드락 대기로 멈췄다 다시 출발하면 v0가 JUNCTION_SPEED보다
        #  낮아지는데, 그때도 이 클램프가 알아서 맞춰준다.)
        v_floor = math.sqrt(max(v0 * v0 - 2.0 * OHT_DECEL * dist, 0.0))
        v_ceil = min(math.sqrt(v0 * v0 + 2.0 * OHT_ACCEL * dist), cap)
        v_end = min(max(v_end, v_floor), v_ceil)

        d_acc = max(cap * cap - v0 * v0, 0.0) / (2.0 * OHT_ACCEL)
        d_dec = max(cap * cap - v_end * v_end, 0.0) / (2.0 * OHT_DECEL)
        if d_acc + d_dec <= dist:
            v_pk = cap
            t_acc = (v_pk - v0) / OHT_ACCEL
            t_dec = (v_pk - v_end) / OHT_DECEL
            t_cru = (dist - d_acc - d_dec) / v_pk if v_pk > 1e-6 else 0.0
        else:
            v_pk = math.sqrt(max(
                (2.0 * OHT_ACCEL * OHT_DECEL * dist
                 + OHT_DECEL * v0 * v0 + OHT_ACCEL * v_end * v_end)
                / (OHT_ACCEL + OHT_DECEL), 0.0))
            v_pk = max(v_pk, v0, v_end)
            t_acc = (v_pk - v0) / OHT_ACCEL
            t_dec = (v_pk - v_end) / OHT_DECEL
            t_cru = 0.0

        dur = max(t_acc + t_cru + t_dec, 0.001)
        return {"v0": v0, "v_peak": v_pk, "v_end": v_end,
                "t_acc": t_acc, "t_cruise": t_cru, "t_dec": t_dec, "dur": dur}

    def _triangle_spec(self):
        # (참고용, 더 이상 render_frame에서 쓰지 않음 - TRI_LOCAL_PTS/캐싱된
        # self._tri_local로 대체됨)
        return TRI_LOCAL_PTS

    def _update_heading(self, x_from, y_from, x_to, y_to):
        dx, dy = x_to - x_from, y_to - y_from
        if dx == 0 and dy == 0:
            return
        self.heading = math.degrees(math.atan2(dy, dx)) - 90
        # [추가 최적화] heading이 바뀌는 이 순간에만 회전된 삼각형 좌표를
        # 계산해서 캐싱한다. render_frame은 매 프레임 2500대 전부에 대해
        # cos/sin을 다시 계산하고 있었는데, 실제로 heading은 새 구간을
        # 시작할 때만 바뀌므로(프레임마다 바뀌는 게 아니므로) 여기서 한 번만
        # 계산해두면 render_frame 쪽은 단순 덧셈만 하면 된다.
        rad = math.radians(self.heading)
        cos_a, sin_a = math.cos(rad), math.sin(rad)
        # [진행방향 버그 수정] heading은 맵 좌표계(y-up) 기준 dx,dy로
        # 계산되는데, render_frame()이 이 오프셋을 더하는 대상은 화면
        # 픽셀 좌표(y-down, cam.pt(cx, IMG_H - cy)로 이미 한 번 y반전된
        # 값)다. 회전만 하고 y를 다시 뒤집지 않은 채 그대로 더하면, y축이
        # 반전된 좌표계 위에 y-up 기준으로 계산한 벡터를 얹는 셈이 되어
        # 상하 방향 성분이 거울처럼 뒤집힌다(좌우 이동은 x만 쓰여서 멀쩡해
        # 보이지만, 상하 성분이 섞인 이동에서는 꼭짓점이 진행 방향의 반대
        # 쪽 - 즉 밑변이 앞을 향하는 것처럼 - 보이는 원인이었다). replay
        # 쪽의 render_replay_frame()은 매 프레임 직접 "py - (...)"로 y를
        # 뒤집어서 그리는데, 이 캐시도 같은 y반전을 미리 적용해두면 두
        # 렌더 경로가 정확히 같은 결과를 낸다.
        self._tri_local = tuple(
            (lx * cos_a - ly * sin_a, -(lx * sin_a + ly * cos_a))
            for lx, ly in TRI_LOCAL_PTS
        )

    # ---- Job 인수/인계 -------------------------------------------------
    def _yield_step(self):
        """길 비켜주기: 작업 중인 차량이 내가 점유한 노드를 기다리고 있다.
        전방(순방향) 인접 노드 중 비어있는 곳으로 한 칸 물러난다.

        역주행은 불가능하므로 반드시 나가는 링크(ADJ) 쪽으로만 움직인다.
        [버그 수정] 갈 곳이 전부 막혀 있으면 예전엔 그냥 포기하고
        _yield_request를 꺼버렸는데, request_yield()는 노드락 대기 쪽에서
        한 번만 오고 재시도가 없어서(release() 대기 중 passivate) 그 한 번을
        놓치면 영원히 못 비켜주는 교착상태가 생겼다. 이제는 실패하면
        _yield_request를 켜진 채로 두고 짧게 쉬었다가 스스로 다시
        시도한다(성공할 때까지, 또는 다른 이유로 이동할 때까지).
        배정받은 대기지점에서 밀려났으면 그 자리는 반납하고, 다음 분산
        라운드에서 새로 배정받는다."""
        neighbors = ADJ.get(self.node) or []
        free = [n for n in neighbors if NODE_HOLDER.get(n) is None]
        if not free:
            yield self.hold(YIELD_RETRY_INTERVAL)   # _yield_request는 켜둔 채 재시도
            return
        # 대기지점이 있으면 그쪽에 가까워지는 방향을 우선한다(비켜주면서도
        # 원래 가려던 자리에서 멀어지지 않게).
        tgt = self._idle_target
        if tgt is not None and tgt in NODE_XY:
            tx, ty = NODE_XY[tgt]
            free.sort(key=lambda n: math.hypot(NODE_XY[n][0] - tx,
                                               NODE_XY[n][1] - ty))
        # [고리 진입 제어] 꽉 찬 고리로 들어가지 않는 칸을 우선한다(안정 정렬).
        free.sort(key=lambda n: ring_entry_blocked(self, n))
        nxt = free[0]
        if ring_entry_blocked(self, nxt):
            RING_STATS["yield_exempt"] += 1
        IDLE_STATS["yield_moves"] += 1
        self._yield_request = False   # 실제로 움직일 때만 해제
        if self.node == self._idle_target:
            # 배정된 자리에서 밀려남 -> 반납하고 다음 라운드에 재배정
            release_standby_point(self)
        self._ring_exempt = True      # 비켜주기 이동은 고리 판정 예외
        try:
            yield from self._travel_path([self.node, nxt])
        finally:
            self._ring_exempt = False

    def _begin_job(self, job):
        """job을 이 차량의 '현재 작업'으로 삼고 픽업 주행을 시작시킨다."""
        release_standby_point(self)   # 잡고 있던 Idle 대기지점 반납
        self._yield_request = False
        self.job = job
        job.oht = self
        job.state = JOB_ASSIGNED
        job.assigned_t = self.env.now()
        if job in PENDING_JOBS:
            PENDING_JOBS.remove(job)
        if self in idle_pool:
            idle_pool.remove(self)
        self.state = OHT_TO_PICKUP
        self.job_start_t = self.env.now()
        self._job_wait = 0.0
        self._job_travel = 0.0
        self._job_process = 0.0
        self._job_loadunload = 0.0
        self._first_stall_t = None      # [누적 정체] 새 작업이니 이전 누적치는 무의미
        self._stall_avoid_links.clear()
        self._abort_leg = True      # 배회/기존 경로를 즉시 접고 새 작업으로
        # [B안] 배회 중인 OHT는 대부분 hold() 상태(scheduled)라 passive가
        # 아니다. 그런 경우엔 여기서 activate할 필요 없이, 현재 진행 중인
        # 배회 스텝이 끝나는 대로 process() 루프가 self.job을 다시 확인해
        # 알아서 job 수행으로 넘어간다. 드물게 막다른 길 등으로 정말
        # passivate된 상태라면 즉시 깨워준다.
        if self.ispassive():
            self.activate()

    def _detach(self, job, back_to_pool=True):
        """이 차량에 물려 있는 job을 떼어낸다.
        현재 작업 슬롯이든 예약(next_job) 슬롯이든 알아서 찾는다.
        자재를 이미 실은 작업(JOB_CARRYING)은 절대 여기로 들어오지 않는다."""
        if job is None:
            return
        if self.next_job is job:
            self.next_job = None
        elif self.job is job:
            self.job = None
            self.state = OHT_IDLE
            self.color = IDLE_COLOR
            self._abort_leg = True
            if self not in idle_pool:
                idle_pool.append(self)
        if job.oht is self:
            job.oht = None
            job.state = JOB_PENDING
            if back_to_pool and job not in PENDING_JOBS:
                PENDING_JOBS.append(job)

    def take_job(self, job):
        """이 OHT가 job을 맡는다.

        [후보 조건 (3) 처리] 이 차량이 이미 자재를 싣고 가는 중이면
        (곧 유휴가 될 차량으로 뽑힌 경우) 운반 중인 작업을 절대 버리지 않고
        '다음 작업'으로 예약만 해둔다. 하차를 끝내는 즉시 이어서 착수한다.
        아직 픽업 전이라면 들고 있던 작업을 풀에 돌려보내고 새 작업으로 교체한다."""
        if self.job is job or self.next_job is job:
            return True                     # 이미 이 차량 것 - 손댈 필요 없음
        prev = job.oht
        if prev is not None and prev is not self:
            if not job.reassignable:
                # 이미 자재를 실은 작업은 절대 못 뺏는다(물리적으로 불가능).
                return False
            prev._detach(job, back_to_pool=False)

        job.n_assign += 1
        if self.job is not None and self.job.state == JOB_CARRYING:
            # 운반 중 -> 다음 작업으로 예약
            if self.next_job is not None:
                self._detach(self.next_job, back_to_pool=True)
            self.next_job = job
            job.oht = self
            job.state = JOB_ASSIGNED
            job.assigned_t = self.env.now()
            if job in PENDING_JOBS:
                PENDING_JOBS.remove(job)
            return True

        # 픽업 전이거나 유휴 -> 즉시 교체해서 착수
        if self.job is not None:
            self._detach(self.job, back_to_pool=True)
        if self.next_job is not None:
            self._detach(self.next_job, back_to_pool=True)
        self._begin_job(job)
        return True

    def _drop_job(self, back_to_pool=True):
        """현재 맡고 있는 작업을 놓는다(픽업 전에만 호출된다)."""
        self._detach(self.job, back_to_pool=back_to_pool)

    # ---- 후보 선별용 조회 ----------------------------------------------
    def eta_to_free(self):
        """이 OHT가 "유휴가 될 때"까지 남은 예상 시간(초).
        정체 대기는 빼고 순수 주행 기준으로만 본다."""
        if self.state == OHT_IDLE:
            return 0.0
        if self._leg_cum is None or self._leg_path is None:
            return 0.0
        remain = self._leg_cum[-1] - self._leg_cum[min(self._leg_idx,
                                                       len(self._leg_cum) - 1)]
        if self.state == OHT_TO_DEST:
            # 도착하면 하차(Load) 시간까지 끝나야 유휴가 된다.
            return max(0.0, remain) + OHT_LOAD_UNLOAD_TIME
        return max(0.0, remain)

    def score_origin_node(self):
        """3단계 W_movetime을 잴 기준 위치.
        - IDLE / 픽업하러 가는 중(=아직 자재 없음): 지금 있는 노드
        - 하차하러 가는 중: 곧 유휴가 될 위치인 하차지점
        (사용자 확인: (3) 후보는 하차지점 기준으로 잰다)"""
        if self.state == OHT_TO_DEST and self.job is not None:
            return self.job.to_node
        return self.node

    def process(self):
        while True:
            job = self.job
            if job is None:
                # ---- 미배정(Idle) 상태 ----
                # 예전에는 인접 노드로 무작위 배회했지만, 실제 OCS는 Idle
                # 차량에게 (1) 길 비켜주기 (2) Zone별 분산 배치 두 가지
                # Route 명령을 내린다. 아래가 그 구현이다.
                self.state = OHT_IDLE
                self.color = IDLE_COLOR
                self._abort_leg = False

                # (1) 길 비켜주기: 작업 중인 차량이 내 자리를 기다린다는
                #     통보(request_yield)를 받았으면 즉시 한 칸 물러난다.
                #     [버그 수정] 플래그는 여기서 끄지 않는다 - _yield_step()
                #     이 실제로 움직였을 때만 끈다. 갈 곳이 없어 실패하면
                #     플래그가 켜진 채로 남아 다음 루프에서 다시 시도한다
                #     (예전엔 여기서 미리 꺼버려서, 실패한 시도가 그대로
                #     유실되고 다시는 재시도가 안 돼 교착상태가 생겼다).
                if self._yield_request:
                    yield from self._yield_step()
                    continue

                # (2) 존 분산: IdleDistributor가 배정해준 대기지점으로 이동.
                tgt = self._idle_target
                if tgt is not None and tgt != self.node:
                    yield from self._goto(tgt)
                    continue

                # 대기지점에 도착했거나 아직 배정을 못 받았으면 그 자리에서
                # 대기한다(무작위 배회를 하지 않으므로 불필요한 노드락 경합과
                # 주행거리가 사라진다). 배정/비켜주기 요청은 언제든 이 hold를
                # 깨우고 들어올 수 있다.
                yield self.hold(_SIM_RNG.uniform(IDLE_STANDBY_HOLD_MIN,
                                               IDLE_STANDBY_HOLD_MAX))
                continue

            # ---- 1) 빈 차로 픽업지점(FromPort)까지 이동 ----
            self.state = OHT_TO_PICKUP
            self.color = EMPTY_COLOR
            self._abort_leg = False
            arrived = yield from self._goto(job.from_node)
            if not arrived:
                if self._abort_leg:
                    # 주행 도중 다른 차량에 Reassign 됐거나(take_job) 작업을
                    # 놓았다. 새 상황으로 루프를 다시 돈다.
                    continue
                # 재시도를 다 쓰고도 길이 없다 -> 작업을 풀에 반납
                self._drop_job(back_to_pool=True)
                continue
            if self.job is not job:
                continue

            # ---- 2) 픽업: 포트 -> OHT (SEMI 기준 Unload) ----
            # 이 시점을 넘기면 자재가 차 위에 있으므로 더 이상 Reassign 불가.
            self.state = OHT_UNLOADING
            job.state = JOB_CARRYING
            yield self.hold(OHT_LOAD_UNLOAD_TIME)
            self._job_loadunload += OHT_LOAD_UNLOAD_TIME
            self.color = LOADED_COLOR

            # ---- 3) 자재를 싣고 목적지(ToPort)로 ----
            self.state = OHT_TO_DEST
            self.color = LOADED_COLOR
            yield from self._goto(job.to_node)

            # ---- 4) 하차: OHT -> 포트 (SEMI 기준 Load) ----
            self.state = OHT_LOADING
            yield self.hold(OHT_LOAD_UNLOAD_TIME)
            self._job_loadunload += OHT_LOAD_UNLOAD_TIME

            # ---- 5) 작업 종료 ----
            # [용량 산정용] 이번 leg 1건의 운반시간(cycle) = 배정~완료까지
            # 총 경과시간. 가동률 계산용 누적치와 건별 통계에 모두 반영한다.
            # [5번 요청사항 - 의미 변화] 로트 모델 도입 전에는 job 1건이
            # 곧 자재 1건의 전체 이동이라 이 cycle이 "자재가 A에서 B까지
            # 가는 데 걸린 시간" 그 자체였다. 이제는 로트가 여러 leg로
            # 쪼개져서, 여기 JOB_STATS는 "leg 1건(예: STK->EQ)의 운반시간"
            # 이고, 로트 전체(STK 출발~최종 STK 복귀)는 LOT_STATS에 따로
            # 쌓인다. 둘을 혼동하지 않도록 이름을 구분해뒀다.
            cycle_time = self.env.now() - self.job_start_t
            self.total_busy += cycle_time
            JOB_STATS.append({
                "cycle": cycle_time,
                "wait": self._job_wait,
                "travel": self._job_travel,
                "process": self._job_process,
                "loadunload": self._job_loadunload,  # H/W Spec Load/Unload 누적시간
                "t_end": self.env.now(),  # [시계열용] 완료 시각(초, sim 시작 기준)
                "job_id": job.job_id,
                "lot_id": job.lot_id,       # 로트에 속하지 않으면 None(실측모드)
                "assign_delay": (job.assigned_t - job.created_t)
                                if job.assigned_t is not None else 0.0,
                "n_assign": job.n_assign,   # 1보다 크면 Reassign을 겪은 작업
            })

            job.state = JOB_DONE
            job.oht = None
            JOB_POOL.pop(job.job_id, None)
            manager.finished += 1

            # [5번 요청사항] 이 leg가 로트 소속이면 다음 단계로 진행시킨다.
            # EQ에 도착한 leg면 가공(PROCESS_TIME)이 OHT와 무관하게 별도
            # 컴포넌트로 흘러간 뒤에 다음 leg가 생긴다 - 그래서 OHT는 여기서
            # 바로 풀려나고, 가공이 끝나면 그때 배정 가능한 아무 OHT나
            # (같은 OHT일 수도, 다른 OHT일 수도) 다음 leg를 맡는다.
            if job.lot_id is not None:
                if job.to_kind == "EQ":
                    LotProcessAndAdvance(lot_id=job.lot_id, eq_node=job.to_node)
                else:
                    advance_lot(job.lot_id, self.env.now())

            self.job = None
            self.state = OHT_IDLE
            self.color = IDLE_COLOR
            self._leg_path = self._leg_cum = None
            self._leg_idx = 0
            if self not in idle_pool:
                idle_pool.append(self)

            # 후보조건 (3)으로 미리 예약해둔 다음 작업이 있으면 곧바로 착수.
            # (n_assign은 예약 시점에 이미 셌으므로 여기서 다시 세지 않는다.)
            nxt, self.next_job = self.next_job, None
            if nxt is not None and nxt.oht is self:
                self._begin_job(nxt)
            # 예약이 없으면 다음 작업은 Dispatcher의 10초 주기 라운드에서
            # 받는다(예전의 즉시 이어받기는 배치 배정과 맞지 않아 제거).

    def _path_clear_ahead(self, path, idx):
        """path[idx](진입하려는 노드)로부터 MIN_FOLLOW_DISTANCE 이내에 다른
        OHT가 있으면 False. 차간 최소거리 유지용 lookahead 체크(실제 물리적
        안전거리 - H/W Spec 기반, 절대 건드리면 안 됨).

        [길 비켜주기 - 근본원인 탐색] 막혀서 못 가는 상황이면, 그와는 별개로
        더 넓은 범위(YIELD_SCAN_DISTANCE)를 마저 훑어서 그 안에 있는 유휴
        차량 전부에게 비켜달라고 요청한다. MIN_FOLLOW_DISTANCE 체크와 달리
        여기서는 처음 걸리는 것에서 멈추지 않는다 - 중간에 작업 중인 차가
        끼어 있어도 건너뛰고 계속 찾는다. 실제로 막고 있는 게 몇 칸 뒤의
        유휴 차(근본 원인)일 때, 그 차를 직접 찾아서 미리 비켜달라고
        해두기 위함이다(바로 앞차만 비켜서는 못 뚫리는 경우 대응)."""
        cum = 0.0
        j = idx
        blocked = False
        while j < len(path) - 1 and cum < MIN_FOLLOW_DISTANCE:
            a, b = path[j], path[j + 1]
            next_cum = cum + edge_length(a, b)
            if FOLLOW_DISTANCE_CHECK_BEFORE_OCCUPANCY \
                    and next_cum > MIN_FOLLOW_DISTANCE:
                # b는 기준 거리 밖이다. 거리를 더하기 전에 멈춘다.
                break
            cum = next_cum
            holder = NODE_HOLDER.get(b)
            if holder not in (None, self) and not blocked:
                blocked = True
                if self.job is not None:
                    request_yield(b, self)
            j += 1
        if not blocked:
            return True

        # [근본원인 탐색] 물리적으로 막힌 게 확인됐으니, 그 지점부터
        # YIELD_SCAN_DISTANCE까지 계속 훑어 유휴 차 전부에게 통보한다.
        if self.job is not None:
            while j < len(path) - 1 and cum < YIELD_SCAN_DISTANCE:
                a, b = path[j], path[j + 1]
                cum += edge_length(a, b)
                holder = NODE_HOLDER.get(b)
                if holder not in (None, self):
                    request_yield(b, self)
                j += 1
        return False

    def _traffic_stop_limit(self, path, idx, cur_li):
        """앞쪽 경로에 다른 OHT가 점유한 노드가 있으면, 그 직전 노드에서
        정지할 수 있도록 지금 구간의 탈출속도 상한을 돌려준다.

        _reroute_stop_limit()과 방식은 같고 정지 지점만 다르다. 그쪽은
        재탐색 때문에 서야 하는 분기점을 찾고, 여기서는 다른 OHT가
        점유한 노드를 찾는다. 제동거리(2,042mm)만큼 앞을 훑는 이유도
        같다 - 링크 길이 중앙값이 1,700mm라 한 링크만 보고 감속하면
        제때 못 선다.

        [한계] 예측이므로 완전하지 않다. 감속을 시작한 뒤 앞이 뚫리면
        불필요하게 느려지고(실제 차량도 마찬가지다), 진입한 뒤에 새로
        막히면 여전히 즉시 정지가 일어난다. 발생 빈도를 줄이는
        완화책이지 원인 제거가 아니다.

        반환 inf는 제동거리 안에 막힌 노드가 없다는 뜻이다."""
        max_brake_dist = OHT_MAX_SPEED * OHT_MAX_SPEED / (2.0 * OHT_DECEL)
        d_ahead = 0.0   # path[idx]부터 path[j]까지의 거리
        d_prev = 0.0    # path[idx]부터 '설 수 있는 마지막 노드'까지의 거리
        j = idx
        while j < len(path) and d_ahead <= max_brake_dist:
            holder = NODE_HOLDER.get(path[j])
            if holder is not None and holder is not self:
                # path[j]에는 못 들어간다. 그 직전 노드에서 서야 하므로
                # 남은 거리 d_prev 안에 0으로 줄일 수 있어야 한다.
                return math.sqrt(2.0 * OHT_DECEL * d_prev)
            d_prev = d_ahead
            if j >= len(path) - 1:
                break
            li = EDGE_LINK_IDX.get((path[j], path[j + 1]))
            if li is None:
                break
            d_ahead += LINK_LEN[li]
            j += 1
        return math.inf

    def _goto(self, target):
        """target 노드까지 주행한다. 재탐색 트리거가 걸리면 지금 위치에서
        경로를 다시 잡아 이어간다. 도착하면 True.

        False를 돌려주는 경우는 두 가지다.
          - _abort_leg: Reassign 등으로 이 작업 자체를 놓았다.
          - 경로 탐색이 MAX_PATH_RETRY번 연속 실패했다(작업을 반납해야 함).
        """
        fails = 0
        while True:
            if self._abort_leg:
                return False
            if self.node == target:
                return True

            self._replan = False
            avoid = self._avoid_links
            # 달리는 중이면 진입 링크를 넘겨 '정지 가정'으로 인한 경로
            # 흔들림을 막는다. 멈춰 있으면(정체 후 등) 정지 출발이 맞다.
            fl = self._arrival_link if self._cur_speed > 1e-6 else None
            blocked_next = self._stall_next   # [진단용] 이번 재탐색이 피하려던 노드
            failed_detour_link = None
            path = shortest_path(self.node, target, avoid=avoid, from_link=fl)
            if path is None and avoid:
                # 우회로가 없으면 임시 제외를 풀고 원래 경로라도 잡는다
                # (그 링크가 뚫릴 때까지 기다리는 편이 낫다).
                path = shortest_path(self.node, target, from_link=fl)
                # 회피 실패 후 같은 첫 링크로 복귀하면 이번 진입 시도에서는
                # 즉시 재탐색을 반복하지 않고 기존 노드 대기 절차를 따른다.
                # 경로를 다시 계산할 때마다 초기화하므로 이후 우회는 유지된다.
                if path and len(path) > 1:
                    first_link = EDGE_LINK_IDX.get((path[0], path[1]))
                    if first_link in avoid:
                        failed_detour_link = first_link
                # [진단] 결과 경로가 여전히 그 막힌 노드로 향하면, 재탐색은
                # 정상 발동했지만 이 지점엔 진짜 대안이 없었다는 뜻이다.
                # "재탐색이 안 먹힌다"는 증상은 대개 버그가 아니라 이 경우다.
                if path and len(path) > 1 and blocked_next is not None \
                        and path[1] == blocked_next:
                    REROUTE_COUNTS["no_alt"] += 1
            self._avoid_links = None

            if path is None:
                # ---- 트리거 (4): 경로 탐색 실패 -> 5초 쿨타임 후 재시도 ----
                # 전방 구간 일시 폐쇄 등으로 길이 아예 없는 상황이다.
                fails += 1
                REROUTE_COUNTS["fail"] += 1
                if fails > MAX_PATH_RETRY:
                    return False
                yield self.hold(PATH_RETRY_COOLDOWN)
                continue

            fails = 0
            yield from self._travel_path(path, failed_detour_link=failed_detour_link)

    def _reroute_stop_limit(self, path, idx, cur_li):
        """지금 주행하는 링크의 '탈출속도 상한'을 돌려준다.

        회사 방법론 (1)번 재탐색은 분기 노드에서 차량을 세우고 수행한다.
        그러면 앞쪽에 곧 정지해야 할 분기점이 있는 경우 미리 감속해야 한다.
        3.5m/s에서 완전히 서려면 2,042mm가 필요한데 링크 길이 중앙값은
        1,700mm라, 한 링크만 보고 감속하면 제때 못 선다. 그래서 경로를 앞으로
        훑으며 '언제 어디서 서야 하는지'를 찾아 필요한 진입속도를 계산한다.

        반환 inf는 이 구간에서 정지 제약이 없다는 뜻이다."""
        now = self.env.now()
        # 이 링크를 통과하는 데 걸릴 시간의 하한(상한속도로 달렸을 때).
        # 정지 시점을 조금 이르게 잡는 쪽이 안전하다.
        t = now + LINK_LEN[cur_li] / max(LINK_CAP[cur_li], 1.0)
        max_brake_dist = OHT_MAX_SPEED * OHT_MAX_SPEED / (2.0 * OHT_DECEL)
        d_ahead = 0.0
        j = idx
        while j < len(path) and d_ahead <= max_brake_dist:
            if (path[j] in JUNCTION_NODES
                    and t - self._last_reroute_t >= JUNCTION_REROUTE_MIN_GAP):
                # path[j]에서 정지. 거기까지 d_ahead가 남았으므로 지금 구간의
                # 탈출속도는 그 거리 안에 0으로 줄일 수 있는 값이어야 한다.
                return math.sqrt(2.0 * OHT_DECEL * d_ahead)
            if j >= len(path) - 1:
                break
            li = EDGE_LINK_IDX.get((path[j], path[j + 1]))
            if li is None:
                break
            d_ahead += LINK_LEN[li]
            t += LINK_LEN[li] / max(LINK_CAP[li], 1.0)
            j += 1
        return math.inf

    def _travel_path(self, path, failed_detour_link=None):
        """path의 각 노드를 순서대로 이동.
        [2500대 대응] 노드 락 자체가 막혀있을 때는 busy-poll 대신
        NODE_WAITERS에 등록하고 passivate() 하여 release() 쪽에서 깨워줄 때만
        재시도한다(엔진 이벤트 폭주 방지, 2500대 규모의 핵심 병목이었음).
        차간 최소거리(follow-distance) 체크는 노드 하나가 아니라 경로상
        여러 노드를 동시에 봐야 해서 event-driven화하기 까다로우므로,
        이 경우에만 예전처럼 짧은 hold(0.2) 재시도를 유지한다(발생 빈도가
        노드 락 경합보다 훨씬 낮아 부담이 작다).
        진입에 성공하면 이전 노드 lock을 즉시 해제한다(데드락 방지 핵심).
        구간별로 가속/등속/감속 프로파일을 계산해 자연스럽게 움직이고,
        경로의 마지막 노드에서만 속도 0으로 정지한다."""
        last_idx = len(path) - 1
        # [구간 진행상황] 후보 선별 1-(3)과 4단계 재탐색이 "남은 예상 주행
        # 시간"을 물어볼 수 있도록, leg를 시작할 때 누적 소요시간표를 한 번만
        # 만들어둔다. 이후엔 노드를 지날 때마다 _leg_idx만 올린다.
        self._leg_path = path
        _total, self._leg_cum = path_travel_cumtime(path)
        self._leg_idx = 0

        for idx in range(1, len(path)):
            if self._abort_leg:
                # Reassign 등으로 이 경로를 더 따라갈 이유가 없어졌다.
                # 지금 노드의 락은 그대로 쥔 채 빠져나간다(다음 루프에서
                # 새 경로를 잡을 때 이어서 쓴다).
                return
            nxt = path[idx]

            # ---- 트리거 (3): 전방 경로 차단 시 즉시 재탐색 ----
            li_next = EDGE_LINK_IDX.get((self.node, nxt))
            if li_next is not None and li_next in BLOCKED_LINKS:
                self._replan = True
                REROUTE_COUNTS["blocked"] += 1
                return
            if self._replan:
                # 외부(notify_blocked_ahead / 정체 감시)에서 이미 요청됨
                return

            # ---- 트리거 (1): 분기점 접근 시 재탐색 (최소 5초 간격) ----
            # 분기 노드에 도달할 때마다 전방 상황을 다시 보고 경로를 갱신한다.
            # 고속 주행 중 연속된 분기점으로 부하가 폭주하지 않도록 5초 간격
            # 제한을 둔다.
            if (ENABLE_JUNCTION_REROUTE
                    and self.node in JUNCTION_NODES
                    and self.env.now() - self._last_reroute_t
                    >= JUNCTION_REROUTE_MIN_GAP):
                # 실제 설비에서 이 재탐색은 차량을 세우고 수행한다.
                # 앞 구간에서 _reroute_stop_limit()으로 미리 감속해왔으므로
                # 여기서 속도가 이미 0에 가깝다. 남은 잔속은 정지로 확정한다.
                self._last_reroute_t = self.env.now()
                self._cur_speed = 0.0
                self._replan = True
                REROUTE_COUNTS["junction"] += 1
                return

            # [정체 추적] 이 노드에 진입하려고 재시도를 시작한 시각. 락 대기든
            # follow-distance 대기든 이 OHT가 물리적으로는 self.node에 멈춰
            # 있는 같은 상태라, 하나의 연속 정체 구간으로 합쳐서 잰다.
            # 첫 시도에 바로 성공하면(가장 흔한 경우) stall_start가 None으로
            # 남아 정체 이벤트 자체가 기록되지 않는다.
            # [원인 분해] 다만 "왜" 막혔는지는 지표로서 의미가 다르므로
            # (노드락 = 교차점 경합, follow = 앞차에 바로 막힘) 원인별 시간을
            # 따로도 누적한다.
            stall_start = None
            stall_lock = 0.0      # 이번 노드 진입에서 노드락 때문에 기다린 시간
            stall_follow = 0.0    # 이번 노드 진입에서 앞차(차간거리) 때문에 기다린 시간
            self._stall_next = nxt
            bail = False

            while True:
                if self._replan or self._abort_leg:
                    # 정체 감시(트리거 2)나 차단 통지(트리거 3)가 깨웠다.
                    # nxt 락은 쥐고 있지 않은 상태이므로 그냥 빠져나간다.
                    cancel_wait(nxt, self)
                    bail = True
                    break
                # ---- [고리 진입 제어] 꽉 찬 고리로는 들어가지 않는다 ----
                if (ENABLE_RING_CONTROL and not self._ring_exempt
                        and ring_entry_blocked(self, nxt)):
                    if stall_start is None:
                        stall_start = self.env.now()
                        self._stall_start = stall_start
                        RING_STATS["wait_events"] += 1
                    # (a) 분기점이면 꽉 찬 고리로 안 들어가는 다른 갈래로 강제 우회
                    if (self.node in JUNCTION_NODES
                            and self._ring_detour_node != self.node
                            and any(not ring_entry_blocked(self, LINK_B[_oli])
                                    for _oli in OUT_LINKS[self.node]
                                    if _oli != li_next)):
                        self._ring_detour_node = self.node
                        self._avoid_links = {li_next} if li_next is not None else None
                        self._cur_speed = 0.0
                        self._replan = True
                        RING_STATS["detour"] += 1
                        return
                    # (b) 우회 불가 -> 그 자리에서 대기 후 재확인
                    self._cur_speed = 0.0
                    t_wait = self.env.now()
                    self._ring_waiting = True
                    try:
                        yield self.hold(RING_WAIT_POLL)
                    finally:
                        self._ring_waiting = False
                    _dt = self.env.now() - t_wait
                    self._job_wait += _dt
                    RING_STATS["wait_time"] += _dt
                    if li_next is not None:
                        LINK_STALL[li_next] += _dt
                    continue
                if not try_acquire(nxt, self):
                    if stall_start is None:
                        stall_start = self.env.now()
                        self._stall_start = stall_start
                        # [즉시 갈래전환] 분기점에서 막 막힌 순간이면 10초씩
                        # 기다리지 않고 그 자리에서 바로 다른 갈래가 있는지
                        # 확인한다. [버그 수정] "확인만 하고 무조건 재탐색"이
                        # 아니라, 다른 출구 중 "지금 실제로 비어있는 곳이
                        # 있을 때만" 재탐색한다 - 양쪽 다 막혀있으면(사용자
                        # 확인: 이 경우는 재탐색 못 하는 게 맞음) 그냥
                        # 아래 평소 대기로 넘어간다. 이 확인 없이 무조건
                        # avoid만 걸면, 두 출구가 서로를 막고 있는 상황에서
                        # "A 회피->B로 재탐색->B도 막힘->B 회피->다시 A로
                        # 재탐색->..."가 시뮬레이션 시간이 전혀 안 흐른 채
                        # 무한 반복되는 버그가 있었다(실측: 300대에서 실행이
                        # 아예 멈춤). 진짜로 비어있는 대안이 있을 때만
                        # 트리거하면 이 핑퐁 자체가 성립하지 않는다.
                        if (ENABLE_JUNCTION_REROUTE and self.node in JUNCTION_NODES
                                and not (idx == 1 and li_next == failed_detour_link)):
                            alt_free = any(
                                NODE_HOLDER.get(LINK_B[_oli]) is None
                                for _oli in OUT_LINKS[self.node]
                                if _oli != li_next
                            )
                            if alt_free:
                                self._last_reroute_t = self.env.now()
                                self._avoid_links = {li_next} if li_next is not None else None
                                self._cur_speed = 0.0
                                self._replan = True
                                REROUTE_COUNTS["junction"] += 1
                                return
                    self._cur_speed = 0.0
                    t_wait = self.env.now()
                    yield wait_for_node(nxt, self)  # release() 시점에 activate될 때까지 잠듦
                    _dt = self.env.now() - t_wait
                    self._job_wait += _dt
                    stall_lock += _dt
                    if li_next is not None:
                        LINK_STALL[li_next] += _dt
                    continue
                if not self._path_clear_ahead(path, idx):
                    if stall_start is None:
                        stall_start = self.env.now()
                        self._stall_start = stall_start
                    release(nxt, self, self.env.now())
                    self._cur_speed = 0.0
                    t_wait = self.env.now()
                    yield self.hold(0.2)  # follow-distance는 짧은 재시도로 유지
                    _dt = self.env.now() - t_wait
                    self._job_wait += _dt
                    stall_follow += _dt
                    if li_next is not None:
                        LINK_STALL[li_next] += _dt
                    continue
                break

            # [합류부 우선권] nxt 락을 얻었다 = 합류부 통과를 승인받았다.
            # 이 OHT가 타고 들어온 진입경로(li_next)에 우선권을 준다. 점유시간
            # 안에 같은 경로로 후속 OHT가 오면 그 차량이 다른 경로 차량보다
            # 먼저 통과한다(release()에서 처리).
            if nxt in MERGE_POINT_NODES:
                grant_merge_priority(nxt, li_next, self.env.now())

            self._stall_start = None
            self._stall_next = None

            if stall_start is None:
                # [누적 정체 리셋] 이번 노드는 막힘 없이 바로 통과했다 =
                # 진짜로 정체가 풀렸다는 뜻이므로, 그동안 쌓아둔 강제우회
                # 누적치를 비운다. 재탐색으로 잠깐 다른 노드에서 대기
                # 없이 지나갔다고 해서 리셋하면(중간에 stall_start가 있는
                # 경우도 여기 들어오면) 누적 취지가 무색해지므로, 정말
                # "이번 노드는 기다린 적이 전혀 없다"일 때만 리셋한다.
                self._first_stall_t = None
                self._stall_avoid_links.clear()

            if stall_start is not None:
                stall_dur = self.env.now() - stall_start
                self.total_stall_time += stall_dur  # [병목률용] 임계값과 무관하게 전부 누적
                self.total_lock_stall_time += stall_lock
                self.total_follow_stall_time += stall_follow
                # [구역별 집계] 이 OHT가 실제로 멈춰 서 있던 위치(self.node)가
                # 어느 베이/중앙통로/합류부인지로 정체 시간을 누적한다.
                # 임계값(STALL_THRESHOLD_SECONDS)과 무관하게 "모든" 정체를
                # 더하므로, 짧지만 잦은 정체까지 구역 지표에 반영된다.
                _accumulate_region_stall(self.node, stall_dur, stall_lock, stall_follow)
                if stall_dur >= STALL_THRESHOLD_SECONDS:
                    STALL_EVENTS.append({
                        "oht": self.oht_id,
                        "node": self.node,       # 정체가 실제로 일어난 위치(진입 못하고 멈춰있던 곳)
                        "zone": NODE_ZONE.get(self.node),  # EQ/STK 포트가 아니면 None
                        "bay_id": NODE_TO_BAY_ID.get(self.node),
                        "corridor_id": NODE_TO_CORRIDOR_ID.get(self.node),
                        "merge_ids": NODE_TO_MERGE_ID.get(self.node, []),
                        "t_start": stall_start,
                        "duration": stall_dur,
                        "lock_wait": stall_lock,
                        "follow_wait": stall_follow,
                    })

            if bail:
                return

            x0, y0 = NODE_XY[self.node]
            x1, y1 = NODE_XY[nxt]
            self._update_heading(x0, y0, x1, y1)

            # [시간 기반 라우팅과 동일한 물리모델]
            # 지금 주행하는 링크(cur_li = self.node -> nxt)의 속도상한과,
            # 그 끝에서의 탈출속도를 JUNCTION_SPEED에서 그대로 가져온다.
            # 이 표는 사전감속을 이미 여러 홉에 걸쳐 전파해뒀기 때문에,
            # "다음 링크만 보고 감속하다 거리가 모자라 커브에 과속 진입"
            # 하는 예전 문제가 구조적으로 생기지 않는다. 라우팅이 예측한
            # 소요시간과 실제 주행시간이 같은 식에서 나오게 된다.
            cur_li = EDGE_LINK_IDX.get((self.node, nxt))
            if cur_li is None:      # 그래프에 없는 조합(방어적)
                dist = edge_length(self.node, nxt)
                cap_v, target_v = OHT_MAX_SPEED, 0.0
            else:
                dist = LINK_LEN[cur_li]
                cap_v = LINK_CAP[cur_li]
                if idx == last_idx:
                    target_v = 0.0   # 경로 끝에서는 정지(포트 Load/Unload)
                else:
                    nxt_li = EDGE_LINK_IDX.get((nxt, path[idx + 1]))
                    target_v = JUNCTION_SPEED.get((cur_li, nxt_li), 0.0) \
                        if nxt_li is not None else 0.0
                # [(1)번 재탐색은 정지 후 수행] 곧 재탐색을 위해 서야 할
                # 분기점이 앞에 있으면 미리 감속한다.
                if ENABLE_JUNCTION_REROUTE:
                    target_v = min(
                        target_v, self._reroute_stop_limit(path, idx, cur_li))
                # [막힘 정지 사전감속] 앞차 때문에 서야 하는 경우도 미리
                # 감속한다. 원문은 이 경우 감속 없이 즉시 정지시켰다.
                if PREDECEL_FOR_BLOCKED_NODE:
                    target_v = min(
                        target_v, self._traffic_stop_limit(path, idx, cur_li))
            prof = self._segment_profile(self._cur_speed, target_v, cap_v, dist)

            self._seg_profile = prof
            self._seg_start_t = self.env.now()
            self._seg_start_xy = (sx(x0), sy_screen(y0))
            self._seg_end_xy = (sx(x1), sy_screen(y1))
            self._seg_dist = dist

            # [조기 반납] 안전거리만큼 전진한 시점에 출발 노드를 놓아
            # 뒤차가 따라붙을 수 있게 한다. 링크가 안전거리보다 짧으면
            # 도착이 먼저이므로 원문대로 한 번에 건넌다.
            t_clear = None
            if RELEASE_PREV_AT_SAFE_DISTANCE and dist > MIN_FOLLOW_DISTANCE:
                t_clear = self._profile_time(prof, MIN_FOLLOW_DISTANCE)
            if t_clear is not None and 0.0 < t_clear < prof["dur"]:
                yield self.hold(t_clear)
                # self.node는 아직 출발 노드다. 락만 먼저 놓는다.
                release(self.node, self, self.env.now())
                yield self.hold(prof["dur"] - t_clear)
            else:
                yield self.hold(prof["dur"])
            self._job_travel += prof["dur"]
            self.total_distance += dist

            self._cur_speed = prof["v_end"]
            if cur_li is not None:
                LINK_FLOW[cur_li] += 1.0
            self._arrival_link = cur_li   # 재탐색 시 '실제 진입 상태' 복원용
            prev_node = self.node
            self.node = nxt
            self._ring_detour_node = None   # [고리 진입 제어] 새 노드에선 다시 우회 가능
            self._leg_idx = idx
            release(prev_node, self, self.env.now())


# ----------------------------------------------------------------------
# 용량 산정 분석 (headless, 통계적 산출)
# ----------------------------------------------------------------------
# 절차:
#   1) 기준 대수(기본 MAX_OHT)로 sim_hours 시간 동안 돌려 로트 1건(leg 단위,
#      STK->[STB]->EQ->[STB]->STK)의 평균 운반시간 W를 측정한다. 로트
#      내용은 JOB_SCHEDULE(고정 스케줄)에서 순서대로 오므로, W는 그
#      스케줄에 대한 평균/표준편차로 산출된다.
#   2) Little's Law(정상상태에서 "평균 동시 진행 건수 L = 투입률 λ × 평균
#      체류시간 W")를 이용해, 목표 가동률 U(80~85%의 중앙값 82.5%)를
#      만족하는 필요 대수 N을 역산한다: N = λ·W / U.
#      여기서 λ = 1 / SPAWN_INTERVAL (초당 자재 투입률).
#   3) 역산된 N으로 실제로 재시뮬레이션해서 가동률이 80~85%에 들어오는지,
#      그 부하에서 처리된 운반(완료 작업) 건수는 얼마인지 확인한다.
#      목표 범위를 벗어나면 (측정가동률/목표가동률) 비율로 N을 보정해
#      최대 4회까지 재시도한다.
#   4) 병목 판정: 평균 대기시간(노드락 대기 + 설비 자원대기)이 전체
#      운반시간에서 차지하는 비중이 크거나(30% 초과), 대수를 늘려도
#      가동률이 목표 상한을 크게 초과한 채 못 내려오면 "병목 있음"으로
#      본다. 이때 WAIT_COUNTS 상위 노드를 혼잡 지점 후보로 같이 보여준다.
def reset_sim_state():
    """N을 바꿔가며 시뮬레이션을 반복 실행하기 위해 모듈 전역 가변 상태를
    초기화한다(OHT/노드락/설비상태/통계 전부)."""
    idle_pool.clear()
    STANDBY_ASSIGNED.clear()
    for _k in IDLE_STATS:
        IDLE_STATS[_k] = 0
    MERGE_HELD_LINK.clear()
    MERGE_HOLD_EXPIRE.clear()
    for k in MERGE_STATS:
        MERGE_STATS[k] = 0
    LINK_FLOW[:] = 0.0
    LINK_STALL[:] = 0.0
    LINK_DELAY[:] = 0.0
    LINK_PENALTY[:] = 1.0
    _on_blocked_changed()   # [결정론 보강] LINK_PENALTY를 1.0으로 되돌렸으면
    # W_movetime용 TIME_CSR도 그 값 기준으로 다시 구워야 한다. 이걸 빼먹으면
    # 직전 실행에서 CongestionMonitor가 갱신해둔 "낡은" TIME_CSR이 다음
    # 실행에 그대로 남아서(같은 seed를 둬도) 배정 스코어가 실행마다 달라지고,
    # 그 작은 차이가 누적돼 전체 시뮬레이션이 완전히 다른 경로로 갈라졌다
    # (실측: 같은 seed=1을 연달아 두 번 돌렸는데도 leg 완료 278건 vs 270건).
    for k in REROUTE_COUNTS:
        REROUTE_COUNTS[k] = 0
    if BLOCKED_LINKS:
        BLOCKED_LINKS.clear()
        _on_blocked_changed()
    JOB_POOL.clear()
    PENDING_JOBS.clear()
    _JOB_SEQ[0] = 0
    _JOB_SCHEDULE_IDX[0] = 0   # Job 스케줄도 매 실행 처음부터 다시 재생
    _SIM_RNG.seed(2)           # 유휴 지터도 매 실행 처음부터 다시 재생
    LOT_REGISTRY.clear()
    _LOT_SEQ[0] = 0
    LOT_STATS.clear()
    OHT_LIST.clear()
    reset_ring_state()        # [고리 진입 제어]
    for nid in NODE_LIST:
        NODE_HOLDER[nid] = None
        NODE_WAITERS[nid] = []
    for nid in EQ_STATUS:
        EQ_STATUS[nid] = "empty"
    EQ_ACTIVE.clear()
    EQ_RESOURCE.clear()  # 이전 env에 묶인 Resource라 반드시 새로 만들어야 함
    JOB_STATS.clear()
    WAIT_COUNTS.clear()
    STALL_EVENTS.clear()
    BAY_STALL_STATS.clear()
    CORRIDOR_STALL_STATS.clear()
    MERGE_STALL_STATS.clear()
    UNASSIGNED_STALL_STATS.update(_new_region_stat())


def run_headless_sim(num_oht, sim_duration, spawn_interval=None, seed=None,
                      return_raw=False):
    """pygame/PIL 렌더링 없이 salabim 엔진만으로 sim_duration(초)만큼
    실행하고 통계 dict를 반환한다."""
    global manager, dispatcher, SPAWN_INTERVAL

    if seed is not None:
        random.seed(seed)

    reset_sim_state()
    prev_spawn_interval = SPAWN_INTERVAL
    if spawn_interval is not None:
        SPAWN_INTERVAL = spawn_interval

    sim.yieldless(False)
    env = sim.Environment(trace=False)

    park_nodes = get_park_nodes(num_oht)   # 고정 시작위치(0번 요청사항)
    # [시간 기반 라우팅] 예전에는 여기서 출발가능 노드 전부에 대해 dijkstra를
    # 미리 구워두는 warmup을 했다. 이제 경로탐색은 pair-state A*로 질의마다
    # 계산하므로(평균 2.3ms) 사전 계산이 필요 없다. 혼잡도 동적 가중치가
    # 들어와도 구조를 그대로 쓸 수 있다.
    ohts = [OHT(park_node=n) for n in park_nodes]
    manager = Manager()
    dispatcher = Dispatcher()   # 10초 주기 스코어 배정
    StallWatchdog()             # 트리거(2) 분기점 정체 감시
    CongestionMonitor()         # 혼잡도 기반 동적 가중치
    IdleDistributor()           # 미배정 OHT를 Zone(베이) N등분 지점으로 분산

    env.run(till=sim_duration)

    SPAWN_INTERVAL = prev_spawn_interval

    n_jobs = len(JOB_STATS)
    if n_jobs == 0:
        avg_cycle = avg_wait = avg_travel = avg_process = avg_loadunload = stdev_cycle = 0.0
    else:
        cycles = [j["cycle"] for j in JOB_STATS]
        avg_cycle = statistics.mean(cycles)
        stdev_cycle = statistics.pstdev(cycles) if n_jobs > 1 else 0.0
        avg_wait = statistics.mean(j["wait"] for j in JOB_STATS)
        avg_travel = statistics.mean(j["travel"] for j in JOB_STATS)
        avg_process = statistics.mean(j["process"] for j in JOB_STATS)
        avg_loadunload = statistics.mean(j["loadunload"] for j in JOB_STATS)  # H/W Spec Load/Unload

    # 가동률 = (모든 OHT의 "배정~완료/현재까지" 누적 가동시간) / (대수*경과시간).
    # 완료된 작업의 시간(total_busy)에 더해, 종료 시점에 아직 작업 중이던
    # OHT의 "배정~종료시각"까지 경과분도 더한다. 이걸 빼먹으면 혼잡이
    # 심해 완료 건수가 0에 가까울 때 "다들 꽉 막혀 바쁜" 상태가 오히려
    # 가동률 0%로 잘못 계산되는 문제가 생긴다.
    completed_busy = sum(a.total_busy for a in ohts)
    in_flight_busy = sum(
        sim_duration - a.job_start_t
        for a in ohts
        if a.job is not None and a.job_start_t is not None
    )
    fleet_busy = completed_busy + in_flight_busy
    utilization = fleet_busy / (num_oht * sim_duration) if num_oht and sim_duration else 0.0
    throughput_per_hour = n_jobs / (sim_duration / 3600.0)

    # [가동률 이원화] 위 utilization은 "배정~완료까지"를 전부 가동으로 치는
    # 값이라, 노드락/설비 대기가 길어질수록 오히려 커지는 역설이 있다
    # (막혀서 못 움직이는데 가동률은 높게 나옴). utilization_active는 그중
    # 실제로 움직이거나(travel), 설비에서 가공(process) 중이거나, 포트에서
    # 싣고내리는(loadunload, H/W Spec) 시간만 뽑은 값으로, utilization과의
    # 차이(gap)가 클수록 "일감은 배정됐지만 정체로 못 움직이는" 비중이 크다는
    # 뜻이다. utilization_wait은 그 반대쪽(노드락 대기 + 설비자원 대기)만의
    # 비율이다.
    completed_active = sum(j["travel"] + j["process"] + j["loadunload"] for j in JOB_STATS)
    completed_wait = sum(j["wait"] for j in JOB_STATS)
    in_flight_active = sum(
        a._job_travel + a._job_process + a._job_loadunload
        for a in ohts
        if a.job is not None and a.job_start_t is not None
    )
    in_flight_wait = sum(
        a._job_wait
        for a in ohts
        if a.job is not None and a.job_start_t is not None
    )
    fleet_active = completed_active + in_flight_active
    fleet_wait = completed_wait + in_flight_wait
    utilization_active = fleet_active / (num_oht * sim_duration) if num_oht and sim_duration else 0.0
    utilization_wait = fleet_wait / (num_oht * sim_duration) if num_oht and sim_duration else 0.0

    # 대수는 많은데(예: 수백대 이상) 시뮬레이션 시간 내내 단 한 건도
    # 완료하지 못했다면, 그 자체가 심각한 정체(gridlock)의 신호다.
    gridlock = (n_jobs == 0 and num_oht >= 20)

    # [정체(stall) 요약] STALL_THRESHOLD_SECONDS(기본 10초) 이상 한 자리에
    # 멈춰있었던 이벤트들의 요약. 상세 원본은 STALL_EVENTS(모듈 전역)에
    # 그대로 남아있으니, 노드/zone별로 더 뜯어보고 싶으면 그걸 직접 쓰면 된다
    # (reset_sim_state()가 다음 실행 때 지우므로, 필요하면 이 함수 호출
    # 직후에 바로 복사해둘 것).
    n_stall_events = len(STALL_EVENTS)
    stalled_oht_ids = {e["oht"] for e in STALL_EVENTS}

    # [5번 요청사항] 로트(자재 1개, STK 출발~최종 STK 복귀) 단위 통계.
    # 위 n_jobs/avg_cycle 등은 전부 leg(구간) 단위임에 유의 - 로트 하나가
    # 보통 2~3개 leg로 쪼개지므로 n_jobs는 로트 수보다 몇 배 많이 나온다.
    n_lots = len(LOT_STATS)
    avg_lot_cycle = (statistics.mean(l["cycle"] for l in LOT_STATS)
                     if n_lots else 0.0)
    lot_throughput_per_hour = n_lots / (sim_duration / 3600.0) if sim_duration else 0.0

    result = {
        "num_oht": num_oht,
        "sim_duration": sim_duration,
        "spawn_interval": SPAWN_INTERVAL if spawn_interval is None else spawn_interval,
        "n_jobs": n_jobs,          # leg(구간) 완료 건수
        "avg_cycle": avg_cycle,    # leg 1건의 평균 운반시간
        "stdev_cycle": stdev_cycle,
        "avg_wait": avg_wait,
        "avg_travel": avg_travel,
        "avg_process": avg_process,
        "avg_loadunload": avg_loadunload,  # H/W Spec Load/Unload 평균 누적시간
        "n_lots": n_lots,                          # 로트(자재) 완료 건수
        "avg_lot_cycle": avg_lot_cycle,             # 로트 전체 평균 소요시간(STK~STK)
        "lot_throughput_per_hour": lot_throughput_per_hour,
        "utilization": utilization,
        "utilization_active": utilization_active,
        "utilization_wait": utilization_wait,
        "throughput_per_hour": throughput_per_hour,   # leg 처리량(참고용)
        "top_congested": WAIT_COUNTS.most_common(5),
        "gridlock": gridlock,
        "n_stall_events": n_stall_events,
        "n_stalled_oht": len(stalled_oht_ids),
        "stalled_oht_ratio": len(stalled_oht_ids) / num_oht if num_oht else 0.0,
        # [고리 진입 제어] 강제 우회 횟수 / 진입 대기 발생 횟수 / 총 대기시간
        "ring_detours": RING_STATS["detour"],
        "ring_wait_events": RING_STATS["wait_events"],
        "ring_wait_time": RING_STATS["wait_time"],
    }
    if return_raw:
        # [시계열/구간별 분석용] 다음 run_headless_sim() 호출이 reset_sim_state()로
        # JOB_STATS/STALL_EVENTS를 지워버리기 전에, 그 원본을 그대로 복사해
        # 결과에 실어둔다. compute_timeseries()가 이걸 그대로 받아 쓴다.
        result["job_stats"] = list(JOB_STATS)
        result["stall_events"] = list(STALL_EVENTS)
        # [구역별 분석용] compute_region_congestion()이 그대로 받아 쓴다.
        result["wait_counts"] = dict(WAIT_COUNTS)
        # [CSV 출력용] OHT 인스턴스 자체를 실어준다. reset_sim_state()는
        # OHT_LIST만 비우고 인스턴스는 살아있으므로, 호출부가 다음 회차
        # 실행 전에 export_oht_metrics_csv(result["ohts"], ...)를 부르면 된다.
        result["ohts"] = ohts
    return result


# ----------------------------------------------------------------------
# 구역별(베이/중앙통로) 정체 리포트
# ----------------------------------------------------------------------
# [요청 반영] "구역을 쪼개 정체가 어디서 많이 발생했는지 구분 가능하도록".
# WAIT_COUNTS(노드별 락 대기 발생 횟수)와 STALL_EVENTS(10초+ 정체 이벤트,
# 위치=node 포함)를 NODE_TO_SECTIONS로 섹션에 대응시키고, 섹션의 베이/
# 중앙통로 분류(SECTION_INFO)에 따라 다시 묶어서 "어느 베이가 제일 막히는지
# / 중앙통로 전체는 얼마나 막히는지"를 한눈에 보여준다.
def compute_region_congestion(wait_counts, stall_events, top_n=15):
    """wait_counts: {node_id: 락대기 발생 횟수}, stall_events: STALL_EVENTS
    형태의 리스트(run_headless_sim(..., return_raw=True)의 결과를 그대로
    넣으면 된다). 콘솔에 요약을 출력하고, 이후 CSV 저장 등에 쓸 수 있게
    베이/중앙통로/미분류 집계 dict를 반환한다.
    [세분화] section_id 단위(731개, 너무 잘게 쪼개져 있어 해석이 어려움)
    대신 NODE_TO_BAY_ID/NODE_TO_CORRIDOR_ID로 물리적 "베이 1~N",
    "중앙통로 1~2" 단위로 묶어서 훨씬 읽기 쉬운 리포트를 만든다."""
    if not NODE_TO_BAY_ID and not NODE_TO_CORRIDOR_ID:
        print("[구역별 정체 리포트] section_regions.json에 베이/중앙통로 번호가 "
              "없어 구역 분석을 건너뜁니다 (build_section_regions.py 실행 필요).")
        return None

    bay_wait = Counter()
    bay_stall_time = Counter()
    bay_stall_count = Counter()
    corridor_wait = Counter()      # {1: ..., 2: ...}
    corridor_stall_time = Counter()
    corridor_stall_count = Counter()
    merge_wait = Counter()         # {합류부 번호: ...} - 베이 집계와 별도로 "또" 센다(중복 집계,
    merge_stall_time = Counter()   # 베이 집계를 대체하는 게 아니라 "이 중 합류부에 해당하는 만큼은
    merge_stall_count = Counter()  # 특히 더 주목하라"는 별도 뷰이므로).
    other_wait = 0.0
    other_stall_time = 0.0
    other_stall_count = 0

    for node_id, cnt in wait_counts.items():
        bid = NODE_TO_BAY_ID.get(node_id)
        cid = NODE_TO_CORRIDOR_ID.get(node_id)
        if bid is not None:
            bay_wait[bid] += cnt
        elif cid is not None:
            corridor_wait[cid] += cnt
        else:
            other_wait += cnt
        for mid in NODE_TO_MERGE_ID.get(node_id, ()):
            merge_wait[mid] += cnt

    for e in stall_events:
        node_id = e["node"]
        bid = NODE_TO_BAY_ID.get(node_id)
        cid = NODE_TO_CORRIDOR_ID.get(node_id)
        if bid is not None:
            bay_stall_time[bid] += e["duration"]
            bay_stall_count[bid] += 1
        elif cid is not None:
            corridor_stall_time[cid] += e["duration"]
            corridor_stall_count[cid] += 1
        else:
            other_stall_time += e["duration"]
            other_stall_count += 1
        for mid in NODE_TO_MERGE_ID.get(node_id, ()):
            merge_stall_time[mid] += e["duration"]
            merge_stall_count[mid] += 1

    print(f"\n[구역별 정체 리포트] 베이 1~{NUM_BAYS} / 중앙통로 1~{NUM_CORRIDORS} / 합류부 1~{NUM_MERGE_ZONES}")
    # [맵 교체] 중앙통로가 2개(위/아래) 고정이 아니라 맵에서 자동 산출된
    # NUM_CORRIDORS개(신규 맵은 5개)라, 번호를 하드코딩하지 않는다.
    for cid in range(1, max(NUM_CORRIDORS, 1) + 1):
        print(f"  중앙통로{cid}  : 락대기 {corridor_wait[cid]:>8.0f}회   "
              f"정체(10s+) {corridor_stall_count[cid]:>4}건   "
              f"누적정체시간 {corridor_stall_time[cid]:>8.0f}s")
    print(f"  미분류(구역 밖) : 락대기 {other_wait:>8.0f}회   "
          f"정체(10s+) {other_stall_count:>4}건   누적정체시간 {other_stall_time:>8.0f}s")

    print(f"\n  정체 심한 베이 상위 {top_n}개 (누적정체시간 기준):")
    ranked = sorted(bay_stall_time.items(), key=lambda kv: kv[1], reverse=True)[:top_n]
    if not ranked:
        print("    (10초 이상 정체가 기록된 베이 없음)")
    for bid, t in ranked:
        print(f"    베이{bid:>2}  락대기={bay_wait.get(bid, 0):>6.0f}회  "
              f"정체건수={bay_stall_count.get(bid, 0):>3}건  누적정체시간={t:>7.0f}s")

    # [합류부] 구 맵에선 16개뿐이라 전부 출력했지만, 신규 맵은 295개라
    # 전부 찍으면 리포트가 화면을 다 잡아먹는다. 베이와 같은 기준으로
    # 상위 top_n개만 보여주고, 전체는 region_metrics.csv에서 확인한다.
    print(f"\n  정체 심한 합류부 상위 {top_n}개 (전체 {NUM_MERGE_ZONES}개 중):")
    merge_ranked = sorted(merge_stall_time.items(),
                          key=lambda kv: kv[1], reverse=True)[:top_n]
    if not merge_ranked:
        print("    (10초 이상 정체가 기록된 합류부 없음)")
    for mid, t in merge_ranked:
        bids = MERGE_ZONE_BAY_IDS.get(mid, [])
        print(f"    합류부{mid:>2} (베이{bids})  락대기={merge_wait.get(mid, 0):>6.0f}회  "
              f"정체건수={merge_stall_count.get(mid, 0):>3}건  누적정체시간={t:>7.0f}s")

    return {
        "bay_wait": dict(bay_wait),
        "bay_stall_time": dict(bay_stall_time),
        "bay_stall_count": dict(bay_stall_count),
        "corridor_wait": dict(corridor_wait),
        "corridor_stall_time": dict(corridor_stall_time),
        "corridor_stall_count": dict(corridor_stall_count),
        "merge_wait": dict(merge_wait),
        "merge_stall_time": dict(merge_stall_time),
        "merge_stall_count": dict(merge_stall_count),
        "other_wait": other_wait,
        "other_stall_time": other_stall_time,
        "other_stall_count": other_stall_count,
    }


# ----------------------------------------------------------------------
# 실험 지표 CSV 출력 (OHT별 / 구역별)
# ----------------------------------------------------------------------
# [요청 반영] 각 OHT의 가동률·병목시간과, 각 구역(베이 / 중앙통로 / 합류부)
# 에서 정체가 얼마나 발생했는지를 CSV 두 벌로 떨군다.
#
# 용어 정리(두 CSV에서 공통으로 쓰는 정의):
#   가동률(utilization)   = (작업 배정~완료 누적시간) / 전체 경과시간.
#                           "일감을 들고 있던 비율"이라 정체가 심해도 높게 나온다.
#   실동률(active_ratio)  = (실제 주행 + 설비 가공 시간) / 전체 경과시간.
#                           진짜로 뭔가 하고 있던 비율. 가동률과의 차이가 곧
#                           "배정은 됐는데 못 움직인" 손실이다.
#   병목률(stall_ratio)   = 총 정지시간 / 전체 경과시간.
#     - lock_stall  : 다음 노드가 점유돼 있어 기다린 시간(교차점/합류부 경합)
#     - follow_stall: 앞차와의 최소 차간거리에 막혀 기다린 시간
#                     ("앞 OHT에 막혀 멈춰있었던 병목시간")


def export_oht_metrics_csv(ohts, sim_duration, path="oht_metrics.csv"):
    """OHT 1대당 한 행. 가동률/실동률/병목률과 그 원인 분해, 주행거리 등."""
    rows = []
    for a in ohts:
        busy = a.total_busy
        # 종료 시점에 아직 작업 중이던 OHT는 그 진행분도 가동시간에 포함
        if a.job is not None and a.job_start_t is not None:
            busy += sim_duration - a.job_start_t
        active = a._job_travel + a._job_process + a._job_loadunload
        rows.append({
            "oht_id": a.oht_id,
            "busy_time_s": round(busy, 2),
            "utilization": round(busy / sim_duration, 4) if sim_duration else 0.0,
            "active_time_s": round(active, 2),
            "active_ratio": round(active / sim_duration, 4) if sim_duration else 0.0,
            "stall_time_s": round(a.total_stall_time, 2),
            "stall_ratio": round(a.total_stall_time / sim_duration, 4) if sim_duration else 0.0,
            "lock_stall_s": round(a.total_lock_stall_time, 2),
            "follow_stall_s": round(a.total_follow_stall_time, 2),
            "travel_time_s": round(a._job_travel, 2),
            "process_time_s": round(a._job_process, 2),
            "loadunload_time_s": round(a._job_loadunload, 2),  # H/W Spec Load/Unload 누적
            "distance": round(a.total_distance, 1),
            "avg_speed": (round(a.total_distance / a._job_travel, 1)
                           if a._job_travel > 0 else 0.0),
        })
    rows.sort(key=lambda r: r["stall_time_s"], reverse=True)
    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"[CSV] OHT별 지표 {len(rows)}행 저장: {path}")
    return rows


def export_region_metrics_csv(sim_duration, num_oht, path="region_metrics.csv"):
    """구역 1개당 한 행. 베이/중앙통로/합류부를 한 파일에
    담고 region_type 컬럼으로 구분한다. 정체시간은 _accumulate_region_stall()
    이 임계값 없이 전부 누적한 값이라 짧고 잦은 정체까지 반영된다."""
    fleet_seconds = num_oht * sim_duration if (num_oht and sim_duration) else 0

    def _row(rtype, rid, st, extra=None):
        r = {
            "region_type": rtype,
            "region_id": rid,
            "region_name": f"{rtype}{rid}",
            "stall_time_s": round(st["stall"], 2),
            "lock_stall_s": round(st["lock"], 2),
            "follow_stall_s": round(st["follow"], 2),
            "stall_events": st["events"],
            # 전체 fleet-초 대비 이 구역에서 까먹은 비율 - 구역 간 직접 비교용
            "stall_share_of_fleet": (round(st["stall"] / fleet_seconds, 6)
                                      if fleet_seconds else 0.0),
            "avg_stall_per_event_s": (round(st["stall"] / st["events"], 2)
                                       if st["events"] else 0.0),
        }
        if extra:
            r.update(extra)
        return r

    rows = []
    for bid, st in BAY_STALL_STATS.items():
        rows.append(_row("베이", bid, st))
    for cid, st in CORRIDOR_STALL_STATS.items():
        rows.append(_row("중앙통로", cid, st))
    for mid, st in MERGE_STALL_STATS.items():
        bids = MERGE_ZONE_BAY_IDS.get(mid, [])
        rows.append(_row("합류부", mid, st,
                          {"merge_of_bay": ";".join(str(b) for b in bids)}))
    rows.append(_row("미분류", 0, UNASSIGNED_STALL_STATS))

    rows.sort(key=lambda r: (r["region_type"], -r["stall_time_s"]))
    fieldnames = ["region_type", "region_id", "region_name", "stall_time_s",
                  "lock_stall_s", "follow_stall_s", "stall_events",
                  "stall_share_of_fleet", "avg_stall_per_event_s", "merge_of_bay"]
    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    print(f"[CSV] 구역별 지표 {len(rows)}행 저장: {path}")
    return rows


def print_region_ranking(top_n=10):
    """CSV와 별개로, 콘솔에서 바로 병목 위치를 확인할 수 있는 요약."""
    for label, stats, namer in (
        ("합류부", MERGE_STALL_STATS, lambda i: f"합류부{i}"),
        ("베이", BAY_STALL_STATS, lambda i: f"베이{i}"),
        ("중앙통로", CORRIDOR_STALL_STATS, lambda i: f"중앙통로{i}"),
    ):
        ranked = sorted(stats.items(), key=lambda kv: kv[1]["stall"], reverse=True)[:top_n]
        if not ranked:
            continue
        print(f"\n  [{label}] 정체시간 상위 {len(ranked)}개")
        for rid, st in ranked:
            print(f"    {namer(rid):<10} 정체={st['stall']:>9.0f}s  "
                  f"(락={st['lock']:>8.0f}s / 앞차={st['follow']:>8.0f}s)  "
                  f"발생={st['events']:>5}건")


# ----------------------------------------------------------------------
# 시계열 지표 (구간별 운반시간/처리량/정체)
# ----------------------------------------------------------------------
# run_headless_sim(..., return_raw=True)가 실어준 job_stats/stall_events
# 원본을 bin_seconds(기본 60초) 단위 구간으로 쪼개서, 구간별로
#   - throughput: 그 구간에 완료된 작업 수 (건/분, "1분당 운반개수")
#   - throughput_per_oht: 위를 대수로 나눈 값 ("1분당 운반개수/대수")
#   - avg_cycle: 그 구간에 완료된 작업들의 평균 운반시간
#   - stall_count: 그 구간에 "시작된" 정체 이벤트(STALL_THRESHOLD_SECONDS
#     이상 멈춤) 수
#   - stalled_oht_ratio: 그 구간 동안 정체 상태였던 "OHT-초"를 전체
#     (대수 x 구간길이)로 나눈 값 - "대기비중"에 해당. 정체 구간이 여러
#     bin에 걸쳐있으면 겹치는 부분만큼만 각 bin에 나눠서 반영한다.
# 을 계산해서 bin별 dict 리스트로 반환한다(그대로 CSV로 떨굴 수 있는 형태).
def compute_timeseries(job_stats, stall_events, sim_duration, num_oht,
                        bin_seconds=60.0):
    n_bins = max(1, math.ceil(sim_duration / bin_seconds))
    bins = []
    for i in range(n_bins):
        bins.append({
            "bin_start_s": i * bin_seconds,
            "bin_end_s": min((i + 1) * bin_seconds, sim_duration),
            "jobs_completed": 0,
            "cycle_sum": 0.0,
            "stall_count": 0,
            "stalled_oht_time_s": 0.0,
        })

    for j in job_stats:
        t_end = j.get("t_end")
        if t_end is None:
            continue
        idx = min(int(t_end // bin_seconds), n_bins - 1)
        bins[idx]["jobs_completed"] += 1
        bins[idx]["cycle_sum"] += j["cycle"]

    for e in stall_events:
        t0, t1 = e["t_start"], e["t_start"] + e["duration"]
        # 정체 이벤트 시작 시점 기준으로 1건 카운트(연속 구간에 걸쳐 여러 번
        # 세지 않기 위해). 시작 bin 하나에만 stall_count를 더한다.
        start_idx = min(int(t0 // bin_seconds), n_bins - 1)
        bins[start_idx]["stall_count"] += 1
        # "OHT-초" 겹침은 실제로 걸쳐있는 모든 bin에 나눠서 더한다(대기비중이
        # 정확하려면 이게 맞다 - stall_count와는 다른 목적의 집계라 따로 계산).
        first_bin = int(t0 // bin_seconds)
        last_bin = min(int(t1 // bin_seconds), n_bins - 1)
        for b in range(max(0, first_bin), last_bin + 1):
            b_start = b * bin_seconds
            b_end = min((b + 1) * bin_seconds, sim_duration)
            overlap = max(0.0, min(t1, b_end) - max(t0, b_start))
            if overlap > 0:
                bins[b]["stalled_oht_time_s"] += overlap

    rows = []
    for b in bins:
        bin_len = b["bin_end_s"] - b["bin_start_s"]
        if bin_len <= 0:
            continue
        throughput_per_min = b["jobs_completed"] / (bin_len / 60.0)
        rows.append({
            "bin_start_s": b["bin_start_s"],
            "bin_end_s": b["bin_end_s"],
            "jobs_completed": b["jobs_completed"],
            "throughput_per_min": throughput_per_min,
            "throughput_per_min_per_oht": (throughput_per_min / num_oht
                                            if num_oht else 0.0),
            "avg_cycle_s": (b["cycle_sum"] / b["jobs_completed"]
                             if b["jobs_completed"] else 0.0),
            "stall_count": b["stall_count"],
            "stalled_oht_ratio": (b["stalled_oht_time_s"] / (num_oht * bin_len)
                                   if num_oht and bin_len else 0.0),
        })
    return rows


def _print_timeseries(rows, label):
    print(f"\n[{label}] 시계열 (구간별)")
    print(f"{'구간(s)':>14} {'완료건수':>8} {'건/분':>8} {'건/분/대':>9} "
          f"{'평균운반(s)':>11} {'정체건수':>8} {'대기비중':>8}")
    for r in rows:
        print(f"{r['bin_start_s']:>6.0f}~{r['bin_end_s']:<6.0f} "
              f"{r['jobs_completed']:>8} {r['throughput_per_min']:>8.2f} "
              f"{r['throughput_per_min_per_oht']:>9.4f} "
              f"{r['avg_cycle_s']:>11.1f} {r['stall_count']:>8} "
              f"{r['stalled_oht_ratio']*100:>7.1f}%")


# ----------------------------------------------------------------------
# 정체 유/무 비교 (같은 대수, 투입률만 다르게 해서 지표 나란히 비교)
# ----------------------------------------------------------------------
# "정체가 없을 때"와 "정체가 발생할 때"를 같은 대수(num_oht)로 놓고
# spawn_interval(투입 간격)만 다르게 줘서 비교한다. spawn_interval이 작을수록
# 시간당 투입 건수가 늘어나 부하가 커진다("시간 당 요청 갯수 늘려서 하기"에
# 해당) - light_spawn_interval을 크게, heavy_spawn_interval을 작게 주면 된다.
def run_congestion_comparison(num_oht, sim_hours=1.0,
                               light_spawn_interval=None,
                               heavy_spawn_interval=None,
                               bin_seconds=60.0, seed=42,
                               out_csv="congestion_comparison_timeseries.csv"):
    sim_duration = sim_hours * 3600.0
    # 기본값: 가벼운 부하 = 현재 실측 평균 간격의 3배(느리게 투입),
    # 무거운 부하 = 실측 평균 간격의 1/3(빠르게 투입) - 둘 다 SPAWN_INTERVAL
    # 전역 기본값(실측 평균) 대비 상대적으로 잡은 값이라, 실측 자체가 이미
    # 꽤 바쁜 팹이라는 걸 감안한 것. 필요하면 인자로 직접 지정해서 쓰면 된다.
    light_spawn_interval = light_spawn_interval or SPAWN_INTERVAL * 3.0
    heavy_spawn_interval = heavy_spawn_interval or SPAWN_INTERVAL / 3.0

    scenarios = [("정체없음(저부하)", light_spawn_interval),
                 ("정체발생(고부하)", heavy_spawn_interval)]

    summary_rows = []
    csv_rows = []
    for label, spawn_interval in scenarios:
        print("\n" + "=" * 70)
        print(f"[{label}] num_oht={num_oht}  spawn_interval={spawn_interval:.4f}s "
              f"(투입률 {3600/spawn_interval:.0f}건/h)  sim_hours={sim_hours}")
        print("=" * 70)
        result = run_headless_sim(num_oht, sim_duration, spawn_interval,
                                   seed=seed, return_raw=True)
        ts = compute_timeseries(result["job_stats"], result["stall_events"],
                                 sim_duration, num_oht, bin_seconds)
        _print_timeseries(ts, label)

        wait_ratio = result["avg_wait"] / result["avg_cycle"] if result["avg_cycle"] else 0.0
        print(f"\n  [요약] 완료건수={result['n_jobs']}  "
              f"평균운반시간={result['avg_cycle']:.1f}s  "
              f"가동률(배정)={result['utilization']*100:.1f}%  "
              f"대기비중(평균)={wait_ratio*100:.1f}%  "
              f"정체이벤트(10s+)={result['n_stall_events']}건  "
              f"정체경험 OHT={result['n_stalled_oht']}/{num_oht}대"
              f"({result['stalled_oht_ratio']*100:.1f}%)  "
              f"그리드락={'예' if result['gridlock'] else '아니오'}")

        summary_rows.append({
            "시나리오": label, "spawn_interval_s": spawn_interval,
            "완료건수": result["n_jobs"], "평균운반시간_s": round(result["avg_cycle"], 1),
            "가동률_배정_pct": round(result["utilization"] * 100, 1),
            "대기비중_pct": round(wait_ratio * 100, 1),
            "정체이벤트건수": result["n_stall_events"],
            "정체경험OHT수": result["n_stalled_oht"],
            "정체경험OHT비율_pct": round(result["stalled_oht_ratio"] * 100, 1),
        })
        for r in ts:
            csv_rows.append({"시나리오": label, **r})

    print("\n\n" + "=" * 70)
    print("[정체 유/무 비교 요약]")
    print("=" * 70)
    headers = list(summary_rows[0].keys())
    col_w = [max(len(h), max(len(str(r[h])) for r in summary_rows)) + 2 for h in headers]
    print("".join(h.rjust(w) for h, w in zip(headers, col_w)))
    for r in summary_rows:
        print("".join(str(r[h]).rjust(w) for h, w in zip(headers, col_w)))

    if out_csv:
        with open(out_csv, "w", newline="", encoding="utf-8-sig") as f:
            writer = csv.DictWriter(f, fieldnames=list(csv_rows[0].keys()))
            writer.writeheader()
            writer.writerows(csv_rows)
        print(f"\n구간별 시계열 CSV 저장: {out_csv}")

    return {"summary": summary_rows, "timeseries_csv_rows": csv_rows}


def analyze_fleet_capacity(baseline_oht=MAX_OHT, sim_hours=2.0,
                            target_util_low=0.80, target_util_high=0.85,
                            spawn_interval=None, max_iter=8, seed=42,
                            baseline_wait_ratio_max=0.30, max_baseline_retries=3,
                            num_reps=3):
    """요청 반영: (1) 기준 대수로 1대당 운반시간 측정(단, baseline 자체가
    이미 혼잡하면 W가 부풀려지므로 유효성 검증 후 필요시 대수를 낮춰 재측정)
    -> (2) 가동률 80% 기준 필요대수 역산 -> (3) 그 대수로 80~85% 부하 재현
    -> (3.5) 같은 N을 seed만 바꿔 num_reps회 반복해 평균/표준편차로 신뢰도
    확인 -> (4) 병목 여부 판정(반복 평균 기준). 전 구간 from-to는 무작위로
    산출된다."""
    spawn_interval = spawn_interval or SPAWN_INTERVAL
    sim_duration = sim_hours * 3600.0
    target_util_mid = (target_util_low + target_util_high) / 2.0

    # ------------------------------------------------------------------
    # [1단계] baseline W 측정 + 유효성 검증.
    # Little's Law 역산은 "baseline W가 정체 없는 자유주행에 가까운 운반
    # 시간"이라는 전제가 깔려 있다. baseline_oht가 이미 혼잡을 유발할
    # 정도로 크면(대기비중 baseline_wait_ratio_max 초과) W 자체가 부풀려져
    # 역산된 필요대수도 같이 부풀려진다. 이런 경우 baseline_oht를 절반씩
    # 줄여가며(최대 max_baseline_retries회) 대기비중이 기준 이하로 내려오는
    # 지점을 찾는다. 끝내 못 찾으면 마지막 결과를 쓰되 경고를 남긴다.
    # ------------------------------------------------------------------
    print("=" * 70)
    print(f"[1단계] 기준 대수로 1대당 운반시간(W) 측정 (from-to 매 건 랜덤, "
          f"baseline 유효성 검증 포함)")
    print("=" * 70)
    cur_baseline = baseline_oht
    base = None
    base_valid = False
    for attempt in range(max_baseline_retries):
        print(f"  [baseline 시도{attempt + 1}] {cur_baseline}대 / {sim_hours}시간")
        base = run_headless_sim(cur_baseline, sim_duration, spawn_interval, seed=seed)
        base_wait_ratio = (base["avg_wait"] / base["avg_cycle"]
                            if base["avg_cycle"] else 0.0)
        print(f"    완료건수={base['n_jobs']}  W={base['avg_cycle']:.1f}s  "
              f"대기비중={base_wait_ratio*100:.1f}%  "
              f"가동률(배정)={base['utilization']*100:.1f}%  "
              f"가동률(실동)={base['utilization_active']*100:.1f}%")
        if base["n_jobs"] > 0 and base_wait_ratio <= baseline_wait_ratio_max:
            base_valid = True
            break
        print(f"    -> baseline 자체가 혼잡함(대기비중 기준 "
              f"{baseline_wait_ratio_max*100:.0f}% 초과 또는 완료건수 0). "
              f"baseline 대수를 줄여 재측정.")
        new_baseline = max(1, cur_baseline // 2)
        if new_baseline == cur_baseline:
            break
        cur_baseline = new_baseline
    if not base_valid:
        print(f"  [경고] baseline 대수를 {max_baseline_retries}회 줄여봤지만 "
              f"여전히 대기비중이 높다. 아래 W는 혼잡의 영향을 받아 다소 "
              f"부풀려져 있을 수 있음(참고용으로만 사용 권장).")

    W = base["avg_cycle"]
    lam = 1.0 / spawn_interval  # 초당 투입률(자재/초)
    print(f"  최종 채택 baseline={cur_baseline}대  W={W:.1f}s "
          f"(표준편차 {base['stdev_cycle']:.1f}s)")
    print(f"  분해: 대기 {base['avg_wait']:.1f}s / 주행 {base['avg_travel']:.1f}s / "
          f"가공 {base['avg_process']:.1f}s / 적재-하차(H/W Spec) {base['avg_loadunload']:.1f}s")
    print(f"  기준 가동률(배정)={base['utilization']*100:.1f}%  "
          f"기준 가동률(실동)={base['utilization_active']*100:.1f}%  "
          f"투입률 λ={lam*3600:.0f}건/h (SPAWN_INTERVAL={spawn_interval}s)")

    # ---- Little's Law: N = λ * W / U (초기 추정치일 뿐, 그대로 믿지 않음) ----
    n_req = max(1, round(lam * W / target_util_mid))
    print(f"\n[2단계] Little's Law로 목표 가동률 {target_util_mid*100:.1f}% "
          f"필요대수 초기 추정: N = λ*W/U = {lam:.4f}*{W:.1f}/{target_util_mid:.3f} "
          f"≈ {n_req}대  (OHT끼리 서로 안 막는다는 대기행렬 이론 가정이라, "
          f"실제 그래프에서는 과다추정 -> 그리드락을 유발할 수 있음. 3단계에서 검증)")

    # ------------------------------------------------------------------
    # [3단계] N 탐색.
    # baseline(cur_baseline)은 이미 그리드락이 아님을 1단계에서 확인했으므로
    # "정상 작동하는 최고 하한(lo_good_n)"으로 쓸 수 있다. Little's Law
    # 추정치(n_req)를 그대로 밀어붙이면 그래프 용량을 넘어서 그리드락(완료
    # 0건인데 가동률은 높은 상태)이 날 수 있는데, 이 경우 "늘릴수록 더
    # 막히므로" 단순 반씩 줄이기만으로는 몇 번 안에 경계를 못 찾을 수 있다
    # (실측 사례: baseline 40대 -> Little's Law 역산 1412대 -> 706 ->
    # 353대까지 반씩 줄여도 여전히 그리드락). 그리드락이 한 번이라도 나면
    # 그 순간부터는 "정상 확인된 lo_good_n"과 "그리드락 확인된 hi_bad_n"
    # 사이를 이분탐색으로 좁혀 들어간다. 그리드락을 아직 못 봤으면 기존처럼
    # 가동률 비례보정을 쓴다.
    # ------------------------------------------------------------------
    lo_good_n, lo_good_result = cur_baseline, base
    hi_bad_n = None
    result = base
    n = n_req
    hit_gridlock = False
    for it in range(max_iter):
        print(f"\n[3단계-시도{it+1}] N={n}대로 재시뮬레이션 (부하 80~85% 재현)")
        result = run_headless_sim(n, sim_duration, spawn_interval, seed=seed + it + 1)
        u = result["utilization"]
        print(f"  가동률(배정)={u*100:.1f}%  가동률(실동)={result['utilization_active']*100:.1f}%  "
              f"처리된 운반 건수={result['n_jobs']}건  "
              f"처리량={result['throughput_per_hour']:.1f}건/h")

        if result["gridlock"]:
            hit_gridlock = True
            hi_bad_n = n
            print(f"  -> 정체(gridlock) 감지: 완료 0건인데 가동률은 높음 "
                  f"(서로 길을 막아 진행이 안 되는 상태). 정상구간 {lo_good_n}대와 "
                  f"그리드락구간 {hi_bad_n}대 사이를 이분탐색으로 좁힘.")
            n_new = max(lo_good_n + 1, (lo_good_n + hi_bad_n) // 2)
            if n_new == n:
                break
            n = n_new
            continue

        # 그리드락이 아니면 "정상 작동 확인된 대수"로 갱신 (fallback용으로 보관)
        lo_good_n, lo_good_result = n, result

        if target_util_low <= u <= target_util_high:
            print(f"  -> 목표 범위(80~85%) 충족. 필요대수 {n}대로 확정.")
            break

        if hi_bad_n is not None:
            # 그리드락 상한을 이미 알고 있으니 그 안에서 목표범위를 향해 이분탐색
            if u < target_util_low:
                n_new = (n + hi_bad_n) // 2       # 그리드락 안 나는 선에서 더 늘림
            else:                                  # target_util_high보다 큰데 그리드락은 아님
                n_new = (lo_good_n + n) // 2       # 살짝 줄여서 목표범위로
        else:
            # 그리드락을 아직 못 봤으면 기존처럼 가동률 비례보정
            n_new = max(1, round(n * (u / target_util_mid)))

        if n_new == n:
            print("  -> 보정폭 0, 반복 종료.")
            break
        n = n_new
    else:
        print("  -> 반복 한도 도달, 마지막 결과로 보고.")

    if result["gridlock"]:
        # 탐색이 그리드락 상태로 끝났다면 그 N을 "필요대수"로 보고할 수
        # 없다(처리량 0건인 대수를 "필요대수"라고 하면 완전히 오해를 부름).
        # 마지막으로 정상 작동이 확인된 lo_good_n/lo_good_result로 대체한다.
        print(f"  [경고] 최종 시도({n}대)가 그리드락 상태로 종료됨. "
              f"보고용 필요대수를 마지막 정상 작동 확인 대수 {lo_good_n}대로 대체 "
              f"(그리드락 상태를 '필요대수'로 보고할 수 없으므로). 목표 가동률에는 "
              f"못 미칠 수 있으니 아래 결과를 반드시 확인할 것.")
        n, result = lo_good_n, lo_good_result

    # ------------------------------------------------------------------
    # [3.5단계] 확정된 N에 대해 seed만 바꿔 num_reps회 반복 실행.
    # 위 탐색 루프는 대수(N)를 바꿀 때마다 seed도 같이 바뀌므로, "대수 변화
    # 때문에 가동률이 달라진 것"인지 "랜덤 시드가 달라 흔들린 것"인지 단일
    # 실행만으로는 구분할 수 없다. 같은 N을 여러 시드로 반복해 평균±표준
    # 편차를 내면 이 결과가 우연이 아닌지 확인할 수 있다. 이 평균값을
    # 4단계 병목 판정의 기준으로 쓴다.
    # ------------------------------------------------------------------
    print(f"\n[3.5단계] 확정 대수 N={n}대로 seed {num_reps}개 반복 (신뢰도 검증)")
    reps = [run_headless_sim(n, sim_duration, spawn_interval, seed=seed + 500 + i)
            for i in range(num_reps)]
    for i, r in enumerate(reps):
        r_wait_ratio = r["avg_wait"] / r["avg_cycle"] if r["avg_cycle"] else 0.0
        print(f"  rep{i+1}(seed={seed+500+i}): 가동률(배정)={r['utilization']*100:.1f}%  "
              f"가동률(실동)={r['utilization_active']*100:.1f}%  "
              f"처리량={r['throughput_per_hour']:.1f}건/h  "
              f"대기비중={r_wait_ratio*100:.1f}%")

    util_vals = [r["utilization"] for r in reps]
    util_active_vals = [r["utilization_active"] for r in reps]
    throughput_vals = [r["throughput_per_hour"] for r in reps]
    wait_ratio_vals = [(r["avg_wait"] / r["avg_cycle"] if r["avg_cycle"] else 0.0)
                        for r in reps]
    replication_summary = {
        "num_reps": num_reps,
        "utilization_mean": statistics.mean(util_vals),
        "utilization_stdev": statistics.pstdev(util_vals) if num_reps > 1 else 0.0,
        "utilization_active_mean": statistics.mean(util_active_vals),
        "utilization_active_stdev": statistics.pstdev(util_active_vals) if num_reps > 1 else 0.0,
        "throughput_mean": statistics.mean(throughput_vals),
        "throughput_stdev": statistics.pstdev(throughput_vals) if num_reps > 1 else 0.0,
        "wait_ratio_mean": statistics.mean(wait_ratio_vals),
        "wait_ratio_stdev": statistics.pstdev(wait_ratio_vals) if num_reps > 1 else 0.0,
        "runs": reps,
    }
    print(f"  -> 평균 가동률(배정)={replication_summary['utilization_mean']*100:.1f}%"
          f"±{replication_summary['utilization_stdev']*100:.1f}%p  "
          f"평균 가동률(실동)={replication_summary['utilization_active_mean']*100:.1f}%"
          f"±{replication_summary['utilization_active_stdev']*100:.1f}%p  "
          f"평균처리량={replication_summary['throughput_mean']:.1f}"
          f"±{replication_summary['throughput_stdev']:.1f}건/h  "
          f"평균대기비중={replication_summary['wait_ratio_mean']*100:.1f}%"
          f"±{replication_summary['wait_ratio_stdev']*100:.1f}%p")
    in_target_range = (target_util_low <= replication_summary["utilization_mean"]
                        <= target_util_high)
    print(f"  -> 반복 평균 기준 목표범위(80~85%) 충족 여부: "
          f"{'충족' if in_target_range else '미충족(단일 실행 결과와 다를 수 있음)'}")

    # ---- 병목 판정 (반복 평균 기준) ----
    wait_ratio = replication_summary["wait_ratio_mean"]
    u_mean = replication_summary["utilization_mean"]
    print(f"\n[4단계] 병목 확인: 반복 평균 대기시간 비중 = {wait_ratio*100:.1f}%")
    bottleneck = (hit_gridlock or wait_ratio > 0.30 or u_mean > target_util_high + 0.05)
    if hit_gridlock:
        print("  -> 병목 있음: 탐색 과정에서 정체(gridlock)가 관측됨 "
              "(해당 대수 구간에서는 노드락 경합이 지배적 제약).")
    if bottleneck:
        print("     혼잡 노드 상위(마지막 반복 기준 락 대기 발생 횟수):")
        for node_id, cnt in reps[-1]["top_congested"]:
            print(f"       node {node_id}: {cnt}회")
    else:
        print("  -> 뚜렷한 병목 없음 (대기시간 비중 정상 범위).")

    print("\n" + "=" * 70)
    print("[요약]")
    print(f"  기준 1대당 운반시간(W)       : {W:.1f}s"
          + ("" if base_valid else "  [주의: baseline 혼잡으로 다소 부풀려짐]"))
    print(f"  목표가동률(80~85%) 필요대수  : {n}대")
    print(f"  해당 대수 반복평균 가동률(배정) : {u_mean*100:.1f}%"
          f"±{replication_summary['utilization_stdev']*100:.1f}%p")
    print(f"  해당 대수 반복평균 가동률(실동) : "
          f"{replication_summary['utilization_active_mean']*100:.1f}%"
          f"±{replication_summary['utilization_active_stdev']*100:.1f}%p")
    print(f"  해당 대수 반복평균 처리량      : "
          f"{replication_summary['throughput_mean']:.1f}"
          f"±{replication_summary['throughput_stdev']:.1f}건/h")
    print(f"  병목 여부                    : {'있음' if bottleneck else '없음'}")
    print("=" * 70)

    return {"W": W, "n_required": n, "baseline": base, "final": result,
            "bottleneck": bottleneck, "replication_summary": replication_summary,
            "baseline_valid": base_valid, "baseline_oht_used": cur_baseline}


# ----------------------------------------------------------------------
# 명시적 OHT 대수 스윕 (Little's Law 역산/이분탐색 없이, 지정한 대수들을
# 그대로 하나씩 돌려서 검증하는 용도).
# analyze_fleet_capacity()의 자동 역산이 이상한 결과(예: 그리드락 경계를
# 잘못 잡음)를 낼 수 있으므로, "이 대수들에서는 실제로 어떻게 되는지"를
# 직접 눈으로 확인하고 싶을 때 이 함수를 쓴다. 시나리오 비교에서 특정
# 조합에 대해 대수 후보를 명시적으로 넣고 싶을 때도 이 함수를 쓰면 된다.
# ----------------------------------------------------------------------
def sweep_fleet_sizes(oht_counts, sim_hours=2.0, spawn_interval=None,
                       seed=42, num_reps=1):
    """oht_counts(리스트)에 있는 대수들을 각각 num_reps회(시드만 다르게)
    돌려서 대수별 가동률(배정/실동)·처리량·대기비중·그리드락 여부를
    표로 반환한다. PROCESS_TIME은 모듈 전역값을 그대로 사용하므로
    호출 전에 필요시 바꿔두면 된다."""
    spawn_interval = spawn_interval or SPAWN_INTERVAL
    sim_duration = sim_hours * 3600.0

    print("=" * 70)
    print(f"[OHT 대수 직접 스윕] {oht_counts} / {sim_hours}시간 / "
          f"seed당 {num_reps}회 반복")
    print("=" * 70)

    rows = []
    for n in oht_counts:
        reps = [run_headless_sim(n, sim_duration, spawn_interval,
                                  seed=seed + 10_000 + n * 7 + i)
                for i in range(num_reps)]
        util_vals = [r["utilization"] for r in reps]
        util_active_vals = [r["utilization_active"] for r in reps]
        throughput_vals = [r["throughput_per_hour"] for r in reps]
        wait_ratio_vals = [(r["avg_wait"] / r["avg_cycle"] if r["avg_cycle"] else 0.0)
                            for r in reps]
        gridlock_any = any(r["gridlock"] for r in reps)
        row = {
            "num_oht": n,
            "utilization_mean": statistics.mean(util_vals),
            "utilization_stdev": statistics.pstdev(util_vals) if num_reps > 1 else 0.0,
            "utilization_active_mean": statistics.mean(util_active_vals),
            "throughput_mean": statistics.mean(throughput_vals),
            "throughput_stdev": statistics.pstdev(throughput_vals) if num_reps > 1 else 0.0,
            "wait_ratio_mean": statistics.mean(wait_ratio_vals),
            "gridlock": gridlock_any,
            "runs": reps,
        }
        rows.append(row)
        print(f"  N={n:>5}대: 가동률(배정)={row['utilization_mean']*100:5.1f}%  "
              f"가동률(실동)={row['utilization_active_mean']*100:5.1f}%  "
              f"처리량={row['throughput_mean']:6.1f}건/h  "
              f"대기비중={row['wait_ratio_mean']*100:5.1f}%  "
              f"{'[그리드락]' if gridlock_any else ''}")

    print("=" * 70)
    return rows


# ----------------------------------------------------------------------
# 줌/팬 뷰포트 (main()/replay_main() 공용)
# ----------------------------------------------------------------------
# 마우스 휠 확대/축소(커서 위치 고정), 오른쪽 버튼 드래그 이동, 키보드
# +/- 확대/축소, R 리셋을 한 곳에서 처리한다. 생성형 시뮬레이션 화면과
# 실측 재생 화면이 완전히 같은 조작감을 갖게 하려고 공용 클래스로 뺐다
# (따로 두 벌 만들면 한쪽만 고치고 다른 쪽을 깜빡하는 사고가 나기 쉬움 -
# 실제로 이전 버전에서 재생기에만 줌을 넣었다가 존재하지 않는 상수를
# 참조해서 에러가 났던 적이 있다).
class Camera:
    """현재 줌/팬 상태 1개를 렌더링에 필요한 형태로 스냅샷한 값.
    (left, top): ref-pixel 공간(sx()/sy_pil() 기준, SCALE로 구운 "도면 전체"
                 좌표계)에서 지금 보이는 영역의 좌상단.
    scale:       ref-pixel 1개가 실제 창에 몇 픽셀로 그려질지 배율
                 (= fit_scale * zoom). 예전엔 이 배율만큼 "이미 그려둔
                 저해상도 그림"을 pygame.transform.scale로 늘렸지만,
                 지금은 render 함수들이 직접 이 scale로 좌표를 계산해서
                 선/원/삼각형을 다시 그린다 -> crop 후 확대(rasterize
                 후 늘리기) 단계가 아예 없어져서, 아무리 확대해도 항상
                 벡터를 그 배율로 새로 그린 결과이므로 계단현상이 생기지
                 않는다.
    w, h:        이 프레임에서 "지도" 부분이 차지하는 창 픽셀 크기
                 (통계/범례 패널은 별도, 줌의 영향을 받지 않는다)."""
    __slots__ = ("left", "top", "scale", "w", "h")

    def __init__(self, left, top, scale, w, h):
        self.left, self.top, self.scale, self.w, self.h = left, top, scale, w, h

    def pt(self, rx, ry):
        """ref-pixel 좌표 -> 현재 창 픽셀 좌표."""
        return ((rx - self.left) * self.scale, (ry - self.top) * self.scale)

    def key(self):
        """정적 레이어 캐시 비교용 키(부동소수 미세 오차는 무시)."""
        return (round(self.left, 2), round(self.top, 2), round(self.scale, 4), self.w, self.h)


class Viewport:
    """지도 표시 영역(줌/팬 대상)과 그 아래 고정 크기 통계/범례 패널을
    분리해서 관리한다. view_cx/cy/zoom으로 camera()를 만들면
    render_frame()/render_replay_frame()이 "보이는 영역만, 창 해상도
    그대로" 다시 그린다(예전처럼 저해상도로 구운 그림 전체를 잘라 확대하는
    방식이 아님).
    [축소 지원] zoom=1.0은 "지도 전체가 창에 딱 맞게 보이는 배율"이다.
    zoom>1이면 확대(지도 일부만 크게), zoom<1이면 축소(지도 전체가 더
    작게, 남는 공간은 배경색 여백)다. visible_size()가 더 이상 img_w/
    img_h로 clamp하지 않으므로, zoom<1일 때는 vis_w/vis_h가 지도 크기를
    넘어서고 그만큼 여백이 생긴다."""

    def __init__(self, img_w, img_h, stats_h, screen_w, screen_h):
        self.img_w = img_w          # 지도 원본 크기(ref-pixel, SCALE 기준)
        self.img_h = img_h
        self.stats_h = stats_h      # 통계/범례 패널 높이(고정, 줌 영향 없음)

        avail_h = max(1, screen_h - stats_h)
        self.fit_scale = min(
            MAX_WINDOW_SCALE,
            (screen_w * SCREEN_FIT_MARGIN) / img_w,
            (avail_h * SCREEN_FIT_MARGIN) / img_h,
        )
        self.map_win_w = max(1, int(img_w * self.fit_scale))
        self.map_win_h = max(1, int(img_h * self.fit_scale))
        self.zoom = 1.0
        self.view_cx = img_w / 2.0
        self.view_cy = img_h / 2.0
        self.dragging = False
        self._drag_start_mouse = (0, 0)
        self._drag_start_center = (self.view_cx, self.view_cy)

    @property
    def win_w(self):
        """창 전체 폭(=지도 폭, 지도와 패널이 같은 폭을 씀)."""
        return self.map_win_w

    @property
    def win_h(self):
        """창 전체 높이 = 지도 높이 + 통계/범례 패널 높이."""
        return self.map_win_h + self.stats_h

    def reset(self):
        self.zoom = 1.0
        self.view_cx = self.img_w / 2.0
        self.view_cy = self.img_h / 2.0

    def visible_size(self):
        """[축소 지원] 예전엔 min(..., img_w/img_h)로 clamp해서 zoom<1일
        때도 vis_w/vis_h가 지도 크기를 못 넘었다(=축소가 무효했던 근본
        원인). 이제는 clamp 없이 배율 그대로 계산해서, zoom<1이면
        vis_w/vis_h가 지도보다 커지고(=지도가 더 작게 보임), zoom>1이면
        더 작아진다(=지도 일부가 크게 보임). 지도 밖 여백은
        crop_box()/build_static_layer()가 배경색으로 채운다."""
        vis_w = self.map_win_w / (self.fit_scale * self.zoom)
        vis_h = self.map_win_h / (self.fit_scale * self.zoom)
        return vis_w, vis_h

    def crop_box(self):
        vis_w, vis_h = self.visible_size()
        # [축소 지원] vis_w/vis_h가 지도 크기보다 작을 때(확대 중이거나
        # zoom=1)만 "지도 밖으로 못 나가게" 클램프한다. vis_w/vis_h가
        # 지도보다 클 때(축소 중)는 클램프할 범위 자체가 역전(img_w-vis_w
        # <0)되므로, 그냥 view_cx/cy를 중심으로 둬서 지도가 창 가운데
        # 작게 보이고 남는 공간은 여백으로 남긴다.
        if vis_w <= self.img_w:
            left = max(0.0, min(self.view_cx - vis_w / 2, self.img_w - vis_w))
        else:
            left = self.view_cx - vis_w / 2
        if vis_h <= self.img_h:
            top = max(0.0, min(self.view_cy - vis_h / 2, self.img_h - vis_h))
        else:
            top = self.view_cy - vis_h / 2
        return left, top, vis_w, vis_h

    def camera(self):
        """지금 줌/팬 상태를 렌더 함수들이 바로 쓸 수 있는 Camera로 변환."""
        left, top, vis_w, vis_h = self.crop_box()
        scale = self.map_win_w / vis_w if vis_w > 0 else 1.0
        return Camera(left, top, scale, self.map_win_w, self.map_win_h)

    def _zoom_at(self, factor, mx, my):
        my = max(0, min(my, self.map_win_h))  # 통계 패널 위에서 휠 굴려도 안전하게 클램프
        left, top, vis_w, vis_h = self.crop_box()
        img_x = left + (mx / self.map_win_w) * vis_w
        img_y = top + (my / self.map_win_h) * vis_h
        self.zoom = max(ZOOM_MIN, min(ZOOM_MAX, self.zoom * factor))
        new_vis_w, new_vis_h = self.visible_size()
        self.view_cx = img_x - (mx / self.map_win_w - 0.5) * new_vis_w
        self.view_cy = img_y - (my / self.map_win_h - 0.5) * new_vis_h

    def handle_event(self, event, screen_setter):
        """이벤트 1개 처리. screen_setter(w, h)는 VIDEORESIZE일 때 pygame
        디스플레이를 새 크기로 다시 만드는 콜백(main()/replay_main()이
        각자의 screen 변수를 갖고 있어서 여기서 직접 못 바꾸므로 위임)."""
        if event.type == pygame.KEYDOWN and event.key == pygame.K_r:
            self.reset()
        elif event.type == pygame.KEYDOWN and event.key in (
                pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
            self._zoom_at(ZOOM_STEP, self.map_win_w / 2, self.map_win_h / 2)
        elif event.type == pygame.KEYDOWN and event.key in (
                pygame.K_MINUS, pygame.K_KP_MINUS):
            self._zoom_at(1.0 / ZOOM_STEP, self.map_win_w / 2, self.map_win_h / 2)
        elif event.type == pygame.VIDEORESIZE:
            self.map_win_w = max(1, event.w)
            self.map_win_h = max(1, event.h - self.stats_h)
            screen_setter(event.w, event.h)
        elif event.type == pygame.MOUSEWHEEL:
            mx, my = pygame.mouse.get_pos()
            self._zoom_at(ZOOM_STEP ** event.y, mx, my)
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 3:
            self.dragging = True
            self._drag_start_mouse = event.pos
            self._drag_start_center = (self.view_cx, self.view_cy)
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 3:
            self.dragging = False
        elif event.type == pygame.MOUSEMOTION and self.dragging:
            dx = event.pos[0] - self._drag_start_mouse[0]
            dy = event.pos[1] - self._drag_start_mouse[1]
            self.view_cx = self._drag_start_center[0] - dx / (self.fit_scale * self.zoom)
            self.view_cy = self._drag_start_center[1] - dy / (self.fit_scale * self.zoom)


# ----------------------------------------------------------------------
# 단일 이미지 합성 렌더링 (2500대 대응 핵심)
# ----------------------------------------------------------------------
# [가장 확실한 구조] salabim의 tkinter 기반 애니메이션은 아예 쓰지 않는다.
# render_frame(t, vp)가 매 프레임 vp(줌/팬 상태)에 맞춰 정적 레이어
# 캐시(get_static_layer)를 재사용/재구성하고, 그 위에 활성 EQ 원 + 모든
# OHT 삼각형 + 통계 텍스트를 PIL로 직접 "창 해상도 그대로" 그려서 이미지
# "한 장"을 반환한다. main()의 pygame 루프는 이 이미지를 그대로 blit하면
# 되고(별도 crop/스케일 단계 없음), 화면 표시 비용이 "오브젝트 개수"가
# 아니라 "이미지 픽셀 수"에만 비례하므로 OHT/EQ가 늘어나도 구조적으로
# 안정적이다(tkinter Canvas처럼 아이템 수에 비례해 느려지는 근본적 한계가
# 없음). 확대해도 항상 이 배율로 다시 그린 결과라 해상도가 깨지지 않는다.
def draw_port_labels(draw, cam):
    """버퍼/EQ 포트 번호(equipment.json의 port_id 그대로)를 현재 카메라
    기준으로 그린다. build_static_layer() 안에서만 호출되므로(정적
    레이어 캐시에 포함됨) render_frame()처럼 매 애니메이션 프레임마다
    다시 불릴 일이 없다 - EQ 3,944개 + 버퍼 15,831개를 순회하는 비용은 카메라가
    바뀔 때(줌/팬 중)에만 든다."""
    eq_r = max(1.0, EQ_R * cam.scale)
    for e in EQ_PORTS:
        px, py = cam.pt(sx(e["x"]), sy_pil(e["y"]))
        if -20 <= px <= cam.w + 20 and -20 <= py <= cam.h + 20:
            draw.text((px, py - eq_r - 2), e["port_id"], font=FONT_PORT_ID,
                       fill="black", stroke_width=1, stroke_fill="white", anchor="mb")
    stk_r = max(1.0, STK_R * cam.scale)
    for e in BUFFER_PORTS:
        px, py = cam.pt(sx(e["x"]), sy_pil(e["y"]))
        if -20 <= px <= cam.w + 20 and -20 <= py <= cam.h + 20:
            draw.text((px, py - stk_r - 2), e["port_id"], font=FONT_PORT_ID,
                       fill="black", stroke_width=1, stroke_fill="white", anchor="mb")


def render_frame(t, vp):
    """vp(현재 줌/팬 상태)를 받아 "창 크기 그대로" 완성된 프레임 1장을
    반환한다. 예전처럼 고정 저해상도 이미지를 만들어 나중에 crop/확대하는
    단계가 없으므로, 반환된 이미지를 그대로 blit하면 어떤 배율로 확대해
    있든 항상 선명하다."""
    cam = vp.camera()

    # [정적 레이어] 레일+레일정체+EQ idle+STK. 카메라가 안 바뀌고
    # RAIL_CONGESTION_REFRESH_SEC(기본 1초)도 안 지났으면 캐시를 그대로
    # 재사용한다(매 프레임 7,785개 선분을 다시 그리지 않음).
    static_img = get_static_layer(
        cam, t,
        lambda: [oht.node for oht in OHT_LIST],
        lambda: [nid for nid, st in EQ_STATUS.items() if st == "empty"],
    )

    frame = Image.new("RGB", (vp.win_w, vp.win_h), BG_COLOR)
    frame.paste(static_img, (0, 0))
    draw = ImageDraw.Draw(frame)

    # [요청 반영] STK/설비(Machine)를 레일 오버레이 위에 그려둔다.
    #   SHOW_EQ_IDLE=True  -> Idle 상태는 정적 레이어에 이미 반투명으로
    #                         깔려있으니, 여기서는 실제로 뭔가 진행 중인
    #                         설비(예약/가공/완료 - EQ_ACTIVE 집합, 개수가
    #                         훨씬 적음)만 불투명하게 그려 눈에 띄게 한다.
    #   SHOW_EQ_IDLE=False -> Idle 설비는 정적 레이어에도 안 그려졌으므로,
    #                         여기서도 EQ_ACTIVE만 그린다.
    eq_r = max(1.0, EQ_R * cam.scale)
    for node_id in EQ_ACTIVE:
        x, y = NODE_XY[node_id]
        px, py = cam.pt(sx(x), sy_pil(y))
        status = EQ_STATUS.get(node_id, "empty")
        draw.ellipse((px - eq_r, py - eq_r, px + eq_r, py + eq_r),
                     fill=MACHINE_STATUS_COLOR[status])
    draw_buffer_ports(draw, cam)
    if SHOW_RING_OVERLAY:
        draw_ring_overlay(draw, cam)   # [고리 진입 제어] G 키

    # [성능] EQ/STK 포트 번호는 이제 build_static_layer() 안에서 캐시와
    # 함께 그려진다(위 static_img에 이미 포함돼 있음) - 여기서 매 프레임
    # 다시 그리지 않는다.

    # [요청 반영] OHT를 화면 맨 위(STK/설비보다도 위)에 오도록 마지막에
    # 그린다. 회전(cos/sin)은 heading이 바뀔 때 _update_heading()에서
    # 이미 계산해 self._tri_local에 캐싱해뒀으므로 여기서는 현재 위치로
    # 평행이동만 하면 된다(삼각함수 재계산 없음). 삼각형 로컬 오프셋도
    # cam.scale만큼 같이 키워서, 확대할수록 OHT도 실제 크기 비율대로
    # 커 보이게 한다(줌해도 늘 같은 픽셀 크기인 예전 방식과 다름).
    s = cam.scale
    show_ids = SHOW_OHT_ID and s >= OHT_ID_LABEL_MIN_SCALE
    label_budget = OHT_ID_LABEL_MAX_COUNT if show_ids else 0
    for oht in OHT_LIST:
        cx, cy = oht._interp_xy(t)  # ref-pixel 좌표(y-up), sx/sy_screen 기준
        ox, oy = cam.pt(cx, IMG_H - cy)  # sy_pil과 동일한 y반전 적용 후 카메라 변환
        poly = [(ox + lx * s, oy + ly * s) for lx, ly in oht._tri_local]
        if OHT_DRAW_OUTLINE:
            draw.polygon(poly, fill=oht.color, outline="black")
        else:
            draw.polygon(poly, fill=oht.color)
        if label_budget > 0 and 0 <= ox <= vp.win_w and 0 <= oy <= vp.map_win_h:
            # [OHT 번호] 흰 글씨 + 검은 테두리(stroke)로, 트랙(초록/회색)이든
            # OHT 색(보라/파랑)이든 배경과 상관없이 항상 잘 보이게 한다.
            # 폰트 크기는 고정(FONT_OHT_ID)이라 줌해도 늘 같은 픽셀 크기로
            # 읽기 편하고, 화면 밖이거나 OHT_ID_LABEL_MAX_COUNT를 넘긴
            # 것들은 그리지 않아(위 주석 참고) 프레임 비용을 억제한다.
            draw.text((ox, oy), str(oht.oht_id), font=FONT_OHT_ID,
                       fill="white", stroke_width=1, stroke_fill="black",
                       anchor="mm")
            label_budget -= 1

    # 통계 텍스트 + 범례(줌 영향 없는 고정 크기 패널).
    if SHOW_STATS_TEXT:
        stats = (f"Finished {manager.finished}   WIP {manager.wip}/{MAX_OHT}   "
                 f"Injected {manager.injected}")
        draw_stats_strip(frame, vp, stats)

    return frame


# ----------------------------------------------------------------------
# 환경 구성 및 실행
# ----------------------------------------------------------------------
# [가장 확실한 구조 변경] salabim의 애니메이션 기능(tkinter 기반)은 아예
# 켜지 않는다. salabim Environment는 오직 이벤트 스케줄링(DES 엔진)으로만
# 쓰고, 화면 표시는 pygame(SDL 기반)이 전담한다.
#   1. env.run(till=...)으로 시뮬레이션 시간을 한 프레임 분량만큼 진행
#   2. render_frame(t)로 그 시점의 전체 장면을 PIL 이미지 한 장으로 합성
#   3. 그 이미지를 pygame Surface로 바꿔 화면에 blit
# tkinter Canvas는 내부적으로 아이템 개수에 비례해 느려지는 구조적 한계가
# 있지만, 이 방식은 "이미지 한 장을 화면에 그리는" 비용만 들기 때문에
# OHT 수, EQ 수, 오버레이 요소가 앞으로 더 늘어나도 구조 자체는 안정적이다.
def toggle_show_port_id():
    """L 키로 EQ/STK 포트 번호 표시를 켜고 끈다. get_static_layer()가
    이 값을 캐시 키에 포함시키므로(위 참고), 여기서 값만 뒤집어주면
    다음 프레임에 자동으로 다시 그려진다(별도 강제 리빌드 호출 불필요)."""
    global SHOW_PORT_ID
    SHOW_PORT_ID = not SHOW_PORT_ID
    print(f"[표시] EQ/STK 라벨: {'ON' if SHOW_PORT_ID else 'OFF'}")


def toggle_show_region_tint():
    """T 키로 베이(파란)/중앙통로(초록) 구역 색 틴트를 켜고 끈다.
    section_regions.json이 없어 틴트 세그먼트가 비어있으면 draw_region_tint()가
    아무것도 안 그리므로, 켜도 화면에 변화가 없을 수 있다는 안내를 같이
    띄운다."""
    global SHOW_REGION_TINT
    SHOW_REGION_TINT = not SHOW_REGION_TINT
    _has_data = bool(BAY_TINT_SEGMENTS or CENTER_TINT_SEGMENTS)
    print(f"[표시] 베이/중앙통로 색 틴트: {'ON' if SHOW_REGION_TINT else 'OFF'}"
          + ("" if _has_data else "  [주의: section_regions.json 없음 - 변화 없음]"))


def toggle_show_oht_id():
    """N 키로 OHT 번호 표시를 켜고 끈다. SHOW_OHT_ID는 render_frame()/
    render_replay_frame()이 매 프레임 직접 읽는 값이라(OHT 위치처럼 매
    프레임 바뀌는 요소라 애초에 정적 레이어 캐시에 들어가지 않음), L/T
    토글과 달리 캐시 무효화를 신경 쓸 필요 없이 값만 뒤집으면 바로 다음
    프레임에 반영된다."""
    global SHOW_OHT_ID
    SHOW_OHT_ID = not SHOW_OHT_ID
    print(f"[표시] OHT 번호: {'ON' if SHOW_OHT_ID else 'OFF'}")


def toggle_show_ring_overlay():
    """G 키로 고리 점유 상태 표시를 켜고 끈다. 매 프레임 그리는 동적
    요소라 캐시 무효화 없이 값만 뒤집으면 된다."""
    global SHOW_RING_OVERLAY
    SHOW_RING_OVERLAY = not SHOW_RING_OVERLAY
    print(f"[표시] 베이 고리: {'ON' if SHOW_RING_OVERLAY else 'OFF'} "
          f"({len(RINGS)}개, 파랑=여유 / 주황=용량 {RING_OVERLAY_WARN:.0%} 이상 / 빨강=만석)")


def draw_ring_overlay(draw, cam):
    """점유가 RING_OVERLAY_WARN 이상인 고리의 링크를 굵게 칠한다(만석이면
    빨강). 한산한 고리는 그리지 않아 레일 혼잡색과 겹치지 않게 한다."""
    w = max(2, round(3 * cam.scale))
    for ri, r in enumerate(RINGS):
        occ = _ring_occupancy(ri)
        if occ >= r["cap"]:
            color = (220, 30, 30)          # 만석 - 진입 차단 중
        elif occ >= RING_OVERLAY_WARN * r["cap"]:
            color = (245, 150, 20)         # 거의 참
        else:
            color = (70, 120, 230)         # 관리 대상 베이 고리(여유)
        for li in r["links"]:
            xa, ya = NODE_XY[LINK_A[li]]
            xb, yb = NODE_XY[LINK_B[li]]
            draw.line([cam.pt(sx(xa), sy_pil(ya)), cam.pt(sx(xb), sy_pil(yb))],
                      fill=color, width=w)


def toggle_show_node_dots():
    """D 키로 모든 주행노드를 점으로 찍어 보여준다/숨긴다. 나가는 링크가
    JUNCTION_DOT_MIN_BRANCHES개 이상인 분기점은 보라, 합류부 관련 노드는
    허가권(주황)/합류대기(빨강)/우선진입(파랑)으로 구분된다
    (agv_fab_sim_mini.py의 SHOW_NODE_DOTS를 이 파일 구조로 옮긴 것).
    get_static_layer()가 이 값을 캐시 키에 포함시키므로 값만 뒤집으면 된다."""
    global SHOW_NODE_DOTS
    SHOW_NODE_DOTS = not SHOW_NODE_DOTS
    n_junction = sum(1 for n in NODE_LIST
                     if len(OUT_LINKS.get(n, ())) >= JUNCTION_DOT_MIN_BRANCHES)
    print(f"[표시] 노드 점: {'ON' if SHOW_NODE_DOTS else 'OFF'} "
          f"(합류지점 {len(MERGE_POINT_NODES)}개 / 합류대기 "
          f"{len(MERGE_WAIT_NODES)}개 / 우선진입 {len(MERGE_PRIORITY_NODES)}개 / "
          f"분기점 {JUNCTION_DOT_MIN_BRANCHES}갈래 이상 {n_junction}개)")


def adjust_junction_dot_threshold(delta):
    """[ / ] 키로 분기점 판정 기준(N갈래 이상)을 1씩 조절한다. 최소 2
    (out-degree>=2, 즉 방법론 3-1번이 말하는 '분기점' 전체)로 바닥을 둔다.
    SHOW_NODE_DOTS가 꺼져 있어도 다음에 켤 때 반영되도록 값만 바꾼다."""
    global JUNCTION_DOT_MIN_BRANCHES
    JUNCTION_DOT_MIN_BRANCHES = max(2, JUNCTION_DOT_MIN_BRANCHES + delta)
    n_junction = sum(1 for n in NODE_LIST
                     if len(OUT_LINKS.get(n, ())) >= JUNCTION_DOT_MIN_BRANCHES)
    print(f"[표시] 분기점 기준: {JUNCTION_DOT_MIN_BRANCHES}갈래 이상 "
          f"({n_junction}개){'  (D로 켜야 보임)' if not SHOW_NODE_DOTS else ''}")



def main(num_oht=None, spawn_interval=None):
    """화면(pygame)을 띄우고 OHT가 실제로 움직이는 생성형 시뮬레이션을 실행.
    num_oht / spawn_interval을 주면 모듈 기본값(MAX_OHT / SPAWN_INTERVAL)
    대신 그 값으로 돈다(CLI의 --oht / --spawn-interval)."""
    global manager, dispatcher, MAX_OHT, SPAWN_INTERVAL

    if num_oht is not None:
        MAX_OHT = num_oht
    if spawn_interval is not None:
        SPAWN_INTERVAL = spawn_interval

    sim.yieldless(False)
    env = sim.Environment(trace=False)  # animate 관련 설정은 아예 건드리지 않음

    # ---- OHT 풀 생성 (MAX_OHT개, 고정된 시작위치에서 대기 시작) ----
    park_nodes = get_park_nodes(MAX_OHT)   # 고정 시작위치(0번 요청사항)
    print(f"[생성형 시뮬레이션] OHT {MAX_OHT}대, 작업 투입 간격 {SPAWN_INTERVAL:.2f}s "
          f"(= 시간당 {3600 / SPAWN_INTERVAL:.0f}건). 작업을 못 받은 OHT는 "
          f"트랙을 배회한다.")

    # [시간 기반 라우팅] 경로탐색이 pair-state A*(질의당 평균 2.3ms)로
    # 바뀌면서 시작 전 dijkstra warmup이 사라졌다. 사전감속 전파
    # (JUNCTION_SPEED)는 모듈 로드 시 한 번, 0.02초면 끝난다.
    ohts = [OHT(park_node=n) for n in park_nodes]

    manager = Manager()
    dispatcher = Dispatcher()   # 10초 주기 스코어 배정
    StallWatchdog()             # 트리거(2) 분기점 정체 감시
    CongestionMonitor()         # 혼잡도 기반 동적 가중치
    IdleDistributor()           # 미배정 OHT를 Zone(베이) N등분 지점으로 분산

    # ---- pygame 창 초기화 (화면 해상도에 맞춰 자동으로 창 크기 결정) ----
    pygame.init()
    pygame.display.set_caption(
        "OHT Fab Simulation  휠/+-: 확대/축소, 우클릭 드래그: 이동, "
        "L: EQ/STK 라벨 on/off, T: 베이/중앙통로 색 on/off, "
        "N: OHT 번호 on/off, R: 리셋")

    display_info = pygame.display.Info()
    screen_w = display_info.current_w or 1920  # 해상도를 못 가져오면 임시 기본값
    screen_h = display_info.current_h or 1080

    vp = Viewport(IMG_W, IMG_H, STATS_H, screen_w, screen_h)
    screen = pygame.display.set_mode((vp.win_w, vp.win_h), pygame.RESIZABLE)
    clock = pygame.time.Clock()

    def set_screen(w, h):
        nonlocal screen
        screen = pygame.display.set_mode((w, h), pygame.RESIZABLE)

    frame_sim_dt = SIM_SPEED_FACTOR / TARGET_FPS  # 프레임당 진행시킬 시뮬레이션 시간

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_l:
                toggle_show_port_id()
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_t:
                toggle_show_region_tint()
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_n:
                toggle_show_oht_id()
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_d:
                toggle_show_node_dots()
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_g:
                toggle_show_ring_overlay()
            elif event.type == pygame.KEYDOWN and event.key in (
                    pygame.K_LEFTBRACKET, pygame.K_RIGHTBRACKET):
                adjust_junction_dot_threshold(
                    -1 if event.key == pygame.K_LEFTBRACKET else 1)
            else:
                vp.handle_event(event, set_screen)
        if not running:
            break

        # 1) 시뮬레이션을 다음 렌더 시점까지 진행 (실시간 동기화 없이
        #    계산 가능한 만큼 최대 속도로 이벤트를 처리한다)
        env.run(till=env.now() + frame_sim_dt)

        # 2) 현재 줌/팬 상태(vp) 기준으로 창 해상도 그대로 프레임 합성
        #    (예전의 "crop 후 확대" 단계가 사라져서 그대로 blit하면 된다)
        frame = render_frame(env.now(), vp)
        surf = pygame.image.fromstring(frame.tobytes(), frame.size, frame.mode)
        screen.blit(surf, (0, 0))
        pygame.display.flip()

        clock.tick(TARGET_FPS)

    pygame.quit()


# ----------------------------------------------------------------------
# 실측 재생(replay) 모드
# ----------------------------------------------------------------------
# 아래는 시뮬레이션이 아니라 "재생기"다. Dijkstra 라우팅도, 가감속
# 물리모델도, 노드락 충돌회피도 전혀 안 쓴다 - extract_oht_replay.py가
# OHT_log_only.log에서 뽑아둔 실제 위치 변화(REPLAY_JSON)를 시간 순서
# 그대로 화면에 옮겨줄 뿐이다. 그래서 대수를 바꿔서 실험하는 건 이 모드로는
# 불가능하다(로그에 정확히 그 대수가 정확히 그렇게 움직인 기록만 있음).
# "OHT가 300대면 어떨까"는 위 main()/analyze_fleet_capacity() 쪽 질문이고,
# "그날 실제로 뭐가 있었나"가 이 replay_main() 쪽 질문이다.
def oht_position(traj_entry, t):
    """한 OHT의 trajectory에서 시각 t의 (x, y, carrier, heading_deg).
    기록은 노드가 바뀔 때만 찍혀있으므로, 두 기록 사이는 등속 이동으로
    선형보간한다(로그엔 노드 통과 시각만 있고 그 사이 위치는 없어서,
    구간 내 등속 가정이 최선의 근사)."""
    times = traj_entry["t"]
    nodes = traj_entry["node"]
    carrier = traj_entry["carrier"]
    n = len(times)
    if n == 0:
        return None

    if t <= times[0]:
        node0 = node1 = nodes[0]
        car = carrier[0]
        frac = 0.0
    elif t >= times[-1]:
        node0 = node1 = nodes[-1]
        car = carrier[-1]
        frac = 0.0
    else:
        idx = bisect.bisect_right(times, t) - 1
        node0, node1 = nodes[idx], nodes[idx + 1]
        car = carrier[idx]
        t0, t1 = times[idx], times[idx + 1]
        frac = (t - t0) / (t1 - t0) if t1 > t0 else 0.0

    x0, y0 = NODE_XY[node0]
    x1, y1 = NODE_XY[node1]
    x = x0 + (x1 - x0) * frac
    y = y0 + (y1 - y0) * frac
    heading = (math.degrees(math.atan2(y1 - y0, x1 - x0)) - 90
               if (x1, y1) != (x0, y0) else 0.0)
    return x, y, car, heading


def eq_replay_status(node_events, node_id, t):
    """extract_eq_status.py가 만든 노드별 (t, status) 타임라인에서 시각 t의
    상태를 구한다. 기록이 없거나 t가 첫 이벤트 이전이면 'empty'."""
    entry = node_events.get(node_id)
    if entry is None:
        return "empty"
    times = entry["t"]
    if not times or t < times[0]:
        return "empty"
    idx = bisect.bisect_right(times, t) - 1
    return entry["status"][idx]


def _replay_occupied_nodes(traj, t):
    """시각 t 기준 각 OHT가 위치한(또는 직전에 지났던) 노드 id 목록.
    구간 이동 중이라 정확히 노드 위 좌표가 아닐 수 있으므로, 정체 판정
    목적상 "직전에 지난 노드(node0)"를 그 OHT가 속한 노드로 근사한다."""
    nodes = []
    for entry in traj.values():
        times = entry["t"]
        if not times:
            continue
        if t <= times[0]:
            nodes.append(entry["node"][0])
            continue
        if t >= times[-1]:
            nodes.append(entry["node"][-1])
            continue
        idx = bisect.bisect_right(times, t) - 1
        nodes.append(entry["node"][idx])
    return nodes


def render_replay_frame(t, traj, eq_events, vp):
    """render_frame()과 동일한 방식: vp(줌/팬)를 받아 창 크기 그대로
    완성된 프레임을 반환한다."""
    cam = vp.camera()

    static_img = get_static_layer(
        cam, t,
        lambda: _replay_occupied_nodes(traj, t),
        lambda: [e["node"] for e in EQ_PORTS
                 if eq_replay_status(eq_events, e["node"], t) == "empty"],
    )

    frame = Image.new("RGB", (vp.win_w, vp.win_h), BG_COLOR)
    frame.paste(static_img, (0, 0))
    draw = ImageDraw.Draw(frame)

    # [요청 반영] STK/설비(Machine)를 레일 오버레이 위에 그려둔다.
    #   SHOW_EQ_IDLE=True  -> Idle 상태는 정적 레이어에 이미 반투명으로
    #                         깔아뒀으니, 여기서는 실제로 뭔가 진행 중인
    #                         설비(예약/가공/완료 - 개수가 훨씬 적음)만
    #                         불투명하게 그려 눈에 띄게 한다.
    #   SHOW_EQ_IDLE=False -> Idle 설비는 정적 레이어에도 안 그려졌으므로,
    #                         아래 "status == empty: continue"에 의해
    #                         여기서도 진행 중인 설비만 그려진다.
    #                         [설비 상태] extract_eq_status.py가 JOB 로그에서
    #                         뽑아둔 타임라인을 조회해서 그린다. reserved/done은
    #                         실측 타임스탬프 그대로고, processing은 하차 직후
    #                         고정 길이(기본 1초)만큼 임의로 끼워넣은 값이다
    #                         (extract_eq_status.py 상단 docstring 참고).
    eq_r = max(1.0, EQ_R * cam.scale)
    for e in EQ_PORTS:
        node_id = e["node"]
        status = eq_replay_status(eq_events, node_id, t)
        if status == "empty":
            continue
        px, py = cam.pt(sx(e["x"]), sy_pil(e["y"]))
        draw.ellipse((px - eq_r, py - eq_r, px + eq_r, py + eq_r),
                     fill=MACHINE_STATUS_COLOR[status])
    draw_buffer_ports(draw, cam)

    # [성능] EQ/STK 포트 번호는 build_static_layer()의 캐시 이미지에 이미
    # 포함돼 있으므로 여기서 다시 그리지 않는다.

    # [요청 반영] OHT(OHT)를 화면 맨 위(STK/설비보다도 위)에 오도록
    # 마지막에 그린다. 삼각형 로컬 오프셋도 cam.scale로 같이 키운다.
    s = cam.scale
    show_ids = SHOW_OHT_ID and s >= OHT_ID_LABEL_MIN_SCALE
    label_budget = OHT_ID_LABEL_MAX_COUNT if show_ids else 0
    n_loaded = 0
    n_active = 0
    for oht_id, entry in traj.items():
        pos = oht_position(entry, t)
        if pos is None:
            continue
        x, y, car, heading = pos
        n_active += 1
        if car:
            n_loaded += 1
        color = LOADED_COLOR if car else EMPTY_COLOR

        px, py = cam.pt(sx(x), sy_pil(y))
        rad = math.radians(heading)
        cos_a, sin_a = math.cos(rad), math.sin(rad)
        poly = [
            (px + (lx * cos_a - ly * sin_a) * s, py - (lx * sin_a + ly * cos_a) * s)
            for lx, ly in TRI_LOCAL_PTS
        ]
        draw.polygon(poly, fill=color)
        if label_budget > 0 and 0 <= px <= vp.win_w and 0 <= py <= vp.map_win_h:
            # [OHT 번호] 실측 로그의 실제 장비 ID(oht_id)를 그대로 보여준다
            # - oht_fab_sim의 생성형 모드처럼 새로 매긴 번호가 아니라, 실제
            # 로그/DB와 대조할 수 있는 원본 식별자라서 재생 모드에선 이
            # 쪽이 더 유용하다. [명칭] 원본 로그는 "VHL(번호)"로 돼있지만
            # 화면 표기는 "OHT(번호)"로 통일한다(내부 조회/키는 원본
            # oht_id를 그대로 쓰고, 화면에 보여줄 문자열만 접두어를 바꾼다).
            # [성능] 화면 밖이거나 OHT_ID_LABEL_MAX_COUNT를 넘긴 것들은
            # 그리지 않아 화면 가득 OHT가 몰릴 때 프레임 비용을 억제한다.
            display_id = "OHT" + oht_id[3:] if oht_id.startswith("VHL") else str(oht_id)
            draw.text((px, py), display_id, font=FONT_OHT_ID,
                       fill="white", stroke_width=1, stroke_fill="black",
                       anchor="mm")
            label_budget -= 1

    # [한글 폰트] render_frame()과 마찬가지로 전역 FONT_STATS를 그대로
    # 재사용한다(폰트 로딩은 모듈 최상단에서 이미 한 번 끝났으므로 여기서
    # 다시 _load_font()를 호출하지 않는다 - 매 프레임 폰트 파일을 다시
    # 여는 건 불필요한 오버헤드).
    if SHOW_STATS_TEXT:
        hh = int(t // 3600) + 9  # 로그 시작이 09:00:00 이었음
        mm = int((t % 3600) // 60)
        ss = int(t % 60)
        stats = (f"[재생/REPLAY] {hh:02d}:{mm:02d}:{ss:02d}  "
                 f"OHT {n_active}/{len(traj)}   적재중 {n_loaded}대")
        draw_stats_strip(frame, vp, stats)

    return frame


def replay_main(traj_path=REPLAY_JSON, eq_path=REPLAY_EQ_JSON, speed=1.0, start=0.0):
    print(f"[replay] {traj_path} 로딩 중...")
    try:
        with open(traj_path, encoding="utf-8") as f:
            traj = json.load(f)
    except FileNotFoundError:
        # [맵 교체] 재생 데이터는 구 맵(FabLayout.txt) 기준 로그에서 뽑는
        # 별도 산출물이라, 신규 맵으로 넘어오면서 없는 게 정상이다.
        # 여기서 죽지 말고 "OHT가 실제로 움직이는" 생성형 시뮬레이션으로
        # 바로 넘겨준다(사용자가 보고 싶은 건 결국 움직이는 화면이므로).
        print(f"[안내] {traj_path} 이(가) 없어 재생 모드를 건너뜁니다.\n"
              f"       재생 데이터는 실측 로그(OHT_log_only.log)에서 "
              f"extract_oht_replay.py로 별도 추출해야 하는 파일입니다.\n"
              f"       대신 Dijkstra 라우팅 + 물리모델로 OHT가 실제로 움직이는 "
              f"생성형 시뮬레이션을 실행합니다.")
        main()
        return
    n_events = sum(len(d["t"]) for d in traj.values())
    t_end = max((d["t"][-1] for d in traj.values() if d["t"]), default=0.0)
    print(f"[replay] OHT {len(traj)}대, 이벤트 {n_events:,}개, "
          f"재생 구간 0~{t_end:.0f}초({t_end/60:.1f}분) 로드 완료")

    # [맵 교체 검증] 이 재생 데이터는 구 맵(FabLayout.txt) 노드 ID로 기록된
    # 것이라, 신규 맵의 주행노드 집합과 대부분 안 맞는다. 안 맞는 노드는
    # oht_position()이 좌표를 못 찾아 화면에서 그냥 사라지므로, 그 상태를
    # "재생이 원래 저런가 보다"로 오해하지 않도록 여기서 먼저 알린다.
    _sample_nodes = set()
    for _d in traj.values():
        _sample_nodes.update(_d["node"][:50])
        if len(_sample_nodes) >= 3000:
            break
    _hit = sum(1 for n in _sample_nodes if n in NODE_XY)
    _ratio = _hit / len(_sample_nodes) if _sample_nodes else 0.0
    if _ratio < 0.9:
        print(f"[경고] 재생 데이터의 노드 ID가 현재 맵과 {_ratio:.0%}만 일치합니다.\n"
              f"        {traj_path}는 구 맵(FabLayout.txt) 기준으로 추출된 것으로 보입니다.\n"
              f"        신규 맵(Map_LargeFab_CCW_v1_2.txt)으로 재생하려면 "
              f"extract_oht_replay.py를\n"
              f"        신규 맵 기준으로 다시 돌려 파일을 새로 만들어야 합니다.\n"
              f"        (일치하지 않는 노드의 OHT는 화면에 그려지지 않습니다)")

    try:
        with open(eq_path, encoding="utf-8") as f:
            eq_events = json.load(f)
        print(f"[replay] {eq_path} 로딩 완료 (설비 {len(eq_events)}개 "
              f"노드에 reserved/done 타임라인 적용, processing은 미사용)")
    except FileNotFoundError:
        print(f"[경고] {eq_path} 을(를) 찾을 수 없어 설비 상태 없이(빈 원 그대로) "
              f"재생합니다. extract_eq_status.py를 먼저 실행해주세요.")
        eq_events = {}

    pygame.init()
    pygame.display.set_caption(
        "OHT Fab Replay (실측 로그 재생, 대수변경 불가)  "
        "휠/+-: 확대/축소, 우클릭 드래그: 이동, Space: 일시정지, "
        "L: EQ/STK 라벨 on/off, T: 베이/중앙통로 색 on/off, "
        "N: OHT 번호 on/off, R: 리셋")

    display_info = pygame.display.Info()
    screen_w = display_info.current_w or 1920
    screen_h = display_info.current_h or 1080

    vp = Viewport(IMG_W, IMG_H, STATS_H, screen_w, screen_h)
    screen = pygame.display.set_mode((vp.win_w, vp.win_h), pygame.RESIZABLE)
    clock = pygame.time.Clock()

    def set_screen(w, h):
        nonlocal screen
        screen = pygame.display.set_mode((w, h), pygame.RESIZABLE)

    paused = False
    t = start

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                paused = not paused
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_l:
                toggle_show_port_id()
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_t:
                toggle_show_region_tint()
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_n:
                toggle_show_oht_id()
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_d:
                toggle_show_node_dots()
            elif event.type == pygame.KEYDOWN and event.key in (
                    pygame.K_LEFTBRACKET, pygame.K_RIGHTBRACKET):
                adjust_junction_dot_threshold(
                    -1 if event.key == pygame.K_LEFTBRACKET else 1)
            else:
                vp.handle_event(event, set_screen)
        if not running:
            break

        if not paused:
            t += speed / TARGET_FPS
            if t > t_end:
                t = t_end
                paused = True
                print("[replay] 재생 구간 끝(로그 마지막 시각 도달). 일시정지.")

        frame = render_replay_frame(t, traj, eq_events, vp)
        surf = pygame.image.fromstring(frame.tobytes(), frame.size, frame.mode)
        screen.blit(surf, (0, 0))
        pygame.display.flip()

        clock.tick(TARGET_FPS)

    pygame.quit()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--analyze", action="store_true",
                         help="pygame 창 없이 용량 산정(필요대수/가동률/병목) headless 분석만 실행")
    parser.add_argument("--baseline-oht", type=int, default=MAX_OHT,
                         help="1단계 기준 대수(기본 MAX_OHT=%d)" % MAX_OHT)
    parser.add_argument("--sim-hours", type=float, default=2.0,
                         help="각 회차 시뮬레이션 시간(시간 단위, 기본 2시간)")
    parser.add_argument("--target-util-low", type=float, default=0.80)
    parser.add_argument("--target-util-high", type=float, default=0.85)
    parser.add_argument("--replay", action="store_true",
                         help="extract_oht_replay.py로 만든 실측 재생(REPLAY_JSON)을 "
                              "재생한다. 대수변경 불가. [주의] 현재 재생 파일은 구 맵 "
                              "노드 ID 기준이라 신규 맵과 맞지 않는다(실행 시 일치율 "
                              "경고 표시).")
    parser.add_argument("--generate", action="store_true",
                         help="[기본값] Dijkstra 라우팅+물리모델로 OHT를 새로 "
                              "'생성'해서 움직이는 시뮬레이션. 아무 옵션 없이 "
                              "실행해도 이 모드로 켜진다.")
    parser.add_argument("--congestion-compare", action="store_true",
                         help="pygame 창 없이, 같은 대수로 저부하/고부하 2가지 "
                              "투입률을 돌려 운반시간/처리량/정체(10초+ 정지) "
                              "지표를 구간별 시계열로 비교하고 CSV로 저장한다.")
    parser.add_argument("--region-report", action="store_true",
                         help="pygame 창 없이 headless 시뮬레이션 1회를 돌리고, "
                              "정체(락대기/10초+ 정지)를 베이/중앙통로/미분류 "
                              "구역별로 집계해 출력한다(section_regions.json 필요, "
                              "build_section_regions.py로 생성).")
    parser.add_argument("--export-metrics", action="store_true",
                         help="pygame 창 없이 headless 시뮬레이션 1회를 돌리고, "
                              "OHT별 지표(가동률/실동률/병목시간 원인분해/주행거리)와 "
                              "구역별 지표(베이/중앙통로/합류부의 정체시간)를 "
                              "각각 CSV로 저장한다.")
    parser.add_argument("--oht-csv", type=str, default="oht_metrics.csv",
                         help="--export-metrics OHT별 지표 CSV 경로 (기본 oht_metrics.csv)")
    parser.add_argument("--region-csv", type=str, default="region_metrics.csv",
                         help="--export-metrics 구역별 지표 CSV 경로 (기본 region_metrics.csv)")
    parser.add_argument("--bin-seconds", type=float, default=60.0,
                         help="--congestion-compare 시계열 구간 길이(초, 기본 60)")
    parser.add_argument("--light-spawn-interval", type=float, default=None,
                         help="--congestion-compare 저부하 투입 간격(초). "
                              "생략 시 SPAWN_INTERVAL*3")
    parser.add_argument("--heavy-spawn-interval", type=float, default=None,
                         help="--congestion-compare 고부하 투입 간격(초). "
                              "생략 시 SPAWN_INTERVAL/3")
    parser.add_argument("--oht", type=int, default=None,
                         help="화면 시뮬레이션(기본/--generate)에서 띄울 OHT 대수. "
                              "생략 시 MAX_OHT=%d" % MAX_OHT)
    parser.add_argument("--spawn-interval", type=float, default=None,
                         help="화면 시뮬레이션의 신규 작업 투입 간격(초). 값을 줄이면 "
                              "일하는 OHT가 늘고, 키우면 배회하는 OHT가 늘어난다. "
                              "생략 시 SPAWN_INTERVAL=%.2f" % SPAWN_INTERVAL)
    parser.add_argument("--stall-threshold", type=float, default=None,
                         help="정체로 칠 최소 정지 시간(초). 생략 시 기본 %.0f초"
                              % STALL_THRESHOLD_SECONDS)
    parser.add_argument("--traj", type=str, default=REPLAY_JSON,
                         help="재생용 trajectory json 경로 (기본: %s)" % REPLAY_JSON)
    parser.add_argument("--eq-traj", type=str, default=REPLAY_EQ_JSON,
                         help="재생용 설비 상태 json 경로 (extract_eq_status.py "
                              "출력, 기본: %s)" % REPLAY_EQ_JSON)
    parser.add_argument("--speed", type=float, default=1.0,
                         help="재생 배속 (--replay 전용, 기본 1.0=실측 그대로 실시간)")
    parser.add_argument("--start", type=float, default=0.0,
                         help="재생 시작 시각(초, --replay 전용, 로그 시작 09:00:00 기준)")
    args = parser.parse_args()

    if args.stall_threshold is not None:
        STALL_THRESHOLD_SECONDS = args.stall_threshold

    if args.analyze:
        analyze_fleet_capacity(baseline_oht=args.baseline_oht, sim_hours=args.sim_hours,
                                target_util_low=args.target_util_low,
                                target_util_high=args.target_util_high)
    elif args.congestion_compare:
        run_congestion_comparison(
            num_oht=args.baseline_oht, sim_hours=args.sim_hours,
            light_spawn_interval=args.light_spawn_interval,
            heavy_spawn_interval=args.heavy_spawn_interval,
            bin_seconds=args.bin_seconds)
    elif args.region_report:
        _result = run_headless_sim(args.baseline_oht, args.sim_hours * 3600.0,
                                    return_raw=True)
        print(f"[region-report] {args.baseline_oht}대 / {args.sim_hours}시간  "
              f"완료건수={_result['n_jobs']}  가동률(배정)={_result['utilization']*100:.1f}%")
        compute_region_congestion(_result["wait_counts"], _result["stall_events"])
    elif args.export_metrics:
        _dur = args.sim_hours * 3600.0
        _result = run_headless_sim(args.baseline_oht, _dur, return_raw=True)
        print(f"\n[export-metrics] {args.baseline_oht}대 / {args.sim_hours}시간  "
              f"완료건수={_result['n_jobs']}  "
              f"가동률(배정)={_result['utilization']*100:.1f}%  "
              f"가동률(실동)={_result['utilization_active']*100:.1f}%")
        print_region_ranking()
        print()
        export_oht_metrics_csv(_result["ohts"], _dur, path=args.oht_csv)
        export_region_metrics_csv(_dur, args.baseline_oht, path=args.region_csv)
    elif args.generate:
        main(num_oht=args.oht, spawn_interval=args.spawn_interval)
    else:
        # [맵 교체로 기본값 되돌림] 예전엔 옵션 없이 실행하면 실측 재생이
        # 떴지만, 재생 데이터(oht_replay_trajectories.json)는 구 맵 노드 ID
        # 기준이라 신규 맵에서는 대부분의 OHT가 화면에 안 나온다. 그래서
        # 기본 실행을 다시 생성형 시뮬레이션으로 되돌린다. 재생을 보려면
        # --replay를 명시적으로 주면 되고, 그때 노드 ID 일치율 경고가
        # replay_main()에서 함께 표시된다.
        main(num_oht=args.oht, spawn_interval=args.spawn_interval)
