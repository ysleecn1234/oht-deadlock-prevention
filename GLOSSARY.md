# 약어 용어집

교수님 요청사항: "코드 작성할 때 약어들을 바꾸라", "SI 같은 약어는 최대한
쓰지 않고 뜻을 전달할 수 있는 내용으로 말하기".

**새로 작성하는 코드는 약어를 쓰지 않습니다.** 다만 원본 시뮬레이터
(`agv_fab_sim.py`)와 기존 실험 스크립트는 이름을 바꾸면 과거 로그와의
대조가 끊기므로 그대로 두고, 여기에 뜻을 정리합니다.

출처를 밝힌 항목은 원본 코드에 설명이 있는 것이고, **"원본에 정의 없음"**
으로 표시한 항목은 코드 어디에도 풀어 쓴 설명이 없어 확인이 필요한
것입니다. 추측해서 적지 않았습니다.

---

## 1. 도메인 용어

| 약어 | 뜻 | 출처 |
|---|---|---|
| OHT | 천장 주행 반송차 (Overhead Hoist Transport) | 원본에 정의 없음 (업계 표준어) |
| FOUP | 웨이퍼 운반 용기 (Front Opening Unified Pod) | 원본에 정의 없음 |
| LOT | 로트. 설비를 거쳐 가는 자재 한 묶음 | — |
| leg | 로트가 한 구간을 이동하는 단위. 로트 1건이 여러 leg로 쪼개짐 | — |
| EQ | 설비 (Equipment). 가공이 일어나는 포트 | 150~160행 |
| STK | 스토커. 자재를 처음 받고 마지막에 되돌리는 창고 | 151, 156행 |
| STB | 트랙 사이드 버퍼. 설비 옆 임시 적치 포트 | 418행, 155~156행 |
| UTB | 포트 종류의 하나 | **원본에 정의 없음** (569행에 목록만 있음) |
| TL | 포트 종류의 하나 | **원본에 정의 없음** (569행) |
| CONV | 포트 종류의 하나 | **원본에 정의 없음** (569행) |
| MLUD | 현장 맵에만 있는 포트 종류 | **원본에 정의 없음** (569행) |

569행 주석: `Rev7.1 Port.Type: EQ / STK / STB / UTB / TL / CONV (+ 현장맵 MLUD)`
573행 주석: `CONV / MLUD / TL = OTHER_PORTS로 분류만 되어 있고, 로트 흐름 로직에는 관여하지 않음`

로트 흐름(160행): `STK -> [STB] -> EQ(가공, 딱 1번) -> [STB] -> STK`

---

## 2. 단위 표기

| 꼬리말 | 뜻 | 예 |
|---|---|---|
| `_MPS` | 초당 미터 (m/s) | `OHT_SPEED_MPS = 3.5` |
| `_MPS2` | 초당 미터 제곱 (m/s²), 가감속도 | `OHT_ACCEL_MPS2 = 2.0` |
| `_MM` | 밀리미터 | `OHT_LENGTH_MM = 850.0` |
| `_M` | 미터 | `MIN_FOLLOW_DISTANCE_M = 1.15` |
| `_SEC` | 초 | `RAIL_CONGESTION_REFRESH_SEC` |
| (꼬리말 없음) | **도면 단위 = 밀리미터** (`UNIT_PER_METER = 1000.0`) | `MIN_FOLLOW_DISTANCE = 1150.0` |

같은 값이 `_M` 과 꼬리말 없는 두 형태로 함께 존재합니다. 꼬리말 없는 쪽이
실제 계산에 쓰이는 도면 단위(mm) 값입니다.

---

## 3. 코드 구조 약어

| 약어 | 뜻 |
|---|---|
| `IDX` | 인덱스 (index). `EDGE_LINK_IDX` = (노드a, 노드b) -> 링크 번호 |
| `ADJ` | 인접 리스트 (adjacency). 노드에서 나가는 링크 목록 |
| `SUCC` | 후속 (successor). `SUCC_LINKS` = 어떤 링크 다음에 올 수 있는 링크들 |
| `LEN` | 길이 (length). `LINK_LEN[i]` = 링크 i의 주행거리(mm) |
| `LINK_A` / `LINK_B` | 링크의 시작 노드 / 끝 노드 |
| `CSR` | 압축 희소 행 (Compressed Sparse Row). 큰 희소 행렬을 메모리 적게 저장하는 형식. 주행시간 행렬에 사용 (1463, 2770행) |
| `POS` | 위치 (position) |
| `TOL` | 허용 오차 (tolerance). `BAY_RING_LANE_TOL_M` |
| `PTS` / `TRI` | 점 목록 (points) / 삼각형 (triangle). 화면에 OHT를 삼각형으로 그릴 때 사용 |
| `R` | 반지름 (radius). `EQ_R`, `STK_R` = 화면 표시 반경(픽셀) |
| `BG` | 배경 (background) |
| `IMG_W` / `IMG_H` / `PAGE_W` / `PAGE_H` | 이미지·페이지의 가로(width) / 세로(height) |
| `SRC` | 출발지 (source). `DIJKSTRA_SRC_CHUNK` |
| `T0` | 기준 시각. `MOVETIME_T0` = 점수가 절반으로 떨어지는 기준 시간(초) |

---

## 4. 배차 점수 항 (`W_` 로 시작하는 것들)

2681행: `Total Score = W_movetime + W_priority + W_delay + W_port`

`W` 는 가중치(Weight)이며, 어느 OHT에게 작업을 줄지 고를 때 더하는 항목입니다.

| 항 | 뜻 |
|---|---|
| `W_movetime` | 그 OHT가 픽업 지점까지 가는 데 걸리는 시간. 가까울수록 높은 점수 |
| `W_priority` | 상위 시스템이 준 작업 긴급도 (최대 50) |
| `W_delay` | 작업이 생성된 뒤 지연된 시간 |
| `W_port` | 포트 가중치 |

---

## 5. 기존 실험 스크립트 환경변수 (`knobs.py`)

아카이브된 실험 로그는 아래 약어 환경변수로 실행되었습니다.
**새 스크립트에서는 쓰지 않습니다.**

| 약어 | 대응하는 시뮬레이터 설정 | 뜻 |
|---|---|---|
| `N` | OHT 대수 | |
| `SI` | `SPAWN_INTERVAL` | 로트 투입 간격(초). 0.45초 = 8,000건/시간 |
| `H` | 시뮬레이션 시간(시간) | |
| `EVERY` | 감시 주기(시뮬 초) | |
| `RC` | `ENABLE_RING_CONTROL` | 베이 고리 진입 제어 on/off |
| `M` | `RING_CAP_MARGIN` | 고리 정원 = 노드 수 // 2 − 이 값 |
| `RCI` | `RING_COUNT_IDLE` | 고리 정원에 유휴 차량도 셀지 |
| `T1` | `ENABLE_JUNCTION_REROUTE` | 분기점 재탐색 on/off |
| `T2` | `JUNCTION_STALL_REROUTE` | 같은 분기점에서 이 시간 못 지나가면 우회(초) |
| `FD` | `FORCE_DETOUR_SECONDS` | 강제 우회까지의 누적 정체 시간(초) |
| `RETRY` | `MAX_PATH_RETRY` | 경로탐색 재시도 횟수 |
| `YIELD` | `request_yield` | 0이면 길 비켜주기 기능을 꺼버림 |
| `CMP` | `CONGESTION_MAX_PENALTY` | 혼잡 가중치 상한 |
| `CA` | `CONGESTION_ALPHA` | 혼잡도 반영 계수 |
| `IRI` | `IDLE_REBALANCE_INTERVAL` | 유휴 차량 재분산 주기(초) |
| `MPW` | `MERGE_PRIORITY_WINDOW` | 합류부 우선권 유지 시간(초) |
| `RSB` | `REASSIGN_STICKY_BONUS` | 재배정 시 기존 차량에 주는 보너스 점수 |
| `RING` | — | 시작 위치를 특정 고리 노드로 몰아넣기 |
| `PIN` | — | 목적지를 그 고리 안 포트로 묶기 |

---

## 6. 새 코드의 명명 규칙

- 약어를 쓰지 않고 뜻이 드러나는 이름을 씁니다
  (`SI` 대신 `SPAWN_INTERVAL_SECONDS`, `N` 대신 `OHT_COUNT`)
- 단위를 이름에 포함합니다 (`_SECONDS`, `_MM`, `_METERS`)
- 결과 파일 이름도 같은 규칙을 따릅니다
  예: `seed00_oht400_interval0.45_fix_on.json`
