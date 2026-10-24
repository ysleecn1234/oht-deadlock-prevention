# 중앙통로 호이스팅 실험용 작업본

기준 커밋: `f71735e4871edede5cab82e3b3506b9a003b8439`. 팀원 원본·기존 simulator_distance_fixed·GitHub는 수정하지 않았다. 이 폴더에서만 추가 기능을 실행한다.

이 브랜치는 팀원 `jisuk/oht-scale-workload`의 위 커밋을 기반으로 `simulator_hoist/`를 추가한다. 기존 `dnd_work/`는 그대로 보존한다. 변경 설명은 [CHANGELOG.md](CHANGELOG.md), 위치 근거와 지도는 [docs/README.md](docs/README.md)에 있다.

처음 설치할 때 `python -m pip install -r requirements.txt`를 실행한다. Python 3.10.11, pygame 2.6.1 환경에서 검증했다.

## 지도 화면으로 보기

```powershell
cd simulator_hoist
python view_hoist.py --mode 4 --oht 1000
```

호이스팅 노드에 주황색 원과 H1/H2/H3 라벨을 표시하며, 해당 노드에서 수행 중이면 빨간색으로 표시한다. 휠로 확대/축소, 우클릭 드래그로 이동, Esc로 종료한다. Windows에서는 `호이스팅_시뮬레이터.pyw` 더블클릭으로 같은 기본 설정을 실행할 수 있다.

GUI는 주행·호이스팅 관찰용이며 데드락 판정에 의한 자동 종료는 하지 않는다. 창을 닫으면 runs/gui_*.json에 호이스팅 기록을 저장한다. 데드락 탐색 결과는 아래 탐지기 실행 방식으로 수집한다. 기존 `python agv_fab_sim.py`도 GUI가 열리지만 별도 환경변수가 없으면 모드 0이며, 관찰용 표시/투입 제한/저장은 view_hoist.py에서 제공한다.

## 모드

| HOIST_MODE | 활성 노드 |
|---|---|
| 0 | 없음 (기본값) |
| 1 | 017192 |
| 2 | 016911 |
| 3 | 017024 |
| 4 | 위 세 노드 |

모든 노드는 중앙통로 2의 일반 주행 노드이며, 기준 지도에서 포트 node/transfer_node에 없고 20,000건 스케줄 목적지에도 없다. 지도/포트가 바뀌면 위치 조건을 재검증해야 한다.

## 실행

PowerShell에서 이 폴더로 이동 후 실행한다:

```powershell
.\run_hoist.ps1 -Mode 4 -Probability 0.1 -Seconds 8 -HoistSeed 42 -OhtCount 400 -Hours 1
```

모드 0~4 비교 시 Mode만 바꾼다. 기본 확률 10%와 시드 42는 실행 가능한 초기 설정이지 확정된 실험 조건이나 실측값은 아니다. 스크립트 실행 정책으로 막히면 실행 정책을 변경하지 말고 아래 Python 직접 실행 방식을 쓴다.

```powershell
$env:HOIST_MODE = '4'
$env:HOIST_PROBABILITY = '0.1'
$env:HOIST_SECONDS = '8'
$env:HOIST_SEED = '42'
$env:OHT_COUNT = '400'
$env:SIMULATION_HOURS = '1'
$env:RESULT_JSON_PATH = 'hoist_mode4.json'
python -X utf8 deadlock_detector.py
```

직접 실행은 같은 이름의 결과 파일을 덮어쓰므로 새 이름을 지정한다. PowerShell 실행기는 runs/에 모드와 시각을 붙인 별도 결과를 만든다. 작업본의 시작위치 파일에는 1,000개 위치가 있어 실행기는 OhtCount를 1,000 이하로 제한한다. 1,500/2,000대 다중 시드 실험은 generate_case_files.py로 충분한 초기 위치를 가진 별도 케이스를 생성하고 CASE_DIRECTORY를 지정해 탐지기를 직접 실행해야 한다. 아직 대규모 실험을 시작하지 않았다.

실행기는 원본 실험 조건인 로트 투입 시도 0.45초 간격, 대기열 상한 2,000건, 교착 판정 600초, 고리 제어 켬을 명시한다. 이는 실제 투입 8,000건/h를 보장하는 조건은 아니다. 기존 스케줄 JSON은 바꾸지 않는다.

## 동작

- 적재 운반 상태(to_dest)의 차량만 대상이다. 빈 차·유휴 주행은 제외한다.
- 경로 준비 시 SHA-256(seed, lot_id, leg_index, node, visit)을 통해 [0,1) 값을 얻어 확률과 비교한다. Python hash(), 공유 난수, OHT 번호, 모드 번호를 사용하지 않는다.
- 방문 횟수는 실제 노드 도착 때만 증가한다. 재경로 탐색/점유 재시도는 방문으로 세지 않는다. 작업의 출발·도착 노드는 중복 상하차 방지를 위해 제외한다.
- 경로상의 선정 정지점까지 거리로 탈출 속도를 제한하여 여러 링크에 걸쳐 미리 감속한다. 도착 속도가 0인지 확인하고 현재 노드 점유를 유지한 채 대기한다. 외부 wakeup으로 8초가 단축되지 않게 남은 시간을 다시 기다린다.
- 작업 상태는 hoisting으로 표시하고, 정상 서비스 시간은 락 정체 시간과 별도로 집계한다. 이 차량 자체는 대기 그래프의 막힌 차량으로 취급하지 않는다.
- 모드별로 다른 지점의 정지 때문에 경로와 시각은 달라질 수 있다. 동일 작업·노드·방문 번호의 판정은 단독/복합 모드에서 같다.
- 기존 라우팅 비용/배차 ETA는 추가 호이스팅을 최적화하지 않는다. 주행 시 정지 제약을 적용하는 방식이며, 기존 배차 알고리즘은 유지한다.

## 결과

탐지기 결과 JSON의 hoisting 아래 설정, 실제 도착 방문 수, 선정 이벤트 수, 완료 수, 관측한 호이스팅 총시간, events를 저장한다. 이벤트에는 작업/로트/차량/노드/방문 번호, hash score, 판정, 도착·시작·종료 시각을 기록한다. 선정되지 않은 방문은 passed, 진행 중 종료는 running 및 end_second=null이다.

job 통계의 hoist는 추가 서비스 시간이고 loadunload는 원래 포트 상하차 시간이다. active 통계에 호이스팅을 포함한다. 실행 도중 중단한 서비스도 탐지기 결과의 elapsed_hoist_seconds 및 진행 작업 active 계산에 반영한다. 원본 GUI/일반 분석 CLI 대신 결과와 이벤트 기록을 위해 deadlock_detector.py를 사용한다.

## 검증

```powershell
python -X utf8 -m unittest test_hoisting
python -X utf8 verify_all.py
```

모드 0~4 실지도 주행, 8초 대기, 정지 속도/노드 점유, 중간 wakeup, 단독·복합 판정 일치, 동일 프로세스 조건 반복 로그 일치, 20대/120초 모드 0 원본 대비 결과 비교를 검사한다. 상세 출력은 verification/에 저장된다. 이 짧은 검증은 고부하 전체 맵에서의 충돌/교착 검증을 대신하지 않는다.

원본 비교 검증은 Git 이력에서 기준 커밋의 시뮬레이터를 읽는다. Git clone한 저장소에서 실행해야 하며, ZIP만 받은 경우 단위 테스트와 verify_motion.py는 실행할 수 있지만 verify_all.py의 원본 비교에는 기준 Git 이력이 필요하다.
