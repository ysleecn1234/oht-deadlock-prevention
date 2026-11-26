# OHT V2 확정 교착 17개 시드
각 seedNNN 폴더에는 해당 실험의 원본 실행 폴더 전체가 들어 있다. original_file_hashes.json에는 원본 파일의 상대 경로, 크기, SHA-256이 있다. index.json에서 검증 시각 checked_t와 마지막 기록 시각을 확인할 수 있다. 실제 교착 시작 시각은 별도로 검토해야 한다.
## 1초 로그 읽기
Git LFS를 사용하는 저장소에서 내려받은 경우 먼저 저장소에서 git lfs pull을 실행한다. 아래 코드는 이 README와 같은 폴더에서 Python으로 실행한다. Python 표준 라이브러리만 사용한다.
```python
from compact_observer import read_states
for state in read_states("seed052/states.jsonl.gz"):
    print(state["t"], len(state["vehicles"]), len(state["waits"]), state["completed_lots"])
    if state["t"] >= 10:
        break
```
로그에는 처음과 300초마다 전체 상태가 있고 그 사이에는 매초 변경분이 있다. read_states는 이를 순서대로 합쳐 매초의 전체 상태를 반환한다. 전체 기간을 분석하려면 예시의 break 조건을 제거한다. 대용량이므로 순차 처리한다. 기록은 관측 상태이며 시뮬레이터 재시작 체크포인트는 아니다.
## 함께 제공하는 자료
- 각 시드의 deadlock_verdict.json, verification.jsonl, candidates.jsonl, result.json에서 최종 판정과 후보 이력을 확인한다.
- config.json, runtime.json, 지도·설비 JSON, 초기 차량 위치, 로트 스케줄, 실행 당시 시뮬레이터 소스는 각 시드 폴더 안에 있다. 일부 원본 메타데이터의 절대 경로는 수집 PC 기준이다.
- gui_captures에는 시뮬레이터가 저장한 원래 이미지와 캡처 명세가 있다.
- feature_comparisons.json에는 교착 확인 10분 전·5분 전·확인 시점의 기존 분석 값이 있다.
- 이번 17건은 네 제어와 StallWatchdog을 모두 끈 조건의 확정 사례다.
