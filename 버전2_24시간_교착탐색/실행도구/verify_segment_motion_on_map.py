"""세 합류 모드의 실지도 주행과 관측 전후 동작을 검사. 작업 보고 파일 생성 없음."""
import argparse
import contextlib
import io
import tempfile
import time
from pathlib import Path

from experiment_support import EXPERIMENT_ROOT, load_simulator, simulation_signature, verify_provided_files
from integrated_deadlock_observer import IntegratedDeadlockRecorder
from run_integrated_deadlock_experiment import run_with_observation


def run_case(case_directory, vehicle_count, duration, mode, observed):
    with contextlib.redirect_stdout(io.StringIO()):
        simulator = load_simulator(case_directory,"segment")
    simulator.MERGE_CONTROL_MODE = mode
    started = time.monotonic()
    recorder = None
    with tempfile.TemporaryDirectory(prefix="oht-map-check-") as temporary, contextlib.ExitStack() as cleanup:
        if observed:
            recorder = IntegratedDeadlockRecorder(simulator,Path(temporary),duration,case_directory)
            for handle in (recorder.delta_handle,recorder.region_handle,recorder.event_handle):
                cleanup.callback(handle.close)
            run_with_observation(simulator,vehicle_count,duration,.5,1,recorder)
            recorder.close(duration,True)
        else:
            simulator.run_headless_sim(vehicle_count,duration,.5,1)
        controller = simulator.get_segment_motion()
        controller.check_safety()
        assert not controller.safety_failures
        assert abs(sum(row[4] for row in simulator.MERGE_PASS_LOG) -
            sum(row["node_wait"] for row in simulator.MERGE_NODE_STATS.values())) < 1e-5
        result = simulation_signature(simulator)
        confirmed = (sum(not record["revoked"] for record in recorder.tracker.confirmed.values())
                     if recorder else "미검사")
        print(f"mode={mode} observed={observed} vehicles={vehicle_count} simulated_seconds={duration} "
              f"wall_seconds={time.monotonic()-started:.3f} transfers={len(simulator.JOB_STATS)} "
              f"completed_lots={len(simulator.LOT_STATS)} merge_passes={len(simulator.MERGE_PASS_LOG)} "
              f"merge_stopped_wait_seconds={sum(row[4] for row in simulator.MERGE_PASS_LOG):.6f} "
              f"minimum_gap_mm={controller.minimum_observed_gap_mm:.9f} safety_failures=0 sufficient_condition_confirmations={confirmed}",flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--num-oht",type=int,default=1000)
    parser.add_argument("--sim-duration",type=int,default=180)
    parser.add_argument("--seed-case",default="시드01")
    parser.add_argument("--modes",nargs="+",choices=("OCS","ADAPTIVE","FCFS"),default=["OCS","ADAPTIVE","FCFS"])
    parser.add_argument("--compare-observation",action="store_true")
    arguments = parser.parse_args()
    case = EXPERIMENT_ROOT / "입력자료" / "안전초기배치_통과구역검증" / arguments.seed_case
    for mode in arguments.modes:
        print(f"검사 시작: {mode} {arguments.num_oht}대 {arguments.sim_duration}초",flush=True)
        observed = run_case(case,arguments.num_oht,arguments.sim_duration,mode,True)
        if arguments.compare_observation:
            unobserved = run_case(case,arguments.num_oht,arguments.sim_duration,mode,False)
            assert observed == unobserved, "관측 전후 실제 위치·주행 계획·합류 대기열·작업 기록 불일치"
            print(f"관측 전후 전체 주행 상태·작업 기록 일치: {mode}",flush=True)
    verify_provided_files()


if __name__ == "__main__": main()
