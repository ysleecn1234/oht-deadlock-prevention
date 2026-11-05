"""원본과 작업본 node 모드를 동일 입력에서 매초 전체 서명으로 대조한다."""
import argparse
from pathlib import Path

from experiment_support import PROVIDED_SIMULATOR_DIRECTORY, load_simulator, simulation_signature


def collect_trace(case_directory, vehicle_count, duration_seconds, source_directory=None, merge_mode="OCS"):
    simulator = load_simulator(case_directory, occupancy_mode="node", source_directory=source_directory)
    simulator.MERGE_CONTROL_MODE = merge_mode
    simulator.random.seed(1)
    simulator.reset_sim_state()
    simulator.SPAWN_INTERVAL = 0.5
    simulator.sim.yieldless(False)
    environment = simulator.sim.Environment(trace=False)
    for node in simulator.get_park_nodes(vehicle_count):
        simulator.OHT(park_node=node)
    simulator.manager = simulator.Manager()
    simulator.dispatcher = simulator.Dispatcher()
    simulator.StallWatchdog()
    simulator.CongestionMonitor()
    simulator.IdleDistributor()
    if simulator.PROGRESS_INTERVAL:
        simulator.ProgressReporter(interval=simulator.PROGRESS_INTERVAL, total=duration_seconds)
    trace = []

    class SignatureRecorder(simulator.sim.Component):
        def process(self):
            while True:
                trace.append([self.env.now(), simulation_signature(simulator)])
                yield self.hold(1)

    SignatureRecorder()
    environment.run(till=duration_seconds)
    trace.append([environment.now(), simulation_signature(simulator)])
    return trace


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case-directory", type=Path, required=True)
    parser.add_argument("--num-oht", type=int, default=100)
    parser.add_argument("--sim-duration", type=int, default=180)
    parser.add_argument("--merge-mode", choices=("OCS", "ADAPTIVE", "FCFS"), default="OCS")
    arguments = parser.parse_args()
    expected = collect_trace(arguments.case_directory.resolve(), arguments.num_oht,
                             arguments.sim_duration, PROVIDED_SIMULATOR_DIRECTORY, arguments.merge_mode)
    actual = collect_trace(arguments.case_directory.resolve(), arguments.num_oht,
                           arguments.sim_duration, merge_mode=arguments.merge_mode)
    differences = [index for index, pair in enumerate(zip(expected, actual)) if pair[0] != pair[1]]
    report = {"is_equal": expected == actual, "vehicle_count": arguments.num_oht,
              "simulation_seconds": arguments.sim_duration, "observed_states": len(expected),
              "different_state_indices": differences, "original_trace": expected, "working_trace": actual}
    if not report["is_equal"]:
        raise AssertionError("기존 node 모드와 원본의 매초 상태가 다름")
    print(f"원본·작업본 node 모드 {len(expected)}개 시각 일치", flush=True)


if __name__ == "__main__":
    main()
