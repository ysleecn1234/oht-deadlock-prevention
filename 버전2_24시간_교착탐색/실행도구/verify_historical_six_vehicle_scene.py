"""5차 기록의 여섯 정지 위치만 새 주행 모델에서 재생. 전체 실험 재현과 구분."""
import json
from types import SimpleNamespace

from experiment_support import EXPERIMENT_ROOT, PROJECT_ROOT, load_simulator, current_timestamp


def main():
    reference = PROJECT_ROOT / "dnd_work/oht-deadlock-prevention/runs/5차_교착확정/5차_s01_oht1500_교착확정.json"
    original = json.loads(reference.read_text())["structural_deadlocks"][0]
    simulator = load_simulator(EXPERIMENT_ROOT / "입력자료/안전초기배치_통과구역검증/시드01", "segment")
    simulator.reset_sim_state()
    simulator.sim.yieldless(False)
    environment = simulator.sim.Environment(trace=False)
    controller = simulator.get_segment_motion()

    class SceneVehicle(simulator.OHT):
        def setup(self, record):
            super().setup(park_node=record["node"])
            self.oht_id = record["oht_id"]
            self.job = SimpleNamespace(state=simulator.JOB_CARRYING)
            self.state = simulator.OHT_TO_DEST
            self.target_node = record["job_to_node"]
            self._abort_leg = False
            self.reached_second = None

        def process(self):
            arrived = yield from controller.goto(self, self.target_node)
            if arrived:
                self.reached_second = self.env.now()
            yield self.passivate()

    vehicles = [SceneVehicle(record=record) for record in original["members"]]
    simulator.StallWatchdog()
    for second in range(601):
        environment.run(till=second)
        controller.check_safety()
    rows = [{"oht_id": vehicle.oht_id, "target_node": vehicle.target_node,
             "reached_second": vehicle.reached_second, **controller.get_position(vehicle)} for vehicle in vehicles]
    report = {"verified_at": current_timestamp(), "reference_path": str(reference),
        "reference_detected_second": original["detected_second"], "simulated_seconds": 600,
        "vehicle_count": 6, "vehicles_reaching_original_target": sum(row["reached_second"] is not None for row in rows),
        "safety_failures": controller.safety_failures, "vehicles": rows,
        "scope": "기록된 여섯 정지 위치와 각 목적 G 노드만 재생. 기존 1,500대·6시간·투입 0.45 전체 실행, 당시 포트 작업·허가 상태·기존 경로를 재현한 결과가 아님"}
    print(json.dumps({"vehicles_reaching_original_target": report["vehicles_reaching_original_target"],
                      "safety_failure_count": len(controller.safety_failures)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
