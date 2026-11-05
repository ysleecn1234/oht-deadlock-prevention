"""실제 발생 건수와 구분하는 구간 모델 충분조건·오인 방지 시험."""
import copy
import unittest

from segment_deadlock_proof import SegmentDeadlockProofInspector
from test_deadlock_proof import model, snapshot


class SegmentProofTests(unittest.TestCase):
    def make_state(self):
        graph, _ = model([("a", "b"), ("b", "c"), ("c", "d"), ("d", "a")])
        regions = {"고리_0": {"nodes": list("abcd"), "cap": 4}}
        state = snapshot(graph, dict(zip("abcd", "bcda")), now=300)
        state.update(occupancy_mode="segment", RING_MEMBERS=[[0, 1, 2, 3]])
        for vehicle in state["vehicles"]:
            number = vehicle["oht_id"]
            vehicle.update(is_route_active=True, movement_options_are_complete=True,
                movement_options=[{"blocking_vehicle_id": (number + 1) % 4, "distance_mm": 1150, "link_indices": [number]}],
                physical_ring_numbers=[0], transfer_node=None)
        state["wait_dependencies"] = [{"oht_id": number, "blocking_vehicle_ids": [(number + 1) % 4],
            "cause": "follow_distance", "resource_name": None, "details": {}} for number in range(4)]
        return graph, regions, state

    @staticmethod
    def types(proofs):
        return {item["type"] for proof in proofs for item in proof["confirmed_types"]}

    def test_four_type_evidence_and_free_exit_counterexamples(self):
        graph, regions, first = self.make_state()
        second = copy.deepcopy(first)
        for number, vehicle in enumerate(second["vehicles"]):
            vehicle["physical_ring_numbers"] = [number // 2]
        third = copy.deepcopy(first)
        third["vehicles"][0]["_merge_wait"] = ["b", 0]
        third["vehicles"][2]["_merge_wait"] = ["d", 2]
        fourth = copy.deepcopy(third)
        fourth["wait_dependencies"][0]["details"] = {"merge_node": "b", "outgoing_segment_full": True}
        earlier = copy.deepcopy(fourth)
        earlier["simulation_second"] = 200
        earlier["wait_dependencies"] = [earlier["wait_dependencies"][0]]
        for vehicle in earlier["vehicles"][1:]:
            vehicle["is_route_active"] = False
        for name, state in (("TYPE I", first), ("TYPE II", second), ("TYPE III", third), ("TYPE IV", fourth)):
            with self.subTest(type=name):
                inspector = SegmentDeadlockProofInspector(graph, regions)
                if name == "TYPE IV":
                    self.assertNotIn(name, self.types(inspector.inspect(state)))
                    inspector = SegmentDeadlockProofInspector(graph, regions)
                    inspector.inspect(earlier)
                self.assertIn(name, self.types(inspector.inspect(state)))
                counterexample = copy.deepcopy(state)
                counterexample["vehicles"][0]["movement_options"].append({"blocking_vehicle_id": None})
                counterexample["simulation_second"] = 86400
                self.assertEqual(inspector.inspect(counterexample), [])

    def test_service_incomplete_evidence_and_movement_never_confirmed(self):
        graph, regions, state = self.make_state()
        for attribute, value in (("is_route_active", False), ("movement_options_are_complete", False),
                                 ("_cur_speed", 1), ("job_state", "assigned")):
            counterexample = copy.deepcopy(state)
            counterexample["vehicles"][0][attribute] = value
            self.assertEqual(SegmentDeadlockProofInspector(graph, regions).inspect(counterexample), [])
        # T 작업을 마친 적재차가 다시 주행하다 막힌 경우는 T 이름만으로
        # 검사에서 배제하지 않는다. 정상 8초 작업은 is_route_active=False.
        state["vehicles"][0]["transfer_node"] = "T1"
        self.assertTrue(SegmentDeadlockProofInspector(graph, regions).inspect(state))

    def test_overlapping_ring_names_and_unrelated_growth_are_not_types(self):
        graph, regions, state = self.make_state()
        for vehicle in state["vehicles"]:
            vehicle["physical_ring_numbers"] = [0, 1]
        proof_types = self.types(SegmentDeadlockProofInspector(graph, regions).inspect(state))
        self.assertNotIn("TYPE II", proof_types)
        self.assertNotIn("TYPE IV", proof_types)


if __name__ == "__main__":
    unittest.main()
