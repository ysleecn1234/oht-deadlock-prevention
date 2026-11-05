"""합류 요청 순서·실제 정지 대기·늦은 후보 사진의 최초 정체 기준 검증."""
from collections import defaultdict, deque
import copy
import gzip
import json
from pathlib import Path
import tempfile
from types import FunctionType, SimpleNamespace
import unittest

import test_segment_motion as motion_test_support
from integrated_deadlock_observer import IntegratedDeadlockRecorder


class MergeModeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        motion_test_support.SegmentMotionTests.setUpClass()
        cls.source = motion_test_support.SegmentMotionTests.source

    def make_model(self):
        return motion_test_support.SegmentMotionTests().make_model(
            {"L":(-6000,0),"R":(0,-6000),"M":(0,0),"B":(12000,0)},
            [("L","M",6000),("R","M",6000),("M","B",12000)])

    def configure(self, simulator, mode):
        simulator.MERGE_CONTROL_MODE = mode
        simulator.MERGE_HELD_LINK = {}
        simulator.MERGE_HOLD_EXPIRE = {}
        simulator.MERGE_PRIORITY_WINDOW = 5
        simulator.MERGE_STATS = defaultdict(int)
        simulator.MERGE_NODE_STATS = defaultdict(lambda: defaultdict(float))
        simulator.merge_node_stat = lambda node: simulator.MERGE_NODE_STATS[node]
        simulator.merge_priority_link = FunctionType(self.source.merge_priority_link.__code__, vars(simulator))
        simulator.grant_merge_priority = FunctionType(self.source.grant_merge_priority.__code__, vars(simulator))
        simulator.merge_request = lambda *arguments: True

    def test_queue_selection_matches_original_fcfs_and_adaptive_release(self):
        for mode, held_link, expiry in (("FCFS",None,0),("ADAPTIVE",1,15),
                                       ("ADAPTIVE",1,9),("ADAPTIVE",99,15)):
            with self.subTest(mode=mode,held=held_link,expiry=expiry):
                environment,simulator,controller,Vehicle = self.make_model()
                self.configure(simulator, mode)
                simulator.MERGE_HELD_LINK["M"] = held_link
                simulator.MERGE_HOLD_EXPIRE["M"] = expiry
                activated = []
                class WaitingVehicle:
                    def __init__(self,node): self.node=node
                    def ispassive(self): return True
                    def activate(self): activated.append(self)
                first, second = WaitingVehicle("L"), WaitingVehicle("R")
                controller.merge_waiters["M"] = {first:[8,1,0,True],second:[9,2,1,True]}
                selected = controller.select_merge_waiter("M",10)
                holder = object()
                simulator.NODE_HOLDER["M"] = holder
                simulator.NODE_WAITERS = {"M":[first,second]}
                simulator._ring_on_release = lambda *arguments:None
                original_release = FunctionType(self.source.release.__code__,vars(simulator))
                original_release("M",holder,10)
                self.assertEqual(activated,[selected])

    def test_all_modes_keep_physical_separation_and_record_actual_wait(self):
        for mode in ("OCS","ADAPTIVE","FCFS"):
            with self.subTest(mode=mode):
                environment,simulator,controller,Vehicle = self.make_model()
                self.configure(simulator,mode)
                Vehicle(park_node="L",goal="B")
                Vehicle(park_node="R",goal="B")
                for sample in range(1,1601):
                    environment.run(till=sample*.01)
                    controller.check_safety()
                passes = simulator.MERGE_PASS_LOG
                self.assertEqual(len(passes),2)
                self.assertGreater(passes[1][4],0)
                self.assertAlmostEqual(sum(row[4] for row in passes),simulator.MERGE_NODE_STATS["M"]["node_wait"],places=8)
                self.assertFalse(controller.safety_failures)
                self.assertGreaterEqual(controller.minimum_observed_gap_mm,1150-1e-6)


class OnsetFrameTests(unittest.TestCase):
    def make_recorder(self, output):
        recorder = object.__new__(IntegratedDeadlockRecorder)
        recorder.output_directory = Path(output)
        (recorder.output_directory / "핵심상태").mkdir()
        recorder.saved_times = {}
        recorder.scene_buffer = deque(maxlen=302)
        return recorder

    def state(self, second, onset):
        return {"simulation_second":second,"occupancy_mode":"segment","vehicles":[
            {"oht_id":0,"_stuck_since":onset,"_stall_next":"M" if onset is not None else None}]}

    def compact(self, second):
        return {"simulation_second":second,"positions":[(0,"A","M",second,0,0,None,None,0,second)]}

    def test_late_photo_keeps_frames_before_wait_onset_after_ring_buffer_expires(self):
        with tempfile.TemporaryDirectory(prefix="oht-onset-test-") as output:
            recorder = self.make_recorder(output)
            for second in (100,280,340,400): recorder.scene_buffer.append(self.compact(second))
            recorder.retain_wait_onsets(self.state(400,400))
            context = recorder.get_event_onset(self.state(400,400),[0])
            for second in range(401,1001): recorder.scene_buffer.append(self.compact(second))
            recorder.retain_wait_onsets(self.state(1000,400))
            references = recorder.save_before_states(Path(output),self.state(1000,400),[0],context=context)
            self.assertEqual([item["actual_simulation_second"] for item in references],[100,280,340])
            for reference in references:
                self.assertEqual(reference["reference_simulation_second"],400)
                with gzip.open(Path(output)/reference["state_path"],"rt") as handle:
                    state = json.load(handle)
                self.assertEqual(state["vehicles"][0]["position_x_millimeters"],reference["actual_simulation_second"])
            recorder.retain_wait_onsets(self.state(1001,None))
            self.assertFalse(recorder.wait_onset_frames)
            self.assertEqual(context["frames"][300]["simulation_second"],100)

    def test_startup_missing_history_is_explicit_instead_of_using_later_scene(self):
        with tempfile.TemporaryDirectory(prefix="oht-onset-test-") as output:
            recorder = self.make_recorder(output)
            recorder.scene_buffer.append(self.compact(0))
            recorder.retain_wait_onsets(self.state(30,30))
            references = recorder.save_before_states(Path(output),self.state(600,30),[0])
            self.assertEqual(len(references),3)
            self.assertTrue(all(not reference["available"] for reference in references))
            self.assertTrue(all("state_path" not in reference for reference in references))

    def test_fractional_onset_uses_previous_sample_and_records_difference(self):
        with tempfile.TemporaryDirectory(prefix="oht-onset-test-") as output:
            recorder = self.make_recorder(output)
            for second in range(100,402): recorder.scene_buffer.append(self.compact(second))
            recorder.retain_wait_onsets(self.state(401,400.75))
            references = recorder.save_before_states(Path(output),self.state(1000,400.75),[0])
            self.assertEqual([item["actual_simulation_second"] for item in references],[100,280,340])
            self.assertTrue(all(item["observation_time_difference_seconds"]==.75 for item in references))


if __name__ == "__main__": unittest.main()
