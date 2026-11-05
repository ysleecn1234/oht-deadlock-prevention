"""실제 가감속 함수와 salabim 사건을 사용하는 구간 주행 검증."""
from collections import defaultdict
import heapq
import math
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

import salabim

from experiment_support import EXPERIMENT_ROOT, load_simulator

sys.path.insert(0, str(EXPERIMENT_ROOT / "시뮬레이터사본"))
from segment_motion import SegmentMotion, MotionSafetyError, get_acceleration_limit, get_arrival_seconds


class SegmentMotionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = load_simulator(EXPERIMENT_ROOT / "입력자료" / "안전초기배치_통과구역검증" / "시드01", "segment")

    def make_model(self, coordinates, edges, ports=None):
        salabim.yieldless(False)
        environment = salabim.Environment(trace=False)
        simulator = SimpleNamespace(NODE_XY=coordinates, NODE_LIST=list(coordinates),
            LINK_A=[edge[0] for edge in edges], LINK_B=[edge[1] for edge in edges],
            LINK_LEN=[edge[2] for edge in edges], LINK_CAP=[edge[3] if len(edge)>3 else 3500 for edge in edges],
            OUT_LINKS={node:[] for node in coordinates}, IN_LINKS={node:[] for node in coordinates},
            EDGE_LINK_IDX={}, ADJ={node:[] for node in coordinates}, NODE_HOLDER=dict.fromkeys(coordinates),
            NODE_RINGS={}, RINGS=[], RING_MEMBERS=[], HELD_NODES={}, JUNCTION_SPEED={},
            ENABLE_RING_CONTROL=False, MERGE_CONTROL_MODE="FCFS", MERGE_CHAIN_OF_LINK={},
            MERGE_OUT_NODES={}, MERGE_ADMITTED={}, PORT_BY_ID=ports or {},
            OHT_LENGTH_MM=850, FOLLOW_CENTER_GAP_MM=1150, OHT_ACCEL=2000, OHT_DECEL=3000,
            OHT_MAX_SPEED=3500, BLOCKED_LINKS=set(), REROUTE_COUNTS=defaultdict(int),
            IDLE_STATS=defaultdict(int), OHT_IDLE="idle", JUNCTION_REROUTE_MIN_GAP=5,
            STALL_THRESHOLD_SECONDS=10, STALL_EVENTS=[], WAIT_COUNTS=defaultdict(int),
            RECORD_MERGE_PASS_LOG=True, MERGE_PASS_LOG=[],
            ENABLE_JUNCTION_REROUTE=True,
            PATH_RETRY_COOLDOWN=5, MERGE_REQUEST_POLL=.2, RING_WAIT_POLL=.5,
            YIELD_RETRY_INTERVAL=.2, MERGE_OUT_FULL_STOPPED_ONLY=True)
        simulator.graph_data={"links":[]}
        for index,edge in enumerate(edges):
            start,end,length_mm=edge[:3]
            simulator.OUT_LINKS[start].append(index)
            simulator.IN_LINKS[end].append(index)
            simulator.ADJ[start].append(end)
            simulator.EDGE_LINK_IDX[(start,end)]=index
            simulator.graph_data["links"].append({"id":str(index),"a":start,"b":end,"length":length_mm})
        simulator.JUNCTION_NODES={node for node in coordinates if len(simulator.OUT_LINKS[node])>=2}
        simulator.MERGE_POINT_NODES={node for node in coordinates if len(simulator.IN_LINKS[node])>1}
        simulator.LINK_FLOW=[0]*len(edges)
        simulator.LINK_STALL=[0.0]*len(edges)
        simulator.OHT_LIST=[]

        def shortest_path(start,goal,avoid=None,from_link=None):
            pending=[(0,start,[start])]; visited=set()
            while pending:
                distance,node,path=heapq.heappop(pending)
                if node==goal:return path
                if node in visited:continue
                visited.add(node)
                for index in simulator.OUT_LINKS[node]:
                    if index in (avoid or set()):continue
                    target=simulator.LINK_B[index]
                    heapq.heappush(pending,(distance+simulator.LINK_LEN[index],target,path+[target]))
            return None
        simulator.shortest_path=shortest_path
        simulator.path_travel_cumtime=lambda path:(0,[0]*len(path))
        simulator.OHT=self.source.OHT
        simulator.LINK_PENALTY=[1.0]*len(edges)
        simulator.OHT_LOAD_UNLOAD_TIME=8.0
        simulator.MERGE_STATS=defaultdict(int)
        simulator.merge_priority_link=lambda *arguments:None
        simulator.grant_merge_priority=lambda *arguments:None
        simulator._accumulate_region_stall=lambda *arguments:None
        simulator.merge_node_stat=lambda node:defaultdict(int)
        controller=SegmentMotion(simulator)
        actual_profile_distance=self.source.OHT._profile_distance

        class Vehicle(salabim.Component):
            _profile_distance=staticmethod(actual_profile_distance)
            def setup(self, park_node, goal, port_id=None, second_goal=None, service_seconds=0):
                self.node=park_node;self.goal=goal;self.port_id=port_id;self.second_goal=second_goal
                self.service_seconds=service_seconds;self.oht_id=len(simulator.OHT_LIST)
                self.total_distance=0.;self._cur_speed=0.;self._job_travel=0.;self._job_wait=0.
                self.total_stall_time=0.;self.total_lock_stall_time=0.;self.total_follow_stall_time=0.
                self._abort_leg=False;self._replan=False;self._avoid_links=None;self._ring_exempt=False
                self._ring_waiting=False;self._merge_admits={};self._merge_wait=None
                self._junction_reroute_node=None;self._last_reroute_t=-1e9;self._no_alt_node=None
                self._stall_start=None;self._stuck_since=None;self._stall_next=None;self._yield_request=False
                self._first_stall_t=None;self._stall_avoid_links=set()
                self.job=SimpleNamespace(state="carrying");self.state="to_dest"
                self.arrival=None;self.arrival_state=None
                controller.register(self);simulator.OHT_LIST.append(self)
            def _update_heading(self,*coordinates):pass
            def process(self):
                self.is_arrived = yield from controller.goto(self,self.goal,self.port_id)
                self.arrival=self.env.now();self.arrival_state=controller.get_position(self)
                if self.second_goal is not None:
                    yield self.hold(self.service_seconds)
                    yield from controller.goto(self,self.second_goal)
                yield self.hold(1000)
        return environment,simulator,controller,Vehicle

    def test_braking_before_update_ends_does_not_reverse_velocity(self):
        acceleration=get_acceleration_limit(113.272,113.272**2/6000,1,3000)
        self.assertAlmostEqual(acceleration,-3000,places=5)
        with self.assertRaises(MotionSafetyError):get_acceleration_limit(3500,100,0.1,3000)
        speed=2985.967182056191
        self.assertGreaterEqual(get_acceleration_limit(speed,speed**2/6000,6.8e-10,3000),-3000)
        self.assertGreater(get_arrival_seconds(2.247816155431792e-9,1073.312629206182,-3000),0)

    def test_real_transfer_position_and_followers_share_logical_node(self):
        coordinates={"P2":(-4000,0),"P1":(-2000,0),"A":(0,0),"B":(20000,0)}
        edges=[("P2","P1",2000),("P1","A",2000),("A","B",20000)]
        port={"port_id":"equipment","parent_link":"2","node":"B","rel_dist":10000,"transfer_node":"T1"}
        environment,simulator,controller,Vehicle=self.make_model(coordinates,edges,{"equipment":port})
        first=Vehicle(park_node="A",goal="B",port_id="equipment",second_goal="B",service_seconds=8)
        Vehicle(park_node="P1",goal="B")
        Vehicle(park_node="P2",goal="B")
        minimum_gap=math.inf;has_shared_node=False
        for sample in range(1,2201):
            environment.run(till=sample*.01)
            positions=sorted(controller.get_position(vehicle)["x_mm"] for vehicle in simulator.OHT_LIST)
            minimum_gap=min(minimum_gap,min(second-first for first,second in zip(positions,positions[1:])))
            has_shared_node |= all(vehicle.node=="A" for vehicle in simulator.OHT_LIST)
        self.assertTrue(has_shared_node)
        self.assertGreaterEqual(minimum_gap,1150-1e-6)
        self.assertAlmostEqual(first.arrival_state["x_mm"],10000,places=5)
        self.assertEqual(first.arrival_state["transfer_node"],"T1")
        self.assertLess(first.arrival_state["speed_mm_per_second"],1e-4)
        self.assertGreater(simulator.OHT_LIST[1].total_follow_stall_time,0)

    def test_short_links_and_curve_cap(self):
        coordinates={"A":(0,0),"C":(27,0),"D":(827,0),"B":(2427,0)}
        edges=[("A","C",27), ("C","D",800,700), ("D","B",1600)]
        port={"port_id":"port","parent_link":"2","node":"B","rel_dist":1000,"transfer_node":"T"}
        environment,simulator,controller,Vehicle=self.make_model(coordinates,edges,{"port":port})
        vehicle=Vehicle(park_node="A",goal="B",port_id="port")
        for sample in range(1,2001):
            environment.run(till=sample*.01)
            position=controller.get_position(vehicle)
            if position["link_index"]==1:self.assertLessEqual(position["speed_mm_per_second"],700+1e-6)
        self.assertAlmostEqual(vehicle.arrival_state["x_mm"],1827,places=5)
        self.assertEqual(vehicle.node,"D")

    def test_initial_686_mm_gap_is_rejected(self):
        environment,simulator,controller,Vehicle=self.make_model({"A":(0,0),"B":(686,0)},[("A","B",686)])
        Vehicle(park_node="A",goal="B")
        with self.assertRaises(MotionSafetyError):Vehicle(park_node="B",goal="A")

    def test_last_committed_position_does_not_reverse_actual_vehicle_order(self):
        environment,simulator,controller,Vehicle=self.make_model({"A":(0,0),"B":(20000,0)},[("A","B",20000)])
        vehicle=Vehicle(park_node="A",goal="B")
        environment.run(till=.5)
        self.assertGreater(controller.get_position(vehicle)["offset_mm"],vehicle._motion_offset_mm)
        self.assertEqual(controller.find_front(vehicle,[0])[1],None)

    def test_stop_at_guide_node_checks_front_in_the_following_link(self):
        coordinates={"A":(0,0),"G":(3000,0),"B":(9000,0)}
        port={"port_id":"port","parent_link":"1","node":"B","rel_dist":847,"transfer_node":"T"}
        environment,simulator,controller,Vehicle=self.make_model(coordinates,[("A","G",3000),("G","B",6000)],{"port":port})
        front=Vehicle(park_node="G",goal="B",port_id="port",second_goal="B",service_seconds=8)
        rear=Vehicle(park_node="A",goal="G")
        for sample in range(1,1501):
            environment.run(till=sample*.01)
            self.assertGreaterEqual(controller.get_position(front)["x_mm"]-controller.get_position(rear)["x_mm"],1150-1e-6)

    def test_rear_vehicle_cannot_reserve_a_control_point_before_front_vehicle(self):
        coordinates={"P":(-4000,0),"A":(-2000,0),"C":(0,0),"B":(6000,0),"D":(0,6000)}
        edges=[("P","A",2000),("A","C",2000),("C","B",6000),("C","D",6000)]
        environment,simulator,controller,Vehicle=self.make_model(coordinates,edges)
        rear=Vehicle(park_node="P",goal="B")
        front=Vehicle(park_node="A",goal="B")
        environment.run(till=.001)
        self.assertIs(simulator.NODE_HOLDER["C"],front)
        self.assertNotIn("C",controller.control_claims[rear])

    def test_initial_control_claim_releases_after_passing_control_point(self):
        coordinates={"A":(0,0),"C":(1124,0),"B":(8000,0),"D":(1124,8000)}
        environment,simulator,controller,Vehicle=self.make_model(coordinates,
            [("A","C",1124),("C","B",6876),("C","D",8000)])
        vehicle=Vehicle(park_node="A",goal="B")
        for sample in range(1,1001):
            environment.run(till=sample*.01)
            controller.check_safety()
            position=controller.get_position(vehicle)
            if position["x_mm"]<1124+425:
                self.assertIs(simulator.NODE_HOLDER["C"],vehicle)

    def test_assignment_abort_brakes_before_returning_to_idle(self):
        coordinates={"P":(-2000,0),"A":(0,0),"G":(4000,0),"B":(10000,0),"C":(20000,0)}
        edges=[("P","A",2000),("A","G",4000),("G","B",6000),("B","C",10000)]
        environment,simulator,controller,Vehicle=self.make_model(coordinates,edges)
        first=Vehicle(park_node="A",goal="C")
        Vehicle(park_node="P",goal="C")
        class AbortAssignment(salabim.Component):
            def process(self):
                yield self.hold(2.4)
                first._abort_leg=True
        AbortAssignment()
        previous_speed=0.0
        for sample in range(1,1001):
            environment.run(till=sample*.01)
            speed=controller.get_position(first)["speed_mm_per_second"]
            self.assertLessEqual(previous_speed-speed,30+1e-5)
            previous_speed=speed
            controller.check_safety()
        self.assertFalse(first.is_arrived)
        self.assertLess(controller.get_position(first)["speed_mm_per_second"],1e-5)

    def test_merge_near_port_protects_work_and_releases_before_next_node(self):
        coordinates={"L":(-5000,0),"R":(0,-5000),"M":(0,0),"B":(5000,0),"C":(10000,0)}
        edges=[("L","M",5000),("R","M",5000),("M","B",5000),("B","C",5000)]
        port={"port_id":"port","parent_link":"0","node":"M","rel_dist":4575,"transfer_node":"T"}
        environment,simulator,controller,Vehicle=self.make_model(coordinates,edges,{"port":port})
        first=Vehicle(park_node="L",goal="M",port_id="port",second_goal="C",service_seconds=8)
        second=Vehicle(park_node="R",goal="C")
        has_released_while_logically_at_merge=False
        for sample in range(1,2501):
            environment.run(till=sample*.01)
            controller.check_safety()
            has_released_while_logically_at_merge |= (first.node=="M" and simulator.NODE_HOLDER["M"] is not first)
        self.assertTrue(has_released_while_logically_at_merge)
        self.assertGreater(first.total_distance,5000)
        self.assertGreater(second.total_distance,5000)


if __name__=="__main__":unittest.main()
