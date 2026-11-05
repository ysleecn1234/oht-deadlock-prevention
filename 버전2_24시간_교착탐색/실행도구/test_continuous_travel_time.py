"""실제 부분 링크·T 정차·가감속 시간과 속도 상태 최단경로를 대조."""
import math
import unittest
from types import SimpleNamespace

import test_segment_motion as motion_test_support
from continuous_travel_time import get_travel_plan, get_segment_profile, TravelTimeModelError


class ContinuousTravelTimeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        motion_test_support.SegmentMotionTests.setUpClass()

    def make_model(self, *arguments):
        return motion_test_support.SegmentMotionTests().make_model(*arguments)

    def test_partial_link_time_matches_original_acceleration_profile(self):
        environment, simulator, controller, Vehicle = self.make_model(
            {"A": (0, 0), "B": (20000, 0)}, [("A", "B", 20000)])
        actual = get_travel_plan(simulator, [0], 6000, 1200, 10000)
        expected = simulator.OHT._segment_profile(1200, 0, 3500, 4000)
        self.assertAlmostEqual(actual["duration_seconds"], expected["dur"], places=9)
        with self.assertRaises(TravelTimeModelError):
            get_travel_plan(simulator, [0], 9900, 3500, 10000)

    def test_tiny_remaining_distance_keeps_speed_continuous_without_artificial_millisecond(self):
        source = motion_test_support.SegmentMotionTests.source
        profile = get_segment_profile(source,1073.312629206182,1073.3126292103705,3500,2.247816155431792e-9)
        self.assertEqual(profile["v0"],1073.312629206182)
        self.assertGreater(profile["dur"],0)
        self.assertLess(profile["dur"],1e-9)
        environment,simulator,controller,Vehicle = self.make_model(
            {"A":(0,0),"B":(2000,0)},[("A","B",2000)])
        plan = get_travel_plan(simulator,[0],2000,.001,2000)
        self.assertAlmostEqual(plan["duration_seconds"],.001/3000,places=12)
        self.assertEqual(plan["profiles"][-1]["v_end"],0)

    def test_full_map_speed_rounding_does_not_remove_the_only_forward_path(self):
        source = motion_test_support.SegmentMotionTests.source
        controller = source.get_segment_motion()
        origin={"node_id":"016527","link_index":None,"offset_mm":0.,"speed_mm_per_second":0.,"fixed_links":()}
        target=("port","STB1368")
        route = controller.travel_times.find_route(origin,target)
        self.assertIsNotNone(route)
        expected = controller.calculate_route_profile(route,0.,0.,float(source.PORT_BY_ID[target[1]]["rel_dist"]))["weighted_seconds"]
        self.assertAlmostEqual(controller.travel_times.get_travel_time_matrix([origin],[target])[0,0],expected,places=6)

    def test_t_stop_braking_propagates_into_previous_short_links(self):
        environment, simulator, controller, Vehicle = self.make_model(
            {"A":(0,0),"B":(2000,0),"C":(2027,0),"D":(4000,0)},
            [("A","B",2000),("B","C",27),("C","D",1973)])
        plan = get_travel_plan(simulator, [0,1,2], 0, 2500, 100)
        self.assertLessEqual(plan["profiles"][1]["v_end"] ** 2, 2 * 3000 * 100 + 1e-5)
        self.assertLess(plan["profiles"][-1]["v_end"], 1e-6)

    def test_new_free_driving_and_time_calculation_use_same_profiles(self):
        port = {"port_id":"port","parent_link":"2","rel_dist":1000,"transfer_node":"T","node":"B"}
        environment, simulator, controller, Vehicle = self.make_model(
            {"A":(0,0),"C":(27,0),"D":(827,0),"B":(2427,0)},
            [("A","C",27),("C","D",800,700),("D","B",1600)], {"port":port})
        expected = get_travel_plan(simulator,[0,1,2],0,0,1000)["duration_seconds"]
        vehicle = Vehicle(park_node="A",goal="B",port_id="port")
        environment.run(till=20)
        self.assertAlmostEqual(vehicle.arrival, expected, places=5)

    def test_matrix_and_route_match_independent_route_enumeration(self):
        coordinates={"A":(0,0),"B":(2000,0),"C":(0,2000),"D":(4000,0),"E":(6000,0)}
        edges=[("A","B",2000),("A","C",2000,700),("B","D",2000),
               ("C","D",2000),("D","E",2000),("E","A",6000)]
        port={"port_id":"port","parent_link":"4","rel_dist":100,"transfer_node":"T","node":"E"}
        environment, simulator, controller, Vehicle=self.make_model(coordinates,edges,{"port":port})
        origin={"node_id":"A","link_index":None,"offset_mm":0.,"speed_mm_per_second":0.,"fixed_links":()}
        candidates=[[0,2,4],[1,3,4]]
        expected=min(get_travel_plan(simulator,path,0,0,100)["duration_seconds"] for path in candidates)
        matrix=controller.travel_times.get_travel_time_matrix([origin],[("port","port")])
        route=controller.travel_times.find_route(origin,("port","port"))
        self.assertAlmostEqual(matrix[0,0],expected,places=6)
        self.assertAlmostEqual(get_travel_plan(simulator,route,0,0,100)["duration_seconds"],expected,places=6)
        simulator.BLOCKED_LINKS.add(0)
        alternate=controller.travel_times.find_route(origin,("port","port"))
        self.assertEqual(alternate,[1,3,4])

    def test_passed_port_requires_forward_loop_instead_of_reverse(self):
        coordinates={"A":(0,0),"B":(2000,0),"C":(2000,2000),"D":(0,2000)}
        edges=[("A","B",2000),("B","C",2000),("C","D",2000),("D","A",2000)]
        port={"port_id":"port","parent_link":"0","rel_dist":1000,"transfer_node":"T","node":"B"}
        environment,simulator,controller,Vehicle=self.make_model(coordinates,edges,{"port":port})
        origin={"node_id":"A","link_index":0,"offset_mm":1500.,"speed_mm_per_second":3000.,"fixed_links":()}
        route=controller.travel_times.find_route(origin,("port","port"))
        self.assertEqual(route,[0,1,2,3,0])
        expected=get_travel_plan(simulator,route,1500,3000,1000)["duration_seconds"]
        self.assertAlmostEqual(controller.travel_times.get_travel_time_matrix([origin],[("port","port")])[0,0],expected,places=6)

    def test_assignment_uses_current_position_and_future_actual_drop_port(self):
        ports={name:{"port_id":name,"parent_link":"0","rel_dist":offset,"transfer_node":name,"node":"B"}
               for name,offset in (("drop",10000),("pickup",14000))}
        environment,simulator,controller,Vehicle=self.make_model(
            {"A":(0,0),"B":(20000,0)},[("A","B",20000)],ports)
        simulator.OHT_TO_DEST="to_dest"
        vehicle=Vehicle(park_node="A",goal="B",port_id="drop")
        environment.run(till=.5)
        vehicle.state="to_pickup"
        current=controller.get_origin_location(vehicle,for_assignment=True)
        self.assertGreater(current["offset_mm"],0)
        self.assertLess(current["offset_mm"],10000)
        vehicle.state="to_dest"
        vehicle.job=SimpleNamespace(to_port="drop",to_node="B")
        future=controller.get_origin_location(vehicle,for_assignment=True)
        self.assertEqual(future["offset_mm"],10000)
        self.assertEqual(future["speed_mm_per_second"],0)
        expected=simulator.OHT._segment_profile(0,0,3500,4000)["dur"]
        self.assertAlmostEqual(controller.travel_times.get_travel_time_matrix([future],[("port","pickup")])[0,0],expected,places=7)

    def test_merge_arrival_uses_acceleration_instead_of_distance_divided_by_speed(self):
        environment,simulator,controller,Vehicle=self.make_model(
            {"A":(0,0),"M":(5000,0),"B":(10000,0)},[("A","M",5000),("M","B",5000)])
        vehicle=Vehicle(park_node="A",goal="B")
        controller.set_route(vehicle,[0,1])
        expected=simulator.OHT._segment_profile(0,3500,3500,5000)["dur"]
        self.assertAlmostEqual(controller.get_merge_arrival_seconds(vehicle,[0,1],0,None,None),expected,places=8)


if __name__=="__main__":unittest.main()
