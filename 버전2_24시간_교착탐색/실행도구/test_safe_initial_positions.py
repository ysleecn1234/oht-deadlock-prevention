"""초기 배치의 거리 경계·분기 연결·재현성·입력 보존 검증. 시뮬레이터 실행 없음."""

import hashlib
import heapq
import json
import math
from pathlib import Path
import random
import tempfile
import unittest
from unittest.mock import patch

from safe_initial_positions import (
    build_track_adjacency, find_closest_initial_pair, generate_safe_positions,
    prepare_safe_cases, read_safety_parameters, validate_initial_positions,
)


SAFETY_PARAMETERS = {"OHT_LENGTH_MM": 850.0, "OHT_STOP_GAP_MM": 300.0,
                     "FOLLOW_CENTER_GAP_MM": 1150.0}


def make_graph(links):
    node_ids = list(dict.fromkeys(node for start_node, end_node, _length_mm in links
                                 for node in (start_node, end_node)))
    return {"nodes": {node_id: [0.0, 0.0] for node_id in node_ids},
            "links": [{"a": start_node, "b": end_node, "length": length_mm}
                      for start_node, end_node, length_mm in links]}


class SafeInitialPositionsTests(unittest.TestCase):
    def test_distance_boundary_and_cross_link_distance(self):
        adjacency, controlled_nodes = build_track_adjacency(make_graph([
            ("A", "middle", 600), ("middle", "B", 550)]))
        report = validate_initial_positions(adjacency, controlled_nodes, ["A", "B"], SAFETY_PARAMETERS)
        self.assertEqual(report["minimum_center_distance_mm"], 1150)
        adjacency["middle"]["B"] = adjacency["B"]["middle"] = 549
        with self.assertRaisesRegex(ValueError, "안전거리 위반"):
            validate_initial_positions(adjacency, controlled_nodes, ["A", "B"], SAFETY_PARAMETERS)

    def test_merge_approaches_are_checked_without_requiring_a_directed_path(self):
        adjacency, controlled_nodes = build_track_adjacency(make_graph([
            ("left", "merge", 400), ("right", "merge", 400), ("merge", "exit", 2000)]))
        self.assertIn("merge", controlled_nodes)
        with self.assertRaisesRegex(ValueError, "안전거리 위반"):
            validate_initial_positions(adjacency, controlled_nodes, ["left", "right"], SAFETY_PARAMETERS)

    def test_unconnected_parallel_rails_do_not_use_screen_distance(self):
        adjacency, controlled_nodes = build_track_adjacency(make_graph([
            ("first_start", "first_end", 2000), ("second_start", "second_end", 2000)]))
        report = validate_initial_positions(adjacency, controlled_nodes,
                                            ["first_start", "second_start"], SAFETY_PARAMETERS)
        self.assertIsNone(report["minimum_center_distance_mm"])
        self.assertTrue(report["is_valid"])

    def test_seed_prefix_is_reproducible_without_changing_global_random(self):
        adjacency, _controlled_nodes = build_track_adjacency(make_graph([
            (str(number), str(number + 1), 600) for number in range(12)]))
        previous_random_state = random.getstate()
        first, _rejected = generate_safe_positions(adjacency, 4, 101, 1150)
        second, _rejected = generate_safe_positions(adjacency, 2, 101, 1150)
        self.assertEqual(first[:2], second)
        self.assertEqual(first, generate_safe_positions(adjacency, 4, 101, 1150)[0])
        self.assertEqual(previous_random_state, random.getstate())

    def test_insufficient_capacity_does_not_relax_distance(self):
        adjacency, _controlled_nodes = build_track_adjacency(make_graph([("A", "B", 686)]))
        with self.assertRaisesRegex(ValueError, "배치 가능한 대수 부족"):
            generate_safe_positions(adjacency, 2, 101, 1150)

    def test_duplicate_and_unknown_positions_are_rejected(self):
        adjacency, controlled_nodes = build_track_adjacency(make_graph([("A", "B", 2000)]))
        for positions in (["A", "A"], ["not_a_node"]):
            with self.assertRaises(ValueError):
                validate_initial_positions(adjacency, controlled_nodes, positions, SAFETY_PARAMETERS)

    def test_closest_pair_matches_independent_all_source_search(self):
        random_generator = random.Random(12)
        for _example_number in range(30):
            graph = make_graph([(str(number), str(number + 1), random_generator.randint(27, 2000))
                                for number in range(11)]
                               + [("2", "9", random_generator.randint(27, 2000))])
            adjacency, _controlled_nodes = build_track_adjacency(graph)
            positions = random_generator.sample(list(adjacency), 5)
            expected = math.inf
            for start_node in positions:
                distances = {start_node: 0.0}
                pending = [(0.0, start_node)]
                while pending:
                    distance_mm, node_id = heapq.heappop(pending)
                    if distance_mm != distances[node_id]:
                        continue
                    if node_id != start_node and node_id in positions:
                        expected = min(expected, distance_mm)
                    for target_node, length_mm in adjacency[node_id].items():
                        candidate = distance_mm + length_mm
                        if candidate < distances.get(target_node, math.inf):
                            distances[target_node] = candidate
                            heapq.heappush(pending, (candidate, target_node))
            self.assertEqual(find_closest_initial_pair(adjacency, positions)[0], expected)

    def test_source_parameters_are_read_without_import(self):
        with tempfile.TemporaryDirectory(prefix="initial_position_test_") as directory:
            source = Path(directory) / "agv_fab_sim.py"
            source.write_text("raise RuntimeError('must not execute')\n"
                              "OHT_LENGTH_MM = 850\nOHT_STOP_GAP_MM = 300\n"
                              "FOLLOW_CENTER_GAP_MM = OHT_LENGTH_MM + OHT_STOP_GAP_MM\n")
            self.assertEqual(read_safety_parameters(source), SAFETY_PARAMETERS)

    def test_input_copy_reuse_and_overwrite_protection(self):
        with tempfile.TemporaryDirectory(prefix="initial_position_test_") as directory:
            root = Path(directory)
            simulator_directory = root / "시뮬레이터사본"
            reference = root / "입력자료" / "시드별입력" / "시드01"
            simulator_directory.mkdir()
            reference.mkdir(parents=True)
            graph = make_graph([("A", "B", 686), ("B", "C", 2000), ("C", "D", 2000)])
            source = simulator_directory / "agv_fab_sim.py"
            source.write_text("OHT_LENGTH_MM = 850\nOHT_STOP_GAP_MM = 300\n"
                              "FOLLOW_CENTER_GAP_MM = OHT_LENGTH_MM + OHT_STOP_GAP_MM\n")
            files = {"agv_graph.json": graph, "equipment.json": [], "track_segments.json": {},
                     "section_regions.json": {}, "oht_lot_schedule.json": [{"keep_this_lot": True}],
                     "oht_park_positions.json": ["A", "B"]}
            for name, content in files.items():
                encoded = json.dumps(content, ensure_ascii=False).encode()
                (reference / name).write_bytes(encoded)
                if name != "oht_park_positions.json":
                    (simulator_directory / name).write_bytes(encoded)
            original_hashes = {name: hashlib.sha256((reference / name).read_bytes()).hexdigest()
                               for name in files}
            manifest = {"basis": "제공 버전 2", "case_number": 1,
                        "starting_position_random_seed": 101, "lot_schedule_random_seed": 201,
                        "idle_jitter_random_seed": 2, "lot_schedule_count": 1,
                        "demand_distribution": "제공 기본 분포",
                        "ordered_node_list_sha256": hashlib.sha256(json.dumps(list(graph["nodes"])).encode()).hexdigest(),
                        "input_file_sha256": original_hashes}
            (reference / "입력생성조건.json").write_text(json.dumps(manifest))
            with patch("safe_initial_positions.EXPERIMENT_ROOT", root), patch(
                    "safe_initial_positions.COPIED_SIMULATOR_DIRECTORY", simulator_directory), patch(
                    "safe_initial_positions.verify_provided_files"):
                reports = prepare_safe_cases([1], 3, lot_schedule_count=1)
                output = Path(reports[0]["directory"])
                self.assertEqual((output / "oht_lot_schedule.json").read_bytes(),
                                 (reference / "oht_lot_schedule.json").read_bytes())
                self.assertTrue(prepare_safe_cases([1], 3)[0]["is_reused"])
                with self.assertRaisesRegex(ValueError, "조건이 다름"):
                    prepare_safe_cases([1], 2)
                with self.assertRaisesRegex(ValueError, "기존 시드별 입력"):
                    prepare_safe_cases([1], 3, root / "입력자료" / "시드별입력")
                with self.assertRaisesRegex(ValueError, "LOT 길이 변경"):
                    prepare_safe_cases([1], 3, lot_schedule_count=2)
                (output / "oht_park_positions.json").write_text('["A", "B"]')
                with self.assertRaisesRegex(ValueError, "출력 파일이 변경"):
                    prepare_safe_cases([1], 3)
            self.assertEqual(original_hashes,
                             {name: hashlib.sha256((reference / name).read_bytes()).hexdigest()
                              for name in files})


if __name__ == "__main__":
    unittest.main()
