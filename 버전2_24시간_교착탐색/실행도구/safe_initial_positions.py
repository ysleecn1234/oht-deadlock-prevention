"""제공 버전2를 실행하지 않고 안전 조건을 적용한 초기 배치만 생성한다.

[2026-11-06 초기 배치] 주행 방향이 아직 정해지지 않은 정지 차량은 연결
레일을 양방향으로 보아 중심 간격을 검사한다. 합류 직전의 서로 다른 레일도
보수적으로 검사하기 위함이며, 시뮬레이터에서 역주행을 허용하는 뜻이 아니다.
서로 연결되지 않은 평행 레일 사이의 화면상 직선거리는 사용하지 않는다.
시드 101~103과 기존 LOT 입력은 유지하고 후보 위치만 안전 조건으로 거른다.
이 검사는 지도 노드 중심과 차체 길이에 기반하며 실물 전환기 형상을 인증하지 않는다.
"""

import ast
from collections import Counter
from datetime import datetime
import hashlib
import heapq
import json
import math
from pathlib import Path
import random
import shutil
import tempfile
from zoneinfo import ZoneInfo

from experiment_support import (
    COPIED_SIMULATOR_DIRECTORY, EXPERIMENT_ROOT, MAP_FILE_NAMES,
    file_sha256, verify_provided_files,
)

GENERATION_RULE_VERSION = "2026-11-06-track-distance-and-control-clearance-2"
COMPARISON_VEHICLE_COUNTS = (500, 750, 1000, 1250, 1500)


def read_safety_parameters(simulator_path):
    """제공 파일의 기존 변수명을 읽는다. 시뮬레이터 import는 하지 않는다."""
    syntax_tree = ast.parse(Path(simulator_path).read_text(encoding="utf-8"))
    parameters = {}
    required_names = {"OHT_LENGTH_MM", "OHT_STOP_GAP_MM", "FOLLOW_CENTER_GAP_MM"}
    for statement in syntax_tree.body:
        if not isinstance(statement, ast.Assign):
            continue
        names = [target.id for target in statement.targets if isinstance(target, ast.Name)]
        for name in names:
            if name not in required_names:
                continue
            expression = statement.value
            if isinstance(expression, ast.Constant) and type(expression.value) in (int, float):
                parameters[name] = float(expression.value)
            elif (isinstance(expression, ast.BinOp) and isinstance(expression.op, ast.Add)
                  and isinstance(expression.left, ast.Name)
                  and isinstance(expression.right, ast.Name)):
                parameters[name] = (parameters[expression.left.id]
                                    + parameters[expression.right.id])
            else:
                raise ValueError(f"안전 사양을 정적으로 읽을 수 없음: {name}")
    if set(parameters) != required_names:
        raise ValueError("기존 안전거리 변수 일부가 없음")
    if any(not math.isfinite(value) or value <= 0 for value in parameters.values()):
        raise ValueError("안전 사양은 유한한 양수여야 함")
    if parameters["FOLLOW_CENTER_GAP_MM"] != (parameters["OHT_LENGTH_MM"]
                                               + parameters["OHT_STOP_GAP_MM"]):
        raise ValueError("차체 길이 + 정지 여유와 중심 간격이 불일치")
    return parameters


def build_track_adjacency(graph):
    """기존 nodes 삽입 순서를 유지하고 초기 배치 검사용 양방향 연결을 만든다."""
    adjacency = {node_id: {} for node_id in graph["nodes"]}
    outgoing_count = Counter()
    incoming_count = Counter()
    for link in graph["links"]:
        start_node = link["a"]
        end_node = link["b"]
        length_mm = float(link["length"])
        if start_node not in adjacency or end_node not in adjacency:
            raise ValueError("지도에 없는 노드를 연결하는 링크")
        if not math.isfinite(length_mm) or length_mm <= 0:
            raise ValueError("링크 길이는 유한한 양수여야 함")
        outgoing_count[start_node] += 1
        incoming_count[end_node] += 1
        for origin, target in ((start_node, end_node), (end_node, start_node)):
            adjacency[origin][target] = min(length_mm,
                                             adjacency[origin].get(target, math.inf))
    controlled_nodes = {node_id for node_id in adjacency
                        if outgoing_count[node_id] >= 2 or incoming_count[node_id] > 1}
    return adjacency, controlled_nodes


def find_nearby_nodes(adjacency, start_node, maximum_distance_mm, is_boundary_included=False):
    distances = {start_node: 0.0}
    pending = [(0.0, start_node)]
    while pending:
        distance_mm, node_id = heapq.heappop(pending)
        if distance_mm != distances[node_id]:
            continue
        for target_node, length_mm in adjacency[node_id].items():
            candidate_distance_mm = distance_mm + length_mm
            is_inside = (candidate_distance_mm <= maximum_distance_mm if is_boundary_included
                         else candidate_distance_mm < maximum_distance_mm)
            if is_inside and candidate_distance_mm < distances.get(target_node, math.inf):
                distances[target_node] = candidate_distance_mm
                heapq.heappush(pending, (candidate_distance_mm, target_node))
    return distances


def generate_safe_positions(adjacency, vehicle_count, starting_position_random_seed,
                            minimum_center_distance_mm, controlled_nodes=None):
    if vehicle_count < 1 or vehicle_count > len(adjacency):
        raise ValueError("차량 수가 지도 노드 수 범위를 벗어남")
    if not math.isfinite(minimum_center_distance_mm) or minimum_center_distance_mm <= 0:
        raise ValueError("최소 간격은 유한한 양수여야 함")
    random_generator = random.Random(starting_position_random_seed)
    candidate_order = random_generator.sample(list(adjacency), len(adjacency))
    positions = []
    excluded_nodes = set()
    occupied_control_points = set()
    controlled_nodes = set(controlled_nodes or ())
    rejected_candidate_count = 0
    for candidate_node in candidate_order:
        if candidate_node in excluded_nodes:
            rejected_candidate_count += 1
            continue
        nearby_nodes = find_nearby_nodes(adjacency, candidate_node, minimum_center_distance_mm)
        protected_points = controlled_nodes.intersection(nearby_nodes)
        if occupied_control_points.intersection(protected_points):
            rejected_candidate_count += 1
            continue
        positions.append(candidate_node)
        excluded_nodes.update(nearby_nodes)
        occupied_control_points.update(protected_points)
        if len(positions) == vehicle_count:
            return positions, rejected_candidate_count
    raise ValueError(f"안전 조건으로 배치 가능한 대수 부족: {len(positions)} / {vehicle_count}")


def find_closest_initial_pair(adjacency, positions):
    """생성기의 배제 목록 대신 독립된 다중 시작점 최단거리로 최근접 쌍을 검사한다."""
    distances = {node_id: 0.0 for node_id in positions}
    nearest_origins = {node_id: node_id for node_id in positions}
    pending = [(0.0, node_id, node_id) for node_id in positions]
    heapq.heapify(pending)
    while pending:
        distance_mm, origin_node, node_id = heapq.heappop(pending)
        if (distance_mm, origin_node) != (distances[node_id], nearest_origins[node_id]):
            continue
        for target_node, length_mm in adjacency[node_id].items():
            candidate = (distance_mm + length_mm, origin_node)
            previous = (distances.get(target_node, math.inf),
                        nearest_origins.get(target_node, origin_node))
            if candidate < previous:
                distances[target_node], nearest_origins[target_node] = candidate
                heapq.heappush(pending, (*candidate, target_node))
    closest_distance_mm = math.inf
    closest_pair = None
    for node_id, neighbors in adjacency.items():
        for target_node, length_mm in neighbors.items():
            origin = nearest_origins.get(node_id)
            target_origin = nearest_origins.get(target_node)
            if origin is None or target_origin is None or origin == target_origin:
                continue
            candidate_distance_mm = distances[node_id] + length_mm + distances[target_node]
            if candidate_distance_mm < closest_distance_mm:
                closest_distance_mm = candidate_distance_mm
                closest_pair = [origin, target_origin]
    return (None if closest_pair is None else closest_distance_mm), closest_pair


def validate_initial_positions(adjacency, controlled_nodes, positions, safety_parameters):
    if not positions or len(set(positions)) != len(positions):
        raise ValueError("빈 배치 또는 중복 위치")
    unknown_nodes = set(positions) - adjacency.keys()
    if unknown_nodes:
        raise ValueError(f"지도에 없는 배치 위치: {sorted(unknown_nodes)}")
    minimum_distance_mm, closest_pair = find_closest_initial_pair(adjacency, positions)
    if (minimum_distance_mm is not None
            and minimum_distance_mm < safety_parameters["FOLLOW_CENTER_GAP_MM"]):
        raise ValueError(f"초기 안전거리 위반: {closest_pair}, {minimum_distance_mm}mm")
    # [2026-11-06] 합류 후 앞차와 1,150mm를 확보하려면 다른 진입측도
    # 제어점 전 1,150mm에서 기다려야 한다. 초기 배치도 같은 통과 보호 구역을 쓴다.
    controlled_occupancy = Counter()
    for node_id in positions:
        body_nodes = find_nearby_nodes(adjacency, node_id,
                                      safety_parameters["FOLLOW_CENTER_GAP_MM"])
        controlled_occupancy.update(controlled_nodes.intersection(body_nodes))
    multiple_occupancy = {node_id: count for node_id, count in controlled_occupancy.items()
                          if count > 1}
    if multiple_occupancy:
        raise ValueError(f"분기·합류 통과 보호 구역의 초기 점유 중복: {multiple_occupancy}")
    return {
        "is_valid": True,
        "vehicle_count": len(positions),
        "minimum_center_distance_mm": minimum_distance_mm,
        "closest_node_pair": closest_pair,
        "required_center_distance_mm": safety_parameters["FOLLOW_CENTER_GAP_MM"],
        "duplicate_position_count": 0,
        "controlled_point_multiple_occupancy_count": 0,
        "controlled_point_maximum_occupancy": max(controlled_occupancy.values(), default=0),
        "distance_rule": "연결 레일의 양방향 최단거리",
        "scope": "초기 정지 배치만 검사. 실제 전환기 구역 형상·주행 중 안전 검증은 별도",
    }


def write_new_json(path, payload):
    with Path(path).open("x", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2, allow_nan=False)
        handle.write("\n")


def prepare_safe_cases(case_numbers, maximum_vehicle_count=1500, output_directory=None,
                       lot_schedule_count=None):
    """기존 LOT·지도는 바이트 사본으로 보존하고 새 입력 폴더에 배치만 바꾼다."""
    verify_provided_files()
    case_numbers = list(case_numbers)
    if not case_numbers or len(set(case_numbers)) != len(case_numbers):
        raise ValueError("시드별 입력 번호는 비어 있거나 중복될 수 없음")
    source_path = COPIED_SIMULATOR_DIRECTORY / "agv_fab_sim.py"
    safety_parameters = read_safety_parameters(source_path)
    graph = json.loads((COPIED_SIMULATOR_DIRECTORY / "agv_graph.json").read_text(encoding="utf-8"))
    adjacency, controlled_nodes = build_track_adjacency(graph)
    ordered_node_list_sha256 = hashlib.sha256(json.dumps(list(adjacency)).encode()).hexdigest()
    output_directory = Path(output_directory or EXPERIMENT_ROOT / "입력자료" / "안전초기배치_통과구역검증").resolve()
    source_root = EXPERIMENT_ROOT / "입력자료" / "시드별입력"
    input_root = (EXPERIMENT_ROOT / "입력자료").resolve()
    if input_root not in output_directory.parents:
        raise ValueError("새 배치 출력은 이 프로젝트의 입력자료 하위에서만 허용")
    if output_directory == source_root.resolve() or source_root.resolve() in output_directory.parents:
        raise ValueError("기존 시드별 입력 폴더에는 출력할 수 없음")
    common_map_directory = (input_root / "지도와설비").resolve()
    if output_directory == common_map_directory or common_map_directory in output_directory.parents:
        raise ValueError("기존 공통 지도 폴더에는 출력할 수 없음")
    output_directory.mkdir(parents=True, exist_ok=True)
    reports = []
    for case_number in case_numbers:
        reference_directory = source_root / f"시드{case_number:02d}"
        reference_manifest = json.loads((reference_directory / "입력생성조건.json").read_text(encoding="utf-8"))
        if lot_schedule_count is not None and reference_manifest["lot_schedule_count"] != lot_schedule_count:
            raise ValueError("안전 배치 생성은 기존 LOT 입력을 그대로 사용함. LOT 길이 변경은 별도 작업")
        if reference_manifest["ordered_node_list_sha256"] != ordered_node_list_sha256:
            raise ValueError("기존 입력의 지도 노드 순서와 불일치")
        for name, expected in reference_manifest["input_file_sha256"].items():
            if file_sha256(reference_directory / name) != expected:
                raise ValueError(f"기존 시드 입력 해시 불일치: {name}")
        for name in MAP_FILE_NAMES:
            if file_sha256(reference_directory / name) != file_sha256(COPIED_SIMULATOR_DIRECTORY / name):
                raise ValueError(f"기준본과 시드 입력 지도가 다름: {name}")
        positions, rejected_count = generate_safe_positions(
            adjacency, maximum_vehicle_count,
            reference_manifest["starting_position_random_seed"],
            safety_parameters["FOLLOW_CENTER_GAP_MM"], controlled_nodes)
        validations = [validate_initial_positions(adjacency, controlled_nodes, positions[:count], safety_parameters)
                       for count in sorted(set(COMPARISON_VEHICLE_COUNTS + (maximum_vehicle_count,)))
                       if count <= maximum_vehicle_count]
        case_directory = output_directory / f"시드{case_number:02d}"
        generator_sha256 = file_sha256(__file__)
        if case_directory.exists():
            previous = json.loads((case_directory / "입력생성조건.json").read_text(encoding="utf-8"))
            if (previous.get("generation_rule_version") != GENERATION_RULE_VERSION
                    or previous.get("safety_parameters") != safety_parameters
                    or previous.get("park_position_count") != maximum_vehicle_count
                    or previous.get("starting_position_random_seed") != reference_manifest["starting_position_random_seed"]
                    or previous.get("reference_input_file_sha256") != reference_manifest["input_file_sha256"]):
                raise ValueError(f"기존 출력 조건이 다름. 새 출력 폴더를 지정해야 함: {case_directory}")
            for name, expected in previous["input_file_sha256"].items():
                if file_sha256(case_directory / name) != expected:
                    raise ValueError(f"기존 출력 파일이 변경됨: {name}")
            if json.loads((case_directory / "oht_park_positions.json").read_text()) != positions:
                raise ValueError("같은 시드로 재생성한 배치가 기존 출력과 다름")
            if json.loads((case_directory / "배치검증.json").read_text()) != validations:
                raise ValueError("기존 배치 검증과 현재 검증 결과가 다름")
            # 원 생성기의 해시를 덮어쓰지 않는다. 코드 정리 이후에도 동일한
            # 입력 조건·재생성 배치·검증 결과가 일치할 때만 읽기 전용 재사용한다.
            reports.append({"case_number": case_number, "directory": str(case_directory),
                            "validations": validations, "is_reused": True})
            continue
        staging_directory = Path(tempfile.mkdtemp(prefix=f"시드{case_number:02d}_생성중_", dir=output_directory))
        try:
            for name in (*MAP_FILE_NAMES, "oht_lot_schedule.json"):
                shutil.copy2(reference_directory / name, staging_directory / name)
            write_new_json(staging_directory / "oht_park_positions.json", positions)
            # 저장한 파일을 다시 읽어 검증한다. LOT는 생성하거나 순서를 바꾸지 않는다.
            restored = json.loads((staging_directory / "oht_park_positions.json").read_text())
            validate_initial_positions(adjacency, controlled_nodes, restored, safety_parameters)
            input_hashes = {name: file_sha256(staging_directory / name)
                            for name in (*MAP_FILE_NAMES, "oht_lot_schedule.json", "oht_park_positions.json")}
            if any(input_hashes[name] != reference_manifest["input_file_sha256"][name]
                   for name in (*MAP_FILE_NAMES, "oht_lot_schedule.json")):
                raise ValueError("지도 또는 LOT 복사본이 기존 입력과 다름")
            manifest = {
                **{name: reference_manifest[name] for name in (
                    "basis", "case_number", "starting_position_random_seed", "lot_schedule_random_seed",
                    "idle_jitter_random_seed", "lot_schedule_count", "demand_distribution")},
                "created_at": datetime.now(ZoneInfo("Asia/Seoul")).isoformat(timespec="seconds"),
                "generation_rule_version": GENERATION_RULE_VERSION,
                "generator_sha256": generator_sha256,
                "simulator_sha256": file_sha256(source_path),
                "reference_input_directory": str(reference_directory.resolve()),
                "reference_input_file_sha256": reference_manifest["input_file_sha256"],
                "ordered_node_list_sha256": ordered_node_list_sha256,
                "park_position_count": maximum_vehicle_count,
                "safety_parameters": safety_parameters,
                "rejected_candidate_count": rejected_count,
                "prefix_rule": "대수별로 동일한 최대 배치의 앞 N개 사용",
                "input_file_sha256": input_hashes,
            }
            write_new_json(staging_directory / "입력생성조건.json", manifest)
            write_new_json(staging_directory / "배치검증.json", validations)
            staging_directory.rename(case_directory)
        finally:
            if staging_directory.exists():
                shutil.rmtree(staging_directory)
        reports.append({"case_number": case_number, "directory": str(case_directory),
                        "validations": validations, "is_reused": False})
    verify_provided_files()
    for case_number in case_numbers:
        reference_directory = source_root / f"시드{case_number:02d}"
        reference_manifest = json.loads((reference_directory / "입력생성조건.json").read_text(encoding="utf-8"))
        for name, expected in reference_manifest["input_file_sha256"].items():
            if file_sha256(reference_directory / name) != expected:
                raise ValueError(f"생성 도중 기존 입력이 변경됨: {reference_directory / name}")
    return reports
