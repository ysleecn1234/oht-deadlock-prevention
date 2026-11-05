"""저장된 상태만 읽어 전체 지도·사건 확대 지도를 생성. 시뮬레이터 불러오기 없음."""

import argparse
import gzip
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

from experiment_support import EXPERIMENT_ROOT, current_timestamp, file_sha256, write_json
from snapshot_storage import read_snapshot
from key_event_image_policy import POLICY_VERSION, select_key_snapshot


def font(size):
    for path in ("/System/Library/Fonts/AppleSDGothicNeo.ttc", "/System/Library/Fonts/Supplemental/Arial Unicode.ttf"):
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def draw_map(snapshot, graph, destination, title, event=None, cropped=False, status_caption=None):
    vehicles = snapshot["vehicles"]
    node_coordinates = graph["nodes"]
    focused_ids = set(event.get("vehicle_ids", ())) if event else set()
    focused_vehicles = [vehicle for vehicle in vehicles if vehicle["oht_id"] in focused_ids]
    if cropped and focused_vehicles:
        points = [(vehicle["position_x_millimeters"], vehicle["position_y_millimeters"])
                  for vehicle in focused_vehicles]
        for vehicle in focused_vehicles:
            if vehicle["_stall_next"] in node_coordinates:
                points.append(node_coordinates[vehicle["_stall_next"]])
    else:
        points = list(node_coordinates.values())
    minimum_x, maximum_x = min(point[0] for point in points), max(point[0] for point in points)
    minimum_y, maximum_y = min(point[1] for point in points), max(point[1] for point in points)
    margin = max(4000, (maximum_x - minimum_x + maximum_y - minimum_y) * 0.06)
    minimum_x -= margin
    maximum_x += margin
    minimum_y -= margin
    maximum_y += margin
    width = 2200 if not cropped else 1500
    height = 1000
    top_margin, bottom_margin = 120, 85
    scale = min((width - 80) / (maximum_x - minimum_x),
                (height - top_margin - bottom_margin) / (maximum_y - minimum_y))
    horizontal_offset = (width - (maximum_x - minimum_x) * scale) / 2

    def pixel(x, y):
        return horizontal_offset + (x - minimum_x) * scale, top_margin + (maximum_y - y) * scale

    image = Image.new("RGB", (width, height), "#ffffff")
    drawing = ImageDraw.Draw(image)
    drawing.text((35, 18), title, font=font(30), fill="#172b4d")
    simulation_second = snapshot["simulation_second"]
    drawing.text((35, 60), f"시뮬레이션 {simulation_second:,.1f}초 ({simulation_second / 3600:.2f}시간)  |  저장 상태 기반 재구성",
                 font=font(21), fill="#41536c")
    drawing.text((35, height - 62), "파랑: 이동·서비스·유휴    주황: 관측 대기    빨강: 60초 이상 대기    검정 테두리: 선택 사건 차량",
                 font=font(20), fill="#41536c")
    drawing.text((35, height - 35), status_caption or "선택 차량의 대기 관계는 후보 근거이며 영구 교착 확정을 뜻하지 않음", font=font(18), fill="#6b7280")
    for start, end in graph["edges"]:
        start_x, start_y = node_coordinates[start]
        end_x, end_y = node_coordinates[end]
        if max(start_x, end_x) < minimum_x or min(start_x, end_x) > maximum_x:
            continue
        if max(start_y, end_y) < minimum_y or min(start_y, end_y) > maximum_y:
            continue
        drawing.line([pixel(start_x, start_y), pixel(end_x, end_y)], fill="#cbd5e1", width=2)
    vehicle_by_id = {vehicle["oht_id"]: vehicle for vehicle in vehicles}
    if cropped:
        for dependency in snapshot["wait_dependencies"]:
            if dependency["oht_id"] not in focused_ids:
                continue
            waiting_vehicle = vehicle_by_id[dependency["oht_id"]]
            start = pixel(waiting_vehicle["position_x_millimeters"], waiting_vehicle["position_y_millimeters"])
            for blocker_id in dependency["blocking_vehicle_ids"]:
                blocker = vehicle_by_id[blocker_id]
                end = pixel(blocker["position_x_millimeters"], blocker["position_y_millimeters"])
                drawing.line([start, end], fill="#a21caf", width=3)
                # 화살표는 저장된 대기 방향을 표시. 좌표를 옮기거나 경로를 재계산하지 않음.
                import math
                angle = math.atan2(end[1] - start[1], end[0] - start[0])
                drawing.polygon([end, (end[0] - 11 * math.cos(angle - 0.45), end[1] - 11 * math.sin(angle - 0.45)),
                                 (end[0] - 11 * math.cos(angle + 0.45), end[1] - 11 * math.sin(angle + 0.45))], fill="#a21caf")
    for vehicle in vehicles:
        x, y = vehicle["position_x_millimeters"], vehicle["position_y_millimeters"]
        if not (minimum_x <= x <= maximum_x and minimum_y <= y <= maximum_y):
            continue
        center_x, center_y = pixel(x, y)
        waiting_duration = vehicle["waiting_duration_seconds"]
        color = "#dc2626" if waiting_duration >= 60 else "#f59e0b" if waiting_duration > 0 else "#2563eb"
        radius = 6 if cropped else 3
        outline = "#111827" if vehicle["oht_id"] in focused_ids else color
        drawing.ellipse((center_x - radius, center_y - radius, center_x + radius, center_y + radius), fill=color, outline=outline, width=2)
        if cropped and vehicle["oht_id"] in focused_ids:
            drawing.text((center_x + 7, center_y - 18), str(vehicle["oht_id"]), font=font(17), fill="#111827")
            if vehicle.get("transfer_node"):
                drawing.text((center_x + 7, center_y + 6), f"T {vehicle['transfer_node']} / {vehicle.get('port_id', '')}",
                             font=font(15), fill="#111827")
    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    image.save(destination)


def record_image_policy_application(output_directory):
    """사진 선별 코드가 바뀔 때 최초 처리 시각과 당시 저장 상태 지점을 보존."""
    code_files = (Path(__file__), Path(__file__).with_name("key_event_image_policy.py"))
    code_hashes = {path.name: file_sha256(path) for path in code_files}
    history_path = output_directory / "중간집계" / "이미지정책적용이력.json"
    history = json.loads(history_path.read_text()) if history_path.exists() else {"applications": []}
    if history["applications"] and history["applications"][-1]["code_sha256"] == code_hashes:
        return
    progress_path = output_directory / "중간집계" / "현재진행상태.json"
    progress = json.loads(progress_path.read_text()) if progress_path.exists() else {}
    latest_state = max((output_directory / "사건상태공유보관").glob("시각_*.json.gz"), default=None)
    history["applications"].append({
        "first_processing_at": current_timestamp(), "policy_version": POLICY_VERSION,
        "code_sha256": code_hashes,
        "latest_progress_report_at": progress.get("updated_at"),
        "latest_progress_report_second": progress.get("simulation_second"),
        "latest_saved_snapshot_file": latest_state.name if latest_state else None,
        "latest_saved_snapshot_second": float(latest_state.name.removeprefix("시각_").removesuffix("초.json.gz")) if latest_state else None,
        "scope": "사진 선별 처리 시점. 시뮬레이션 설정 변경 아님",
        "note": "시뮬레이터와 비동기 처리하므로 저장 상태 시각은 당시 최신 관측 지점이며 실제 실행 시각과 일치한다고 단정하지 않음. 과거 저장 상태에도 새 기준을 적용할 수 있음"})
    write_json(history_path, history)


def render_pending(output_directory, maximum_events=1):
    record_image_policy_application(output_directory)
    graph = json.loads((EXPERIMENT_ROOT / "입력자료" / "지도와설비" / "agv_graph.json").read_text())
    regions = json.loads((output_directory / "구간별시계열" / "지도구간목록.json").read_text())
    rendered = 0
    reviewed = 0
    for event_path in sorted((output_directory / "사건별기록").glob("*/사건정보.json")):
        directory = event_path.parent
        marker = directory / "이미지생성상태.json"
        event = json.loads(event_path.read_text())
        previous_marker = json.loads(marker.read_text()) if marker.exists() else {}
        # 이전에 생성한 이미지는 유지. 새 정책은 핵심 시점만 추가 생성.
        if previous_marker.get("key_event_image_generated"):
            continue
        if previous_marker.get("next_review_simulation_second", 0) > event["last_observed_second"]:
            continue
        references = json.loads((directory / "발생전후상태" / "상태파일목록.json").read_text())["paths_relative_to_experiment_directory"]
        if not references:
            continue
        key_path, reason = select_key_snapshot(output_directory, event, references, regions)
        reviewed += 1
        if key_path is None:
            previous_marker.update(policy_version=POLICY_VERSION, selection_reason=reason,
                next_review_simulation_second=event["last_observed_second"] + 120,
                state_records_preserved=True)
            write_json(marker, previous_marker)
            if reviewed >= max(4, maximum_events * 2):
                break
            continue
        selected = [("핵심상태", key_path)]
        for label, relative_path in selected:
            snapshot = read_snapshot(output_directory / relative_path)
            frame_event = event
            title = f"{output_directory.name} / 사건 {event['event_number']} / {label} / {frame_event['label']}"
            draw_map(snapshot, graph, directory / "전체지도" / f"{label}.png", title, frame_event)
            draw_map(snapshot, graph, directory / "구간확대지도" / f"{label}.png", title, frame_event, cropped=True)
            if label == "핵심상태":
                write_json(directory / "대기관계" / "핵심상태관계.json", snapshot["wait_dependencies"])
        previous_marker.update(generated=True, key_event_image_generated=True,
            policy_version=POLICY_VERSION, selection_reason=reason,
            final_frame_generated=False, snapshot_reference=key_path,
            note="발생 전 상태는 원자료로 보존. 핵심 포화·장기 순환만 이미지로 생성. 확정 교착 이미지는 별도 판정기에서 생성")
        write_json(marker, previous_marker)
        rendered += 1
        if rendered >= maximum_events:
            break
    return rendered


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-directory", type=Path, required=True)
    parser.add_argument("--maximum-events", type=int, default=1)
    parser.add_argument("--snapshot", type=Path)
    arguments = parser.parse_args()
    if arguments.snapshot:
        snapshot = read_snapshot(arguments.snapshot)
        graph = json.loads((EXPERIMENT_ROOT / "입력자료" / "지도와설비" / "agv_graph.json").read_text())
        draw_map(snapshot, graph, arguments.output_directory / "상태복원검증.png", "버전 2 헤드리스 상태 복원 검증")
    else:
        rendered_count = render_pending(arguments.output_directory, arguments.maximum_events)
        if rendered_count:
            print(f"사건 이미지 생성: {rendered_count}개", flush=True)


if __name__ == "__main__":
    main()
