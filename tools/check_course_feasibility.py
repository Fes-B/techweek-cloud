"""Static geometric feasibility check for a benchmark course.

Reads the static axis-aligned ``Solid`` boxes of a Webots world and the
``WAYPOINTS`` tuple of its benchmark controller, then reports for every
waypoint segment whether a robot with the planner's collision radius plus the
static clearance margin can

* reach a point inside the waypoint's acceptance radius, and
* travel from the previous waypoint's reachable region to it (grid search in
  configuration space), compared with the length of the straight segment.

It never edits the world or the waypoints; it only explains whether a failure
at a given waypoint is an avoidance failure or a course-geometry limit.
Moving actors (``Robot`` nodes) are ignored.
"""

import argparse
import ast
from collections import deque
import heapq
import json
import math
from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from config import DWA_ROBOT_RADIUS, DWA_STATIC_CLEARANCE_MARGIN


def static_boxes(world_path):
    """Return (name, cx, cy, sx, sy) for top-level Solids with Box bounds."""
    text = Path(world_path).read_text(encoding="utf-8")
    boxes = []
    for match in re.finditer(r"^Solid \{(.*?)^\}", text, re.S | re.M):
        body = match.group(1)
        if re.search(r"^\s*rotation\s", body, re.M):
            raise ValueError("rotated static solids are not supported")
        translation = re.search(r"translation\s+(\S+)\s+(\S+)\s+(\S+)", body)
        name = re.search(r'name\s+"([^"]*)"', body)
        size = re.search(r"boundingObject\s+Box\s*\{\s*size\s+(\S+)\s+(\S+)\s+(\S+)", body)
        if not translation or not size:
            continue
        cz = float(translation.group(3))
        sz = float(size.group(3))
        if cz + sz / 2.0 <= 0.0:
            continue  # floor
        boxes.append((
            name.group(1) if name else "?",
            float(translation.group(1)), float(translation.group(2)),
            float(size.group(1)), float(size.group(2)),
        ))
    return boxes


def controller_waypoints(controller_path, variable="WAYPOINTS"):
    tree = ast.parse(Path(controller_path).read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == variable
            for target in node.targets
        ):
            return [tuple(point) for point in ast.literal_eval(node.value)]
    raise ValueError(f"{variable} not found in {controller_path}")


def box_distance(point, box):
    _, cx, cy, sx, sy = box
    dx = max(abs(point[0] - cx) - sx / 2.0, 0.0)
    dy = max(abs(point[1] - cy) - sy / 2.0, 0.0)
    return math.hypot(dx, dy)


def clearance(point, boxes):
    return min((box_distance(point, box) for box in boxes), default=float("inf"))


def nearest_box(point, boxes):
    return min(boxes, key=lambda box: box_distance(point, box))


class Grid:
    def __init__(self, boxes, required, resolution=0.02, points=(), padding=1.5):
        xs = [b[1] - b[3] / 2 for b in boxes] + [b[1] + b[3] / 2 for b in boxes]
        ys = [b[2] - b[4] / 2 for b in boxes] + [b[2] + b[4] / 2 for b in boxes]
        xs += [p[0] - padding for p in points] + [p[0] + padding for p in points]
        ys += [p[1] - padding for p in points] + [p[1] + padding for p in points]
        self.x0, self.y0 = min(xs), min(ys)
        self.res = resolution
        self.cols = int(math.ceil((max(xs) - self.x0) / resolution)) + 1
        self.rows = int(math.ceil((max(ys) - self.y0) / resolution)) + 1
        self.free = [
            [clearance(self.world(r, c), boxes) > required for c in range(self.cols)]
            for r in range(self.rows)
        ]

    def world(self, row, col):
        return (self.x0 + col * self.res, self.y0 + row * self.res)

    def cell(self, point):
        return (
            int(round((point[1] - self.y0) / self.res)),
            int(round((point[0] - self.x0) / self.res)),
        )

    def cells_within(self, point, radius):
        row, col = self.cell(point)
        span = int(math.ceil(radius / self.res))
        for r in range(max(0, row - span), min(self.rows, row + span + 1)):
            for c in range(max(0, col - span), min(self.cols, col + span + 1)):
                if self.free[r][c] and math.dist(self.world(r, c), point) < radius:
                    yield (r, c)

    def shortest(self, sources, targets):
        targets = set(targets)
        if not sources or not targets:
            return None
        dist = {cell: 0.0 for cell in sources}
        queue = [(0.0, cell) for cell in sources]
        heapq.heapify(queue)
        steps = [(dr, dc, self.res * math.hypot(dr, dc))
                 for dr in (-1, 0, 1) for dc in (-1, 0, 1) if dr or dc]
        while queue:
            d, (r, c) = heapq.heappop(queue)
            if (r, c) in targets:
                return d
            if d > dist.get((r, c), float("inf")):
                continue
            for dr, dc, cost in steps:
                nr, nc = r + dr, c + dc
                if 0 <= nr < self.rows and 0 <= nc < self.cols and self.free[nr][nc]:
                    nd = d + cost
                    if nd < dist.get((nr, nc), float("inf")):
                        dist[(nr, nc)] = nd
                        heapq.heappush(queue, (nd, (nr, nc)))
        return None


def segment_min_clearance(a, b, boxes, step=0.01):
    count = max(1, int(math.dist(a, b) / step))
    best = (float("inf"), None)
    for i in range(count + 1):
        t = i / count
        point = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
        value = clearance(point, boxes)
        if value < best[0]:
            best = (value, point)
    return best


def analyse(world, controller, start=None, tolerance=0.14,
            radius=DWA_ROBOT_RADIUS + DWA_STATIC_CLEARANCE_MARGIN):
    boxes = static_boxes(world)
    waypoints = controller_waypoints(controller)
    grid = Grid(boxes, radius, points=waypoints + ([start] if start else []))
    report = []
    previous_region = None
    previous_point = start
    for index, waypoint in enumerate(waypoints):
        region = list(grid.cells_within(waypoint, tolerance))
        entry = {
            "index": index,
            "waypoint": waypoint,
            "waypoint_clearance": round(clearance(waypoint, boxes), 3),
            "nearest": nearest_box(waypoint, boxes)[0],
            "reachable_with_margin": bool(region),
        }
        if previous_point is not None:
            seg_clearance, seg_point = segment_min_clearance(previous_point, waypoint, boxes)
            entry["segment_min_clearance"] = round(seg_clearance, 3)
            entry["segment_pinch"] = [round(v, 3) for v in seg_point]
            entry["segment_pinch_obstacle"] = nearest_box(seg_point, boxes)[0]
            straight = math.dist(previous_point, waypoint)
            if previous_region is None:
                previous_region = list(grid.cells_within(previous_point, tolerance))
            path = grid.shortest(previous_region, region)
            entry["straight_m"] = round(straight, 2)
            entry["free_path_m"] = None if path is None else round(path, 2)
        entry["feasible"] = entry["reachable_with_margin"] and (
            previous_point is None or entry.get("free_path_m") is not None
        )
        report.append(entry)
        previous_region = region
        previous_point = waypoint
    return {"radius": radius, "tolerance": tolerance, "segments": report}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("world", type=Path)
    parser.add_argument("controller", type=Path)
    parser.add_argument("--start", type=float, nargs=2)
    parser.add_argument("--tolerance", type=float, default=0.14)
    parser.add_argument("--radius", type=float,
                        default=DWA_ROBOT_RADIUS + DWA_STATIC_CLEARANCE_MARGIN)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    result = analyse(args.world, args.controller, args.start, args.tolerance, args.radius)
    if args.json:
        print(json.dumps(result, indent=2))
        return 0
    print(f"required clearance {result['radius']:.3f} m, acceptance {result['tolerance']:.2f} m")
    bad = 0
    for entry in result["segments"]:
        flags = []
        if not entry["reachable_with_margin"]:
            flags.append("WAYPOINT-UNREACHABLE")
        if entry.get("segment_min_clearance", float("inf")) <= result["radius"]:
            flags.append("STRAIGHT-SEGMENT-BLOCKED")
        if "free_path_m" in entry and entry["free_path_m"] is None:
            flags.append("NO-FREE-PATH")
        elif entry.get("free_path_m") and entry["free_path_m"] > 1.5 * entry["straight_m"] + 0.3:
            flags.append("DETOUR")
        bad += not entry["feasible"]
        print(
            f"wp{entry['index']:2d} {entry['waypoint']} clr={entry['waypoint_clearance']:.3f}"
            f" seg_min={entry.get('segment_min_clearance', '-')}"
            f" straight={entry.get('straight_m', '-')} free={entry.get('free_path_m', '-')}"
            f" {' '.join(flags)}"
            + (f" pinch={entry['segment_pinch']} ({entry['segment_pinch_obstacle']})"
               if "STRAIGHT-SEGMENT-BLOCKED" in flags else "")
        )
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
