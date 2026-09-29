"""Generate minimal regression and stress worlds for the avoidance stack.

Every world reuses the robot block of worlds/avoidance_extended.wbt verbatim
(same body, wheels, LiDAR, compass, physics), swaps only its controller for
``avoidance_course_controller`` and writes the route into customData.  The
original benchmark worlds are never modified.

    python tools/build_avoidance_courses.py            # write all courses
    python tools/build_avoidance_courses.py --check    # geometric feasibility

Passages are sized for the planner's collision radius plus static clearance
margin (0.22 m from each wall face, i.e. >= 0.44 m wall to wall); the
``--check`` option verifies each route with tools/check_course_feasibility.py.
"""

import argparse
from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

WORLDS = ROOT / "worlds"
SOURCE_WORLD = WORLDS / "avoidance_extended.wbt"
CONTROLLER = "avoidance_course_controller"
WALL = 0.12


def wall_x(x0, x1, y, name):
    """Horizontal wall along x from x0 to x1 centred on y."""
    return (name, 0.5 * (x0 + x1), y, abs(x1 - x0), WALL)


def wall_y(y0, y1, x, name):
    """Vertical wall along y from y0 to y1 centred on x."""
    return (name, x, 0.5 * (y0 + y1), WALL, abs(y1 - y0))


def box(x, y, name, size=0.45):
    return (name, x, y, size, size)


def actor(name, x, y, settings):
    return (name, x, y, settings)


COURSES = {}


def course(name, start, waypoints, boxes=(), actors=(), max_seconds=120.0,
           stall_seconds=35.0, floor=(14.0, 10.0), kind="minimal"):
    COURSES[name] = {
        "start": start, "waypoints": waypoints, "boxes": list(boxes),
        "actors": list(actors), "max": max_seconds, "stall": stall_seconds,
        "floor": floor, "kind": kind,
    }


# --- minimal single-feature regressions ------------------------------------
course("min_clear", (0.0, 0.0, 0.0), [(3.0, 0.0)])
course("min_front_left", (0.0, 0.0, 0.0), [(3.5, 0.0)],
       [box(1.20, 0.275, "front-left block")])
course("min_front_right", (0.0, 0.0, 0.0), [(3.5, 0.0)],
       [box(1.20, -0.275, "front-right block")])
course("min_frontal", (0.0, 0.0, 0.0), [(3.5, 0.0)],
       [box(1.20, 0.0, "frontal block")])
course("min_side", (0.0, 0.0, 0.0), [(3.5, 0.0)],
       [box(1.20, 0.65, "side block")])
course("min_corridor", (0.0, 0.0, 0.0), [(1.0, 0.0), (4.5, 0.0)],
       [wall_x(0.5, 3.5, 0.55, "corridor north"),
        wall_x(0.5, 3.5, -0.55, "corridor south")])
course("min_s_turn", (0.0, 0.0, 0.0), [(2.0, 0.0), (2.0, 1.5), (4.0, 1.5)],
       [wall_x(-0.5, 2.55, -0.55, "leg1 south"),
        wall_x(-0.5, 1.45, 0.55, "leg1 north"),
        wall_y(0.55, 2.05, 1.45, "riser west"),
        wall_y(-0.55, 0.95, 2.55, "riser east"),
        wall_x(1.45, 4.5, 2.05, "leg3 north"),
        wall_x(2.55, 4.5, 0.95, "leg3 south")])
course("min_slalom", (0.0, 0.0, 0.0),
       [(1.0, -0.45), (2.0, 0.45), (3.0, -0.45), (4.0, 0.45), (5.0, 0.0)],
       [box(1.0, 0.30, "slalom 1"), box(2.0, -0.30, "slalom 2"),
        box(3.0, 0.30, "slalom 3"), box(4.0, -0.30, "slalom 4"),
        wall_x(0.3, 4.7, 1.10, "slalom north"),
        wall_x(0.3, 4.7, -1.10, "slalom south")])

# --- stress environments -----------------------------------------------------
# Tighter than min_slalom: 0.95 m box pitch leaves 0.50 m diagonal gaps
# (>= 0.44 m required by radius + static margin) and 0.415 m to the walls on
# the box side, so every pass must weave on the open side.
course("stress_tight_slalom", (0.0, 0.0, 0.0),
       [(0.9, -0.45), (1.85, 0.45), (2.8, -0.45), (3.75, 0.45), (4.7, -0.45), (5.6, 0.0)],
       [box(0.9, 0.30, "s1"), box(1.85, -0.30, "s2"), box(2.8, 0.30, "s3"),
        box(3.75, -0.30, "s4"), box(4.7, 0.30, "s5"),
        wall_x(0.2, 5.3, 1.0, "north"), wall_x(0.2, 5.3, -1.0, "south")],
       kind="stress")
course("stress_alternating_corners", (0.0, 0.0, 0.0), [(7.0, 0.0)],
       [box(1.2, 0.275, "c1"), box(2.6, -0.275, "c2"),
        box(4.0, 0.275, "c3"), box(5.4, -0.275, "c4")],
       kind="stress")
course("stress_s_corridor", (0.0, 0.0, 0.0),
       [(1.6, 0.0), (1.6, 1.2), (3.2, 1.2), (3.2, 0.0), (4.8, 0.0)],
       [wall_x(-0.5, 2.0, -0.40, "a south"), wall_x(-0.5, 1.2, 0.40, "a north"),
        wall_y(0.40, 1.60, 1.20, "b west"), wall_y(-0.40, 0.80, 2.00, "b east"),
        wall_x(1.20, 3.60, 1.60, "c north"), wall_x(2.00, 2.80, 0.80, "c south"),
        wall_y(-0.40, 0.80, 2.80, "d west"), wall_y(0.40, 1.60, 3.60, "d east"),
        wall_x(2.80, 5.3, -0.40, "e south"), wall_x(3.60, 5.3, 0.40, "e north")],
       kind="stress")
course("stress_u_trap", (0.0, 0.0, 0.0), [(3.0, 0.0)],
       [wall_y(-0.8, 0.8, 1.8, "u back"), wall_x(0.9, 1.8, 0.8, "u north arm"),
        wall_x(0.9, 1.8, -0.8, "u south arm")],
       max_seconds=150.0, kind="stress")
# Diagnostic companions (same geometry, reported separately): the U-trap with
# a 120 s no-progress window distinguishes a permanent stall from a slow but
# correct wall-follow detour, and the chicane with gap waypoints is what a
# global planner would hand the local planner.
course("stress_u_trap_long", (0.0, 0.0, 0.0), [(3.0, 0.0)],
       [wall_y(-0.8, 0.8, 1.8, "u back"), wall_x(0.9, 1.8, 0.8, "u north arm"),
        wall_x(0.9, 1.8, -0.8, "u south arm")],
       max_seconds=240.0, stall_seconds=120.0, kind="stress")
course("stress_chicane_waypoints", (0.0, 0.0, 0.0),
       [(1.5, 0.62), (3.0, -0.65), (4.5, 0.65), (6.0, 0.0)],
       [wall_y(-1.0, 0.30, 1.5, "gate 1"), wall_y(-0.30, 1.0, 3.0, "gate 2"),
        wall_y(-1.0, 0.30, 4.5, "gate 3"),
        wall_x(0.0, 6.5, 1.06, "north"), wall_x(0.0, 6.5, -1.06, "south")],
       max_seconds=180.0, kind="stress")
course("stress_corridor_exit_obstacle", (0.0, 0.0, 0.0), [(1.0, 0.0), (5.5, 0.0)],
       [wall_x(0.5, 3.5, 0.50, "corridor north"),
        wall_x(0.5, 3.5, -0.50, "corridor south"),
        box(4.2, 0.0, "exit block")],
       kind="stress")
course("stress_chicane", (0.0, 0.0, 0.0), [(6.0, 0.0)],
       [wall_y(-1.0, 0.30, 1.5, "gate 1"), wall_y(-0.30, 1.0, 3.0, "gate 2"),
        wall_y(-1.0, 0.30, 4.5, "gate 3"),
        wall_x(0.0, 6.5, 1.06, "north"), wall_x(0.0, 6.5, -1.06, "south")],
       max_seconds=180.0, kind="stress")
course("stress_mixed_static_dynamic", (0.0, 0.0, 0.0), [(1.2, 0.0), (5.0, 0.0)],
       [box(1.8, 0.45, "static before crossing"), box(3.8, -0.45, "static after crossing")],
       [actor("crossing actor", 2.8, -1.4,
              "axis=y min=-1.40 max=1.40 speed=0.30 dwell=2.0 start=min "
              "trigger_axis=x trigger=1.40 trigger_dir=ge robot=EXTENDED_ROBOT")],
       kind="stress")


# Extended corridor 2 in its original coordinates with only actor C (fast,
# 0.45 m/s, 3.5 s dwell at both lane ends inside the corridor) and the
# original waypoints, one of which lies on the actor's lane.
course("stress_corridor_crossing", (-8.60, 1.60, 1.5708),
       [(-8.60, 2.20), (-8.60, 3.20), (-8.60, 4.40)],
       [wall_y(2.20, 5.20, -9.30, "corridor2 west wall"),
        wall_y(2.20, 5.20, -7.90, "corridor2 east wall")],
       [actor("dynamic actor C fast", -9.00, 3.20,
              "axis=x min=-9.00 max=-8.20 speed=0.45 dwell=3.5 start=min "
              "trigger_axis=y trigger=2.70 trigger_dir=ge gate_axis=x "
              "gate_min=-9.00 gate_max=-8.20 robot=EXTENDED_ROBOT")],
       max_seconds=120.0, floor=(6.0, 8.0), kind="stress")


def _endurance():
    boxes = [
        wall_x(-0.5, 8.5, -1.2, "outer south"), wall_x(-0.5, 8.5, 5.2, "outer north"),
        wall_y(-1.2, 5.2, -0.5, "outer west"), wall_y(-1.2, 5.2, 8.5, "outer east"),
        wall_x(1.2, 6.8, 2.0, "island"),
        box(2.0, -0.25, "e1"), box(4.0, 0.25, "e2"), box(6.0, -0.25, "e3"),
        box(7.2, 1.2, "e4"), box(6.0, 4.25, "e5"), box(4.0, 3.75, "e6"),
        box(2.0, 4.25, "e7"), box(0.3, 2.8, "e8"),
    ]
    lap = [(1.2, -0.5), (3.0, 0.3), (5.0, -0.5), (7.2, 0.0), (7.5, 2.6),
           (7.0, 4.0), (5.0, 4.5), (3.0, 3.6), (1.0, 4.4), (0.5, 1.6), (0.0, 0.0)]
    course("stress_endurance", (0.0, 0.0, 0.0), lap * 3, boxes,
           max_seconds=900.0, floor=(12.0, 9.0), kind="stress")


_endurance()


def _extended_detour():
    """Original Extended world with only its infeasible segment re-routed.

    Waypoint 7 (6.50, -6.50) lies 0.10 m from the corridor wall tip and the
    only safe way to it from waypoint 6 goes around slalom obstacle 3
    (tools/check_course_feasibility.py).  A global planner would insert these
    detour points; the local planner cannot find a 3 m detour by itself.
    """
    sys.path.insert(0, str(ROOT / "tools"))
    from check_course_feasibility import controller_waypoints

    waypoints = controller_waypoints(
        ROOT / "controllers/avoidance_extended_controller/avoidance_extended_controller.py"
    )
    detour = [(5.25, -6.30), (5.20, -7.35), (6.35, -7.35), (6.62, -6.85)]
    route = waypoints[:7] + detour + waypoints[7:]
    return route


def robot_block(source_text, start, customdata):
    begin = source_text.index("DEF EXTENDED_ROBOT Robot {")
    end = source_text.index("\n}\n", begin) + 3
    block = source_text[begin:end]
    block = re.sub(r"translation -9\.50 -6\.50 0\.04",
                   f"translation {start[0]:.3f} {start[1]:.3f} 0.04", block, count=1)
    block = re.sub(r"rotation 0 0 1 0\n",
                   f"rotation 0 0 1 {start[2]:.6f}\n", block, count=1)
    block = block.replace('controller "avoidance_extended_controller"',
                          f'controller "{CONTROLLER}"\n  customData "{customdata}"')
    return block


def customdata(name, spec):
    start = ",".join(f"{v:g}" for v in spec["start"])
    route = "|".join(f"{x:g},{y:g}" for x, y in spec["waypoints"])
    return (f"course={name} start={start} wp={route} "
            f"max={spec['max']:g} stall={spec['stall']:g}")


def render(name, spec, source_text):
    header = source_text[:source_text.index("Solid {")]
    fx, fy = spec["floor"]
    xs = [p[0] for p in spec["waypoints"]] + [spec["start"][0]]
    ys = [p[1] for p in spec["waypoints"]] + [spec["start"][1]]
    cx, cy = 0.5 * (min(xs) + max(xs)), 0.5 * (min(ys) + max(ys))
    parts = [
        f"# Generated by tools/build_avoidance_courses.py ({spec['kind']} course '{name}').\n"
        f"# Robot block copied from worlds/avoidance_extended.wbt; do not edit by hand.\n",
        header.replace("#VRML_SIM R2025a utf8\n", ""),
        "Solid {\n"
        f"  translation {cx:.3f} {cy:.3f} -0.025\n"
        '  name "course floor"\n'
        "  children [ Shape { appearance PBRAppearance { baseColor 0.76 0.76 0.73 "
        f"roughness 0.9 }} geometry Box {{ size {fx} {fy} 0.05 }} }} ]\n"
        f"  boundingObject Box {{ size {fx} {fy} 0.05 }}\n}}\n",
        robot_block(source_text, spec["start"], customdata(name, spec)),
    ]
    for label, x, y, sx, sy in spec["boxes"]:
        parts.append(
            "Solid {\n"
            f"  translation {x:.4f} {y:.4f} 0.30\n"
            f'  name "{label}"\n'
            "  children [ Shape { appearance PBRAppearance { baseColor 0.78 0.45 0.18 } "
            f"geometry Box {{ size {sx:.4f} {sy:.4f} 0.6 }} }} ]\n"
            f"  boundingObject Box {{ size {sx:.4f} {sy:.4f} 0.6 }}\n}}\n"
        )
    for label, x, y, settings in spec["actors"]:
        parts.append(
            "Robot {\n"
            f"  translation {x:.3f} {y:.3f} 0.18\n"
            f'  name "{label}"\n'
            "  children [ Shape { appearance PBRAppearance { baseColor 0.9 0.1 0.1 } "
            "geometry Box { size 0.28 0.28 0.36 } } ]\n"
            "  boundingObject Box { size 0.28 0.28 0.36 }\n"
            '  controller "extended_crossing_controller"\n'
            f'  customData "{settings}"\n'
            "  supervisor TRUE\n}\n"
        )
    return "#VRML_SIM R2025a utf8\n" + "".join(parts)


def render_extended_detour(source_text):
    spec = {"start": (-9.50, -6.50, 0.0), "waypoints": _extended_detour(),
            "max": 600.0, "stall": 35.0}
    block = robot_block(source_text, spec["start"],
                        customdata("stress_extended_detour", spec))
    begin = source_text.index("DEF EXTENDED_ROBOT Robot {")
    end = source_text.index("\n}\n", begin) + 3
    return (
        source_text[:begin]
        + block
        + source_text[end:]
    ).replace(
        "#VRML_SIM R2025a utf8\n",
        "#VRML_SIM R2025a utf8\n# Generated by tools/build_avoidance_courses.py: "
        "avoidance_extended.wbt with only waypoint 7 re-routed (see _extended_detour).\n",
        1,
    )


def world_path(name):
    return WORLDS / f"course_{name}.wbt"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true",
                        help="run the geometric feasibility check on every course")
    args = parser.parse_args()
    source_text = SOURCE_WORLD.read_text(encoding="utf-8")
    written = []
    for name, spec in COURSES.items():
        world_path(name).write_text(render(name, spec, source_text), encoding="utf-8")
        written.append(name)
    world_path("stress_extended_detour").write_text(
        render_extended_detour(source_text), encoding="utf-8")
    written.append("stress_extended_detour")
    print("\n".join(str(world_path(name).relative_to(ROOT)) for name in written))
    if args.check:
        from check_course_feasibility import Grid, clearance, static_boxes
        from config import DWA_ROBOT_RADIUS, DWA_STATIC_CLEARANCE_MARGIN
        required = DWA_ROBOT_RADIUS + DWA_STATIC_CLEARANCE_MARGIN
        failed = False
        for name in written:
            boxes = static_boxes(world_path(name))
            text = world_path(name).read_text(encoding="utf-8")
            data = re.search(r'customData "course=\S+ start=(\S+) wp=(\S+)', text)
            start = tuple(float(v) for v in data.group(1).split(","))[:2]
            route = [tuple(float(v) for v in item.split(",")) for item in data.group(2).split("|")]
            grid = Grid(boxes, required, resolution=0.03, points=route + [start])
            previous = list(grid.cells_within(start, 0.14))
            status = []
            for point in route:
                region = list(grid.cells_within(point, 0.14))
                path = grid.shortest(previous, region)
                status.append("ok" if path is not None else "BLOCKED")
                previous = region
            bad = [i for i, s in enumerate(status) if s != "ok"]
            failed |= bool(bad)
            print(f"{name}: {'feasible' if not bad else 'INFEASIBLE at ' + str(bad)}"
                  f" (start clearance {clearance(start, boxes):.2f} m)")
        return 1 if failed else 0
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
