"""Scenario geometry shared by the 2D harness and the generated Webots worlds.

Every scenario is expressed in its own local frame (robot starts near the
origin facing +x).  ``build_world`` places each scenario at a fixed offset in
one Webots world, so the Webots runs exercise exactly the geometry the
deterministic harness uses.  All scenarios must stay physically passable for
a disk of 0.25 m radius (the planner keeps 0.22 m); ``tools.avoidance_sim``
has ``passable()`` and the unit tests enforce it.

Usage (regenerate the world files after editing a scenario):
    python tools/avoidance_scenarios.py
"""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
WALL = 0.12
BOX_HEIGHT = 0.50
WALL_HEIGHT = 0.60


def box(cx, cy, sx, sy):
    return (cx, cy, sx, sy)


def hwall(x0, x1, y):
    """Horizontal wall from x0 to x1 centred on y."""
    return box(0.5 * (x0 + x1), y, abs(x1 - x0), WALL)


def vwall(x, y0, y1):
    """Vertical wall from y0 to y1 centred on x."""
    return box(x, 0.5 * (y0 + y1), WALL, abs(y1 - y0))


def mover(axis, fixed, low, high, speed, trigger_axis, trigger, dwell=3.5, size=0.28,
          start="min"):
    """A box crossing along ``axis`` once the robot reaches ``trigger``.

    Mirrors controllers/extended_crossing_controller: starts at ``low``
    (``start="min"``) or ``high`` (``start="max"``), crosses to the other
    end, dwells, returns.  ``trigger`` fires when the robot's
    ``trigger_axis`` coordinate is >= the value (scenario frame).
    """
    return dict(axis=axis, fixed=fixed, low=low, high=high, speed=speed,
                trigger_axis=trigger_axis, trigger=trigger, dwell=dwell, size=size,
                start=start)


def _cases():
    cases = {}
    goal = [(3.2, 0.0)]
    cases["baseline"] = dict(start=(0.0, 0.0, 0.0), waypoints=goal, boxes=[])
    cases["front_left"] = dict(start=(0.0, 0.0, 0.0), waypoints=goal,
                               boxes=[box(0.90, 0.275, 0.45, 0.45)])
    cases["front_right"] = dict(start=(0.0, 0.0, 0.0), waypoints=goal,
                                boxes=[box(0.90, -0.275, 0.45, 0.45)])
    cases["frontal"] = dict(start=(0.0, 0.0, 0.0), waypoints=goal,
                            boxes=[box(0.90, 0.0, 0.45, 0.45)])
    cases["corridor"] = dict(start=(0.0, 0.0, 0.0), waypoints=[(4.6, 0.0)],
                             boxes=[hwall(0.4, 4.4, 0.60), hwall(0.4, 4.4, -0.60)])
    # L corner: corridor east then north, 1.10 m free width.
    cases["corner"] = dict(
        start=(0.0, 0.0, 0.0), waypoints=[(2.0, 3.0)],
        boxes=[hwall(-0.5, 2.66, -0.61), hwall(-0.5, 1.45, 0.61),
               vwall(2.61, -0.61, 3.3), vwall(1.39, 0.61, 3.3)],
    )
    # --- stress set -----------------------------------------------------
    cases["slalom_narrow"] = dict(
        start=(0.0, 0.0, 0.0), waypoints=[(5.4, 0.0)],
        boxes=[box(1.2, 0.20, 0.40, 0.40), box(2.3, -0.20, 0.40, 0.40),
               box(3.4, 0.20, 0.40, 0.40), box(4.5, -0.20, 0.40, 0.40)],
    )
    cases["alternating_corners"] = dict(
        start=(0.0, 0.0, 0.0), waypoints=[(5.0, 0.0)],
        boxes=[box(1.0, 0.275, 0.45, 0.45), box(2.4, -0.275, 0.45, 0.45),
               box(3.8, 0.275, 0.45, 0.45)],
    )
    cases["s_passage"] = dict(
        start=(0.0, 0.0, 0.0), waypoints=[(1.5, 0.0), (1.5, 1.8), (3.6, 1.8)],
        boxes=[hwall(-0.5, 2.1, -0.55), hwall(-0.5, 0.95, 0.55),
               vwall(2.1, -0.55, 1.25), vwall(0.95, 0.55, 2.35),
               hwall(2.1, 4.2, 1.25), hwall(0.95, 4.2, 2.35)],
    )
    # U open towards the robot, goal behind its bottom: classic local minimum.
    cases["u_trap"] = dict(
        start=(0.0, 0.0, 0.0), waypoints=[(3.4, 0.0)],
        boxes=[vwall(1.8, -0.75, 0.75), hwall(0.9, 1.8, 0.75), hwall(0.9, 1.8, -0.75)],
    )
    cases["narrow_corridor"] = dict(
        start=(0.0, 0.0, 0.0), waypoints=[(4.4, 0.0)],
        boxes=[hwall(0.5, 4.0, 0.47), hwall(0.5, 4.0, -0.47)],
    )
    cases["corridor_exit_obstacle"] = dict(
        start=(0.0, 0.0, 0.0), waypoints=[(4.4, 0.0)],
        boxes=[hwall(0.3, 2.5, 0.60), hwall(0.3, 2.5, -0.60), box(3.3, 0.05, 0.45, 0.45)],
    )
    cases["mixed_sizes"] = dict(
        start=(0.0, 0.0, 0.0), waypoints=[(6.0, 0.0)],
        boxes=[box(1.0, 0.10, 0.25, 0.25), box(2.2, -0.25, 0.70, 0.45),
               box(3.5, 0.30, 0.35, 0.60), box(4.7, -0.05, 0.20, 0.20)],
    )
    # Recovery chain: large frontal block then an immediate second block.
    cases["recovery_chain"] = dict(
        start=(0.0, 0.0, 0.0), waypoints=[(4.4, 0.0)],
        boxes=[box(0.9, 0.0, 0.50, 0.80), box(2.3, 0.10, 0.50, 0.80),
               box(3.5, -0.05, 0.45, 0.45)],
    )
    cases["static_dynamic_mixed"] = dict(
        start=(0.0, 0.0, 0.0), waypoints=[(4.0, 0.0)],
        boxes=[box(1.0, 0.30, 0.45, 0.45)],
        movers=[mover("y", 2.4, -1.4, 1.4, 0.30, "x", 1.2)],
    )
    cases["crossing"] = dict(
        start=(0.0, 0.0, 0.0), waypoints=[(3.5, 0.0)],
        boxes=[],
        movers=[mover("y", 1.8, -1.4, 1.4, 0.30, "x", 0.5)],
    )
    # Mover from the robot's left, timed to reach the lane together with the
    # robot at cruise speed (the Extended dynamic-A contact geometry).
    cases["side_approach"] = dict(
        start=(0.0, 0.0, 0.0), waypoints=[(3.6, 0.0)],
        boxes=[],
        movers=[mover("y", 2.0, -1.6, 1.4, 0.30, "x", 0.9, start="max")],
    )
    # Endurance: one long course chaining the stress motifs.
    cases["endurance"] = dict(
        start=(0.0, 0.0, 0.0),
        seconds=180.0,  # ~17 m of waypoints: longer than the default 90 s budget
        waypoints=[(4.4, 0.0), (6.5, 0.0), (6.5, 3.0), (2.0, 3.0), (-0.8, 3.0)],
        boxes=[
            box(1.0, 0.275, 0.45, 0.45), box(2.4, -0.275, 0.45, 0.45),
            box(3.6, 0.0, 0.45, 0.45),
            hwall(4.9, 7.3, -0.62), vwall(7.3, -0.62, 3.6),
            box(6.4, 1.5, 0.45, 0.45),
            vwall(3.9, 2.25, 3.75), hwall(3.9, 5.5, 3.75),
            box(1.2, 3.1, 0.45, 0.45), box(0.2, 2.75, 0.30, 0.30),
        ],
    )
    return cases


CASES = _cases()
REGRESSION_CASES = ("baseline", "front_left", "front_right", "frontal", "corridor", "corner")
STRESS_CASES = (
    "slalom_narrow", "alternating_corners", "s_passage", "u_trap",
    "narrow_corridor", "corridor_exit_obstacle", "mixed_sizes",
    "recovery_chain", "static_dynamic_mixed", "crossing", "side_approach", "endurance",
)

# World layout: each scenario at its own offset, far beyond LiDAR range (4 m)
# of the neighbouring ones.
WORLDS = {
    "avoidance_regression": REGRESSION_CASES,
    "avoidance_stress": STRESS_CASES,
}
_SPACING_X = 14.0
_SPACING_Y = 12.0
_COLUMNS = 4


def offset(name):
    for cases in WORLDS.values():
        if name in cases:
            index = cases.index(name)
            return (_SPACING_X * (index % _COLUMNS), _SPACING_Y * (index // _COLUMNS))
    return (0.0, 0.0)


def world_case(name):
    """Scenario translated into world coordinates of its generated world."""
    case = CASES[name]
    ox, oy = offset(name)
    moved = dict(case)
    moved["start"] = (case["start"][0] + ox, case["start"][1] + oy, case["start"][2])
    moved["waypoints"] = [(x + ox, y + oy) for x, y in case["waypoints"]]
    moved["boxes"] = [(cx + ox, cy + oy, sx, sy) for cx, cy, sx, sy in case["boxes"]]
    moved["movers"] = [
        dict(spec,
             fixed=spec["fixed"] + (oy if spec["axis"] == "x" else ox),
             low=spec["low"] + (ox if spec["axis"] == "x" else oy),
             high=spec["high"] + (ox if spec["axis"] == "x" else oy),
             trigger=spec["trigger"] + (ox if spec["trigger_axis"] == "x" else oy),
             gate=(oy - 3.0, oy + 3.0) if spec["trigger_axis"] == "x" else (ox - 3.0, ox + 7.0))
        for spec in case.get("movers", [])
    ]
    return moved


# ---------------------------------------------------------------------------
# Webots world generation
# ---------------------------------------------------------------------------
_ROBOT = (ROOT / "worlds" / "avoidance_extended.wbt").read_text(encoding="utf-8")


def _robot_node():
    """The Extended robot node verbatim, renamed and pointed at our controller."""
    start = _ROBOT.index("DEF EXTENDED_ROBOT Robot {")
    end = _ROBOT.index("\n}\n", start) + 3
    node = _ROBOT[start:end]
    node = node.replace("DEF EXTENDED_ROBOT", "DEF REGRESSION_ROBOT")
    node = node.replace('name "extended endurance robot"', 'name "regression robot"')
    node = node.replace('translation -9.50 -6.50 0.04', 'translation 0 0 0.04')
    node = node.replace('controller "avoidance_extended_controller"',
                        'controller "avoidance_regression_controller"')
    assert 'controller "avoidance_regression_controller"' in node
    return node


def _solid(name, rect, height, color):
    cx, cy, sx, sy = rect
    return (
        "Solid {\n"
        f"  translation {cx:.4f} {cy:.4f} {height / 2:.3f}\n"
        f'  name "{name}"\n'
        f"  children [ Shape {{ appearance PBRAppearance {{ baseColor {color} roughness 0.7 }} "
        f"geometry Box {{ size {sx:.4f} {sy:.4f} {height:.3f} }} }} ]\n"
        f"  boundingObject Box {{ size {sx:.4f} {sy:.4f} {height:.3f} }}\n"
        "}\n"
    )


def _mover_node(name, spec):
    axis = spec["axis"]
    begin = spec["high"] if spec.get("start", "min") == "max" else spec["low"]
    x, y = (begin, spec["fixed"]) if axis == "x" else (spec["fixed"], begin)
    gate_axis = "y" if spec["trigger_axis"] == "x" else "x"
    size = spec["size"]
    custom = (
        f"axis={axis} min={spec['low']:.3f} max={spec['high']:.3f} speed={spec['speed']:.2f} "
        f"dwell={spec['dwell']:.1f} start={spec.get('start', 'min')} trigger_axis={spec['trigger_axis']} "
        f"trigger={spec['trigger']:.3f} trigger_dir=ge gate_axis={gate_axis} "
        f"gate_min={spec['gate'][0]:.2f} gate_max={spec['gate'][1]:.2f} robot=REGRESSION_ROBOT"
    )
    return (
        "Robot {\n"
        f"  translation {x:.4f} {y:.4f} 0.18\n"
        f'  name "{name}"\n'
        "  children [ Shape { appearance PBRAppearance { baseColor 0.10 0.70 0.30 roughness 0.6 } "
        f"geometry Box {{ size {size} {size} 0.36 }} }} ]\n"
        f"  boundingObject Box {{ size {size} {size} 0.36 }}\n"
        '  controller "extended_crossing_controller"\n'
        f'  customData "{custom}"\n'
        "  supervisor TRUE\n"
        "}\n"
    )


def build_world(world_name):
    names = WORLDS[world_name]
    rows = (len(names) + _COLUMNS - 1) // _COLUMNS
    width = _SPACING_X * _COLUMNS + 10.0
    height = _SPACING_Y * rows + 10.0
    parts = [
        "#VRML_SIM R2025a utf8\n",
        f"# GENERATED by tools/avoidance_scenarios.py -- do not edit by hand.\n"
        f"# Scenarios: {', '.join(names)}.\n"
        "# Robot body, wheels, LiDAR, compass and timestep are identical to\n"
        "# worlds/avoidance_extended.wbt; the controller teleports the robot to the\n"
        "# selected scenario's start pose before the first physics step.\n\n",
        "WorldInfo {\n  basicTimeStep 32\n}\n",
        "Viewpoint {\n  orientation 0 0 1 0\n"
        f"  position {width / 2 - 5:.1f} {height / 2 - 5:.1f} 40\n}}\n",
        "Background {\n  skyColor [ 0.76 0.86 1 ]\n}\n",
        "DirectionalLight {\n  direction -0.4 0.2 -1\n  intensity 1.2\n}\n",
        "Solid {\n"
        f"  translation {width / 2 - 5:.1f} {height / 2 - 5:.1f} -0.025\n"
        '  name "regression floor"\n'
        "  children [ Shape { appearance PBRAppearance { baseColor 0.76 0.76 0.73 roughness 0.9 } "
        f"geometry Box {{ size {width:.1f} {height:.1f} 0.05 }} }} ]\n"
        f"  boundingObject Box {{ size {width:.1f} {height:.1f} 0.05 }}\n"
        "}\n\n",
        _robot_node(),
    ]
    for name in names:
        case = world_case(name)
        parts.append(f"\n# --- scenario {name} (offset {offset(name)}) ---\n")
        for index, rect in enumerate(case["boxes"]):
            is_wall = min(rect[2], rect[3]) <= WALL + 1e-9
            parts.append(_solid(
                f"{name} {'wall' if is_wall else 'box'} {index}", rect,
                WALL_HEIGHT if is_wall else BOX_HEIGHT,
                "0.34 0.39 0.43" if is_wall else "0.80 0.40 0.15",
            ))
        for index, spec in enumerate(case["movers"]):
            parts.append(_mover_node(f"{name} mover {index}", spec))
    return "".join(parts)


def main():
    for world_name in WORLDS:
        path = ROOT / "worlds" / f"{world_name}.wbt"
        path.write_text(build_world(world_name), encoding="utf-8")
        print(f"wrote {path.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
