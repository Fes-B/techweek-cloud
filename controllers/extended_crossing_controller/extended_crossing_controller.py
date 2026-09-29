"""Deterministic crossing actor for the extended endurance benchmark.

Independent copy of practice_crossing_controller.py's state machine (that file
is shared with the protected avoidance_practice.wbt smoke test and must not be
touched). Adds two optional, backward-compatible customData keys needed for
actors placed on legs where the robot travels in the negative direction:

  start=min|max   which end the actor starts at (default: min, matching the
                  original controller exactly)
  trigger_dir=ge|le   whether the trigger fires on target >= trigger (default:
                  ge, matching the original ">=" check) or target <= trigger

  gate_axis/gate_min/gate_max   an optional second condition the robot's
                  position must satisfy (gate_min <= position[gate_axis] <=
                  gate_max) at the same time as the primary trigger. Needed
                  whenever a long course revisits similar coordinate ranges
                  in different passes -- a single-axis trigger would then
                  fire the first time the robot ever crosses that value,
                  regardless of which pass it is on. Default: no gate (the
                  original single-axis behavior).
"""

from controller import Supervisor


def parse_settings(text):
    settings = {}
    for token in text.split():
        if "=" in token:
            key, value = token.split("=", 1)
            settings[key] = value
    return settings


def main():
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    node = robot.getSelf()
    translation = node.getField("translation")
    settings = parse_settings(node.getField("customData").getSFString())

    axis_name = settings.get("axis", "x").lower()
    axis_index = {"x": 0, "y": 1, "z": 2}.get(axis_name, 0)
    minimum = float(settings.get("min", "0.3"))
    maximum = float(settings.get("max", "3.1"))
    speed = float(settings.get("speed", "0.30"))
    dwell = float(settings.get("dwell", "3.5"))
    trigger_axis_name = settings.get("trigger_axis", "y").lower()
    trigger_axis = {"x": 0, "y": 1, "z": 2}.get(trigger_axis_name, 1)
    trigger_value = float(settings.get("trigger", "-0.5"))
    trigger_dir = settings.get("trigger_dir", "ge").lower()
    start_end = settings.get("start", "min").lower()
    gate_axis_name = settings.get("gate_axis")
    gate_axis = {"x": 0, "y": 1, "z": 2}.get(
        gate_axis_name.lower() if gate_axis_name else None
    )
    gate_min = float(settings.get("gate_min", "-1e9"))
    gate_max = float(settings.get("gate_max", "1e9"))
    robot_def = settings.get("robot", "PRACTICE_ROBOT")
    target = robot.getFromDef(robot_def)
    if target is None:
        raise RuntimeError(f"crossing trigger robot DEF not found: {robot_def}")

    start_value = minimum if start_end == "min" else maximum
    # Moving away from the start toward the opposite end on first activation.
    initial_direction = 1.0 if start_end == "min" else -1.0

    position = list(translation.getSFVec3f())
    position[axis_index] = start_value
    translation.setSFVec3f(position)
    state = "waiting"
    direction = initial_direction
    dwell_until = 0.0
    print(
        f"[CROSSING_ACTOR] waiting; {axis_name}={minimum:.2f}..{maximum:.2f}, "
        f"start={start_end}, speed={speed:.2f} m/s, dwell={dwell:.2f} s",
        flush=True,
    )

    while robot.step(timestep) != -1:
        now = robot.getTime()
        position = list(translation.getSFVec3f())
        if state == "waiting":
            target_position = target.getPosition()
            triggered = (
                target_position[trigger_axis] >= trigger_value
                if trigger_dir == "ge"
                else target_position[trigger_axis] <= trigger_value
            )
            if triggered and gate_axis is not None:
                triggered = gate_min <= target_position[gate_axis] <= gate_max
            if triggered:
                state = "moving"
                print(f"[CROSSING_ACTOR] triggered t={now:.2f}", flush=True)
        elif state == "dwelling":
            if now >= dwell_until:
                direction *= -1.0
                state = "moving"
                print(f"[CROSSING_ACTOR] re-enter t={now:.2f}", flush=True)
        else:
            position[axis_index] += direction * speed * timestep / 1000.0
            endpoint = maximum if direction > 0.0 else minimum
            reached = (
                position[axis_index] >= endpoint
                if direction > 0.0
                else position[axis_index] <= endpoint
            )
            if reached:
                position[axis_index] = endpoint
                state = "dwelling"
                dwell_until = now + dwell
                print(
                    f"[CROSSING_ACTOR] clear-side dwell t={now:.2f} "
                    f"until={dwell_until:.2f}",
                    flush=True,
                )
            translation.setSFVec3f(position)


if __name__ == "__main__":
    main()
