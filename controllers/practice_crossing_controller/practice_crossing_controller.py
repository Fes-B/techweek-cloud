"""Deterministic crossing actor with a real clear interval for practice runs."""

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
    robot_def = settings.get("robot", "PRACTICE_ROBOT")
    target = robot.getFromDef(robot_def)
    if target is None:
        raise RuntimeError(f"crossing trigger robot DEF not found: {robot_def}")

    position = list(translation.getSFVec3f())
    position[axis_index] = minimum
    translation.setSFVec3f(position)
    state = "waiting"
    direction = 1.0
    dwell_until = 0.0
    print(
        f"[CROSSING_ACTOR] waiting; {axis_name}={minimum:.2f}..{maximum:.2f}, "
        f"speed={speed:.2f} m/s, dwell={dwell:.2f} s",
        flush=True,
    )

    while robot.step(timestep) != -1:
        now = robot.getTime()
        position = list(translation.getSFVec3f())
        if state == "waiting":
            target_position = target.getPosition()
            if target_position[trigger_axis] >= trigger_value:
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
