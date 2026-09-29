"""Move a kinematic obstacle back and forth along one world axis."""

import math

from controller import Supervisor


def parse_settings(text):
    settings = {}
    for token in text.split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        settings[key] = value
    return settings


def main():
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    node = robot.getSelf()
    translation = node.getField("translation")
    origin = list(translation.getSFVec3f())
    settings = parse_settings(node.getField("customData").getSFString())

    axis_name = settings.get("axis", "y").lower()
    axis_index = {"x": 0, "y": 1, "z": 2}.get(axis_name, 1)
    amplitude = float(settings.get("amplitude", "1.0"))
    speed = float(settings.get("speed", "0.5"))
    phase = float(settings.get("phase", "0.0"))

    print(
        f"[Moving obstacle] {node.getField('name').getSFString()}: "
        f"axis={axis_name}, amplitude={amplitude:.2f} m, speed={speed:.2f} rad/s"
    )

    while robot.step(timestep) != -1:
        position = list(origin)
        position[axis_index] += amplitude * math.sin(speed * robot.getTime() + phase)
        translation.setSFVec3f(position)


if __name__ == "__main__":
    main()
