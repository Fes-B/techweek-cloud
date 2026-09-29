"""Build a reproducible wall maze; actor routes stay inside free corridors."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def block(text, start):
    depth = 0
    for index in range(text.index('{', start), len(text)):
        depth += (text[index] == '{') - (text[index] == '}')
        if depth == 0:
            return text[start:index + 1]
    raise ValueError('Unclosed robot node')


def wall(name, x, y, sx, sy):
    return f'''Solid {{
  translation {x} {y} 0.3
  name "{name}"
  children [ Shape {{ appearance PBRAppearance {{ baseColor 0.4 0.45 0.5 }}
    geometry Box {{ size {sx} {sy} 0.6 }} }} ]
  boundingObject Box {{ size {sx} {sy} 0.6 }}
}}'''


def actor(name, x, y, phase):
    return f'''Robot {{
  translation {x} {y} 0.18
  name "{name}"
  children [ Shape {{ appearance PBRAppearance {{ baseColor 1 0.6 0.1 }}
    geometry Box {{ size 0.28 0.28 0.36 }} }} ]
  boundingObject Box {{ size 0.28 0.28 0.36 }}
  controller "moving_obstacle_controller"
  customData "axis=x amplitude=0.40 speed=1.0 phase={phase}"
  supervisor TRUE
}}'''


def main():
    source = (ROOT / 'worlds/practice.wbt').read_text(encoding='utf-8')
    robot = block(source, source.index('Robot {'))
    # Two independently driven wheels with passive ball casters. Geometry and
    # encoder names are practice settings, not claimed event specifications.
    offset = 0
    joints = []
    while True:
        start = robot.find('HingeJoint {', offset)
        if start < 0:
            break
        joint = block(robot, start)
        joints.append(joint)
        offset = start + len(joint)
    for joint in joints:
        if 'rear ' in joint:
            robot = robot.replace(joint, '')
        else:
            side = 'left' if 'left wheel motor' in joint else 'right'
            replacement = joint.replace('anchor 0.09', 'anchor 0')
            replacement = replacement.replace('translation 0.09', 'translation 0')
            replacement = replacement.replace(
                'device [', f'device [ PositionSensor {{ name "{side} wheel sensor" }}', 1)
            robot = robot.replace(joint, replacement)
    casters = ''
    for x in (-.10, .10):
        casters += f'''BallJoint {{
          jointParameters BallJointParameters {{ anchor {x} 0 -0.025 }}
          endPoint Solid {{
            translation {x} 0 -0.025
            children [ Shape {{ geometry Sphere {{ radius 0.015 }} }} ]
            boundingObject Sphere {{ radius 0.015 }}
            physics Physics {{ density -1 mass 0.02 }}
          }}
        }}'''
    robot = robot.replace('Lidar {', casters + '\n    Compass { name "heading compass" }\n    Lidar {', 1)
    robot = robot.replace('translation -1.45 0 0.04', 'translation -3 -2 0.04', 1)
    robot = robot.replace('controller "techweek_controller"', 'controller "dense_test_controller"')
    nodes = ['''#VRML_SIM R2025a utf8
WorldInfo { basicTimeStep 32 }
Viewpoint { orientation 0 0 1 0 position 0 0 12 }
Background { skyColor [ 0.75 0.85 1 ] }
DirectionalLight { direction -0.4 0.2 -1 intensity 1.2 }
Solid {
  translation 0 0 -0.025
  name "floor"
  children [ Shape { appearance PBRAppearance { baseColor 0.75 0.75 0.75 }
    geometry Box { size 8 6 0.05 } } ]
  boundingObject Box { size 8 6 0.05 }
}''', robot]
    nodes += [wall('north', 0, 3, 8, .12), wall('south', 0, -3, 8, .12),
              wall('west', -4, 0, .12, 6), wall('east', 4, 0, .12, 6)]
    for i, x in enumerate((-2.0, -0.6, 0.8, 2.2)):
        nodes.append(wall(f'divider {i}', x, -.9 if i % 2 == 0 else .9, .16, 4.2))
    nodes += [actor('crossing west', -3, 0, 0),
              actor('crossing middle', .1, 0, 1.4),
              actor('crossing east', 3.1, 0, 2.6)]
    output = ROOT / 'worlds/dense_obstacles.wbt'
    output.write_text('\n'.join(nodes) + '\n', encoding='utf-8')
    print(output)


if __name__ == '__main__':
    main()
