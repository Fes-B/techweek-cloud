"""Practice-only evaluator. Ground truth is used for pose and scoring only."""
import json
import math
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from controller import Supervisor
from avoidance import DynamicWindowAvoidance
from config import TIME_STEP
from robot_io import RobotIO
from main import goal_in_robot_frame
from practice_odometry import EncoderPose

# Placeholder global path: replace with the team's global planner at integration.
OUTBOUND = [(-3, 2), (-1.3, 2), (-1.3, -2), (.1, -2),
            (.1, 2), (1.5, 2), (1.5, -2), (3.1, -2), (3.1, 2)]
WAYPOINTS = OUTBOUND + list(reversed(OUTBOUND[:-1])) + [(-3, -2)]
def main():
    print(f'[PYTHON] {sys.version.split()[0]} {sys.executable}', flush=True)
    robot = Supervisor()
    io = RobotIO(robot)
    planner = DynamicWindowAvoidance(lidar_field_of_view=io.lidar.getFov())
    odometry = EncoderPose(robot, (-3.0, -2.0, 0.0))
    node = robot.getSelf()
    node.enableContactPointsTracking(TIME_STEP, True)
    index = 0
    contacts = 0
    touching = False
    best_distance = float('inf')
    progress_time = 0.0
    last_print = -1.0
    compute_total = compute_max = max_position_error = 0.0
    compute_steps = 0
    while robot.step(TIME_STEP) != -1:
        now = robot.getTime()
        p = node.getPosition()
        orientation = node.getOrientation()
        truth = (p[0], p[1], math.atan2(orientation[3], orientation[0]))
        pose = odometry.update()
        max_position_error = max(max_position_error, math.dist(pose[:2], truth[:2]))
        distance = math.dist(pose[:2], WAYPOINTS[index])
        if distance < .14:
            index += 1
            print(f'[WAYPOINT] time={now:.2f} reached={index}/{len(WAYPOINTS)}', flush=True)
            best_distance = float('inf')
            progress_time = now
        # Floor contacts are at z=0. Include wheel-side contacts above 1 cm.
        contact = any(c.getPoint()[2] > .01 for c in node.getContactPoints(True))
        if contact and not touching:
            contacts += 1
            print(f'[CONTACT] time={now:.2f} pose={pose}', flush=True)
        touching = contact
        estimated_complete = index == len(WAYPOINTS)
        complete = estimated_complete and math.dist(truth[:2], WAYPOINTS[-1]) < .25
        stalled = now - progress_time > 45.0
        if estimated_complete or stalled or contacts or now > 360:
            io.stop()
            result = dict(
                completed=complete, contacts=contacts, stalled=stalled,
                seconds=now, reached=index, total=len(WAYPOINTS),
                position_error=math.dist(pose[:2], truth[:2]),
                max_position_error=max_position_error,
                planner_mean_ms=1000 * compute_total / max(1, compute_steps),
                planner_max_ms=1000 * compute_max)
            Path(tempfile.gettempdir(), 'techweek-dense-result.json').write_text(
                json.dumps(result, indent=2), encoding='utf-8')
            print('[RESULT] ' + json.dumps(result), flush=True)
            robot.step(TIME_STEP)
            robot.simulationQuit(0 if complete and not contacts else 1)
            break
        distance = math.dist(pose[:2], WAYPOINTS[index])
        measured_distance = math.dist(truth[:2], WAYPOINTS[index])
        if measured_distance < best_distance - .04:
            best_distance = measured_distance
            progress_time = now
        ranges = io.get_lidar()
        started = time.perf_counter()
        left, right, action = planner.choose_action(
            ranges, goal_in_robot_frame(WAYPOINTS[index], pose), pose, now)
        elapsed = time.perf_counter() - started
        compute_total += elapsed
        compute_max = max(compute_max, elapsed)
        compute_steps += 1
        io.set_wheel_speed(left, right)
        if now - last_print >= 2:
            print(f'[RUN] time={now:.2f} waypoint={index} distance={distance:.2f} '
                  f'pose={pose} truth={truth} action={action}', flush=True)
            last_print = now


if __name__ == '__main__':
    main()
