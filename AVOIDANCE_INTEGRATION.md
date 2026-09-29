# Local avoidance integration

`avoidance.py` is independent of Webots and of the practice world's moving-obstacle controllers. Create one `DynamicWindowAvoidance` instance for the robot and call it once per control step:

```python
dwa_left, dwa_right, action = avoidance.choose_action(
    ranges, local_goal, pose=(x, y, yaw), sim_time=seconds
)
normal_left, normal_right = path_follower.compute_control(...)  # team interface
if (
    not avoidance.crossing_commit_active
    and avoidance.is_command_safe(normal_left, normal_right)
):
    left, right = normal_left, normal_right
else:
    left, right = dwa_left, dwa_right
robot_io.set_wheel_speed(left, right)
```

- `ranges`: the current LiDAR distances in metres, ordered as `RobotIO.get_lidar()` returns them. The configured LiDAR field of view and ray order must be checked against the event robot.
- `local_goal`: the next global path waypoint transformed into robot coordinates, in metres. The robot faces local `+x`; local `+y` is left. `main.goal_in_robot_frame()` shows the conversion. Mapping and A* can supply this waypoint without modifying `avoidance.py`.
- `pose`: Localization's current world `(x, y, yaw)` estimate in metres and radians. It is used to match LiDAR clusters and to measure progress until a committed crossing has cleared the crossing zone. The practice `main.py` temporarily obtains this from Webots `Supervisor`; replace that line with Localization output when available.
- `sim_time`: monotonically increasing seconds from the same control loop. It is used to estimate obstacle velocity from recent scans.
- Return value: left and right wheel speeds in radians per second, plus a diagnostic string. Send the wheel speeds through `RobotIO`. The planner does not call motor APIs.
- `is_command_safe(left, right)` checks the Path Follower's proposed wheel speeds against the latest scan. Call `choose_action` every step before this check so the moving-obstacle tracker receives uninterrupted observations. This is a predicted clearance check, not a safety guarantee.
- While `crossing_commit_active` is true, keep the avoidance command in control instead of selecting a separately safe Path Follower command. The commit command still passes through the same static/dynamic safety gate and may stop without losing the commit; only an imminent emergency-envelope collision cancels it.

The planner uses current LiDAR points and recent measured displacement. It does not read an obstacle's scripted trajectory, controller state, or future path. Moving clusters require consecutive observations before their velocity is used. A short reachable region accounts for possible changes in motion; the robot replans after the next scan.

`main.py` is only a practice adapter. Its `PRACTICE_WAYPOINTS` are a demonstration global path for `worlds/practice.wbt`; connect the team's Mapping/Frontier/A* path selection there. The event world, LiDAR specification, robot geometry, and obstacle size still require validation before claiming mission safety. Current practice Webots runs have recorded contacts, so the collision-free requirement remains open.
