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

- `select_control_command()` in `main.py` uses `is_release_ready()`, not a bare `is_command_safe()`: control returns to the path follower only after `DWA_PATH_RELEASE_CONFIRM_STEPS` safe frames, while no recovery/backoff/detour is active, and only when the straight line toward the local goal keeps static headroom (a rotate-in-place nominal command is otherwise "safe" but drives back into the obstacle on the next frame).

Behaviour notes for the event integration (see `config.py` for the constants):

- The LiDAR ray angle model follows Webots: a full-turn LiDAR spaces rays by `fov / N` (index `N/2` straight ahead, `3N/4` exactly left). A partial field of view spans both edges with `N-1` intervals. Verify the event sensor's ordering with a probe before trusting sector indices.
- Scoring (not admissibility) ranks rollouts by full-horizon static headroom, penalises turning in place while already facing the goal, and, while an avoidance episode is latched, prefers the goal-relative side of the current blocker. Safety filtering (radius, static/dynamic margins, braking) is unchanged.
- A pose-based progress watchdog (`DWA_PROGRESS_*`) starts a wall-follow detour when the local goal has not come 0.10 m closer for 8 s (crossing waits excluded); the detour ends when the robot is closer than where it started and the goal line is clear. A local planner can still only find short detours: routes that need several metres away from the goal belong to the global planner.
- While avoidance is yielding to a crossing (`crossing_waiting`), `is_release_ready()` keeps control; a mover that stops beside the robot's path (dwelling at its lane end) keeps the wait alive for up to 10 s instead of letting the robot creep into the lane.
- New moving tracks need ray-consistency evidence (a surface vacated space or appeared where earlier rays passed freely); this removes phantom tracks from static boxes seen while driving past them.
- `tools/avoidance_trace.py` records planner inputs when `AVOID_TRACE=<file>` is set in the benchmark/course controllers and replays them offline; `tools/check_course_feasibility.py` checks a world/waypoint set against the collision radius plus static margin; `tools/build_avoidance_courses.py` and `tools/run_avoidance_courses.py` generate and run the minimal and stress course worlds (`worlds/course_*.wbt`).

`main.py` is only a practice adapter. Its `PRACTICE_WAYPOINTS` are a demonstration global path for `worlds/practice.wbt`; connect the team's Mapping/Frontier/A* path selection there. The event world, LiDAR specification, robot geometry, and obstacle size still require validation before claiming mission safety. Current practice Webots runs have recorded contacts, so the collision-free requirement remains open.
