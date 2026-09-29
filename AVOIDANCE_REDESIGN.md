# Avoidance redesign (v2): DWA-primary static avoidance + progress-based recovery

Branch: `cloud/avoidance-redesign`.  The legacy planner (`avoidance.py`) and
its tests are kept unchanged as the reference implementation.  The new
planner lives in `avoidance_v2.py`; controllers select it with
`AVOIDANCE_PLANNER=v2`.

---------------------------------------------------------------------------

## 1. Legacy architecture as it actually runs

Traced from `avoidance.DynamicWindowAvoidance.choose_action()` and
`main.select_control_command()` (call order and early returns, not intent).

### 1.1 Per-frame priority inside `choose_action()` (first `return` wins)

```
 0  wall-escape cooldown update (0.5 s latch while wall_escape_active)
 1  no ranges                      -> reset everything, STOP
 2  tracker(scan) -> obstacles; crossing-commit odometry progress
 3  sector distances: front (+-10 deg), front arc (+-50 deg), rear;
    side_speed_limit = static side-sector yield timer (v <= 0.08 within 1.0 m)
 4  front-corner latch release (front arc >= 0.75 m and moved >= 0.17 m)
 5  [sensor] crossing supervisor ------------------------------ RETURN
        imminent retreat / clearing dash / yield / gap confirm / commit
 6  BACKOFF  if backoff_steps>0 or static_front<0.28 (rear>0.40):
        seeds recovery_turn_sign (sector medians) + recovery_phase='turn'
        ends on front>=0.45 | rear blocked | 1.5 s (frame count) | corner rule
        curved reverse or corner-escape reverse ---------------- RETURN
 7  if recovery_phase is not None:
        re-run DWA rollout ("escape" check)
        zero-forward trap -> front-corner escape --------------- RETURN
        corner escape + positive rollout -> clear all latches -- RETURN
        3-frame streak of forward candidate -> clear latches --- RETURN
        emergency_turn | wall_follow (_follow_wall) | turn
            -> _validate_recovery_action (4 fallbacks) --------- RETURN
    elif static_front < 0.50 -> start 'turn' recovery ---------- RETURN
 8  DWA rollout (window centred on current_v/current_w)
 9  front_corner_local_minimum (best v<=0.02, no v>0.02 candidate,
    front arc < 0.75) -> front-corner escape ------------------- RETURN
10  no safe candidate: dynamic escape | straight reverse | emergency turn
11  apply best DWA candidate ----------------------------------- RETURN
```

### 1.2 Arbitration in `select_control_command()`

```
if not crossing_commit_active and is_release_ready(nominal):   # 3-frame hysteresis,
    PATH_FOLLOW (nominal: turn in place 1.2 rad/s if |err|>=0.35, else 0.16 m/s)
else:
    avoidance command (final safety stop if unsafe)
is_release_ready() is closed while: recovery_phase in (turn, emergency_turn),
backoff_steps>0, no-safe-candidate flag, wall-escape cooldown>0, corner escape.
```

### 1.3 State that can be active at the same time

`recovery_phase` (turn / emergency_turn / wall_follow), `recovery_turn_sign`,
`wall_side`, `wall_escape_active` (+0.5 s cooldown), `wall_front_turn_sign`,
`wall_clear_steps`, `backoff_steps`, `_front_corner_escape_active` +
`_front_corner_escape_origin`, `_recovery_escape_streak`,
`_path_release_streak`, `_last_rollout_had_no_safe_candidate`,
`side_yield_elapsed` / `side_clear_steps`, `last_turn_sign`, crossing
`waiting / gap_steps / commit_active / remaining / departure_observed`.

### 1.4 Root causes of the observed stalls

1. **Two owners of the wheels.**  Path follower and avoidance arbitrate every
   frame through hysteresis and five latches.  When PATH_FOLLOW drives, DWA's
   `current_v/current_w` are not updated, so the next DWA window is centred on
   a velocity the robot is not actually doing.
2. **Scripted recovery writes the DWA window.**  Pure turns write
   `current_v = 0`; the next window is `v in [0, 0.0192]`.  The corner
   detector requires a candidate with `v > 0.02`, which that window can never
   contain -> "zero-forward trap" -> scripted escape -> `current_v = 0` again.
   The loop is created by a threshold inconsistent with the acceleration
   window, not by the geometry.
3. **Recovery is entered from LiDAR sectors, not from lost progress.**  A
   corner moving in and out of the +-10 deg / +-50 deg sectors during a turn
   starts and stops `turn`, `backoff`, `wall_follow` and `front-corner`
   episodes although DWA may have safe candidates.
4. **Turn direction is re-derived from sector medians** in several places
   (`recovery_turn_sign`, `wall_front_turn_sign`, `last_turn_sign`) with a
   0.75 m switch margin; the signs can disagree and flip.
5. **Greedy Euclidean scoring.**  The sensor score is dominated by
   `-5 * |goal - p(1 s)|`; for a goal behind an obstacle corner the best
   short rollout is always the one that points back into the corner.  Every
   scripted escape is therefore undone by the next normal DWA frame (this is
   also why the earlier `simple_avoidance.py` prototype -- watchdog recovery
   on top of the same greedy score -- still fails the frontal case).
6. **Static side-sector speed cap** (`v <= 0.08 m/s` near any wall for up to
   5 s) further shrinks the window near static geometry.

Measured in this environment (Linux, Webots R2025a, same worlds):

| run | result |
|---|---|
| legacy, minimal Webots (6 scenarios) | 2/6 (front-left, front-right, frontal, corner stall) |
| legacy, 2D harness (18 scenarios) | 6/18 |
| legacy, Webots stress world (12 scenarios) | 4/12 |
| legacy, Extended | stalled at waypoint index 7 (wp 4 needed 44 s); 0 contacts |
| legacy, Practice | 11/11, 0 contacts; `completed=false` only from the crossing benchmark flag |

---------------------------------------------------------------------------

## 2. New architecture (`avoidance_v2.py`)

### 2.1 Layers and ownership

```
 LiDAR + odometry pose + local goal (next waypoint, robot frame)
        |
        v
 DynamicSupervisor.observe()      legacy tracker, unchanged
        |
        v
 DynamicSupervisor.override()  -- moving obstacle owns this frame? --> command
        |  (legacy crossing logic unchanged; static clocks PAUSED,
        |   static state NOT changed)
        v
 Static state machine (single owner, one explicit state):
   NORMAL_DWA | RECOVERY_BACKOFF | RECOVERY_COMMIT
        |
        v
 _apply()/_emit(): shared swept static+dynamic safety veto -> STOP if unsafe
        |          sets current_v/current_w = the command actually returned
        v
 wheel speeds -> RobotIO
```

The path follower no longer produces wheel commands: the waypoint is the
DWA goal.  There is no release arbitration and no hysteresis latch.

### 2.2 States

| state | ENTER | ACTION | EXIT |
|---|---|---|---|
| `NORMAL_DWA` | start; recovery finished | sample dynamic window (forward only), reject with the legacy safety model, score by cost-to-go progress; no admissible candidate -> STOP | watchdog STUCK |
| `RECOVERY_BACKOFF` | STUCK and the reverse trajectory passes the safety model | straight reverse `RECOVERY_BACKOFF_SPEED` | odometry displacement >= `RECOVERY_BACKOFF_DISTANCE`, or reverse becomes unsafe, or timeout -> COMMIT |
| `RECOVERY_COMMIT` | backoff exit, or STUCK with unsafe reverse | turn in the direction chosen **once** at STUCK until the measured heading change >= `RECOVERY_COMMIT_ANGLE`, then forward until odometry distance >= `RECOVERY_COMMIT_FORWARD_DISTANCE`; vetoed turn = STOP (no flip) | commit complete, forward blocked, turn blocked > `RECOVERY_BLOCKED_TIMEOUT`, or timeout -> NORMAL_DWA + watchdog reset + grace |

Direction choice (once per attempt): roll out the left and right primitives
(backoff + committed turn + forward) and compare feasibility (static and
dynamic clearance) > expected cost-to-go progress > minimum clearance;
tie-breakers only: previous direction at this site, last DWA turn sign,
wider side, left.  Repeated STUCK within `RECOVERY_SITE_RADIUS` of the last
recovery (same waypoint) escalates backoff/turn/forward by
`RECOVERY_ESCALATION` (max `RECOVERY_MAX_ESCALATION_STEPS`).

### 2.3 Progress watchdog

STUCK when, in active (non-paused) time, no *progress event* happened for
`WATCHDOG_WINDOW_SECONDS` while the waypoint is outside
`WATCHDOG_GOAL_TOLERANCE`.  A progress event is any of: goal-distance
improvement >= `WATCHDOG_MIN_GOAL_IMPROVEMENT`, translation >=
`WATCHDOG_MIN_DISPLACEMENT`, heading change >= `WATCHDOG_MIN_HEADING_CHANGE`
(so a deliberate slow turn is not STUCK).  A second, longer window
(`WATCHDOG_LONG_*`) ignores heading and small translations to catch in-place
oscillation.  Paused (clock frozen) while the dynamic supervisor owns the
frame or moving obstacles veto otherwise-admissible candidates; reset on a
waypoint change; `WATCHDOG_GRACE_SECONDS` after every recovery.  Commanded
speed and LiDAR sectors play no role.

### 2.4 DWA scoring: obstacle-aware cost-to-go

Admissibility is exactly the legacy model (`_sensor_clearances`: static
clearance over 1.2 s > radius + static margin, dynamic reachable clearance >
radius + dynamic margin, braking distance).  Only the ranking changed:

```
score = 4.0 * progress + 0.4 * heading + 0.6 * clearance + 0.4 * speed + 0.05 * same_turn_sign
progress = (C(robot) - min C(rollout points)) / (v_max * horizon)
heading  = cos(end heading - steepest-descent direction of C at the end point)
```

`C` is a local navigation function (Brock & Khatib's "global dynamic window"
idea; ROS base_local_planner uses the same kind of local cost-to-go):
a world-aligned 0.1 m grid (+-2.5 m) built every frame from the static scan
points only.  Cells within the static safety distance (0.22 m) of a point are
"blocked" (100x traversal cost), cells holding a point are impassable, free
cells with line of sight to the goal get their exact Euclidean distance, and
a Dijkstra wavefront fills the shadows -- bounded to cells the robot can reach
within 1 m.  Rollout tails beyond the 1.2 s safety prefix are cut at the first
blocked cell.  The field never authorises motion; it only removes the greedy
Euclidean local minimum (goal behind a corner, U-shapes, corridor exits).

### 2.5 Removed / replaced legacy mechanisms (in v2 only)

| legacy | v2 |
|---|---|
| front-corner escape state, origin, latch, zero-forward trap detector | none: DWA + cost-to-go; STUCK -> generic recovery |
| `wall_side`, `wall_escape_active` (+cooldown), `wall_front_turn_sign`, `_follow_wall()` | none |
| sector-triggered `turn` / `emergency_turn` / backoff | watchdog-triggered RECOVERY_BACKOFF / RECOVERY_COMMIT |
| static side-sector yield timer / speed cap | none (DWA clearance term) |
| path-follower release arbitration, 3-frame streaks | none: single owner |
| per-frame turn-sign re-selection | direction chosen once per attempt |
| backoff completion by frame count | odometry displacement |

Kept unchanged (reused from `avoidance.py`): LiDAR tracker, crossing yield /
gap confirmation / commit / clearing / imminent retreat, dynamic escape when
no candidate is admissible near movers, the static/dynamic/braking safety
model and every safety constant.

### 2.6 Static admissibility, execution buffer, pass-through waypoints

* **Admissibility = the shared veto rule.**  `_static_admissible` is a
  vectorised copy of `DynamicWindowAvoidance._static_motion_is_safe` (unit
  test: identical on random candidates).  Plain "min clearance > 0.22" would
  reject every candidate once the robot has drifted a millimetre inside the
  margin, including the move-away candidates the veto accepts.
* **Execution buffer (stricter only).**  With `STATIC_DWA_EXECUTION_BUFFER`
  = 1 cm: from a clearance above 0.23 m a candidate must stay above 0.23 m;
  inside the buffer it may not lower the clearance (beyond 2 mm noise).
  Reason: DWA otherwise plans exactly along the 0.22 m boundary; LiDAR noise
  then puts the robot just inside the margin next to a wall, where the legacy
  "must move away" rule admits *nothing* (forward and reverse both approach a
  wall point abeam, in-place rotation is forbidden inside the margin).
  Extended truth run 10 froze 15 s at waypoint 8 in exactly that state.
  The veto itself is unchanged, so recovery can still act inside the buffer.
* **Pass-through waypoints (scoring only).**  A rollout that passes within
  `STATIC_DWA_WAYPOINT_PASS_RADIUS` (0.10 m, below the 0.14 m switch distance)
  of a non-projected waypoint scores as having reached it.  Before, the
  heading term penalised rollouts ending beyond the goal, so DWA braked from
  0.24 to ~0.06 m/s on every approach (Extended: 2-3 s lost per waypoint).

### 2.7 Dynamic layer additions (all stricter than legacy, or tie-break only)

No margin is lowered.  Each addition either rejects more candidates than the
legacy model or only chooses between two options that both already violate a
margin.

| mechanism | where | rule |
|---|---|---|
| braking (inevitable-collision) check | `_braking_gap` | a candidate executed for one step, then braked at the DWA acceleration limits and held for `DYNAMIC_ANTICIPATION_HOLD_SECONDS`, must keep `DYNAMIC_SAFE_DISTANCE` from every *approaching* mover (speed >= 0.10 m/s, closing >= 0.05 m/s).  If the max-braking candidate was clear last frame it is clear now, so the window does not dead-end at cruise speed |
| rollout-tail clearance | `_rollout_tail_clearance` | the 1.2-2.5 s part of the rollout against movers, used in the clearance *score* only |
| dynamic caution speed | `_evaluate_window` | while a tracked mover within crossing range closes on the robot the window is capped at `DYNAMIC_CAUTION_SPEED` (= legacy nominal path speed, which the crossing distances were tuned for) |
| dynamic refuge | `_dynamic_refuge` | no admissible candidate near movers: the statically safe window candidate with the most room (legacy 1.2 s check and braking check) if it beats staying put, else the legacy `_escape_from_dynamic` |
| safer than stopping | `_emit` | a dynamic-layer command failing the dynamic veto is kept only if STOP itself violates the dynamic margin, the command passes the full static check, keeps the emergency clearance and has strictly more clearance than STOP |
| crossing commit deadlock | `DynamicSupervisor.override` | a commit vetoed by *static* geometry for `DYNAMIC_COMMIT_STATIC_BLOCK_SECONDS` is cancelled (a commit vetoed by a mover keeps waiting) |
| static persistence | `StaticPersistenceFilter` | a tracker "mover" is handed to the static layer when >= 80 % of its points lie in world cells occupied for >= 2 s; a real mover's leading cells are always new, so any cluster (span <= 0.55 m) moving at >= 0.055 m/s is never demoted |

The static-persistence filter fixes a tracker artifact (category F): a grazing
wall end gains and loses a few beams frame to frame, its centroid jumps by
~6 cm, the tracker reports ~0.1-0.3 m/s and, once promoted, keeps it as a mover
indefinitely.  Extended truth run 8 stalled 30 s at waypoint 18 (U-turn) in a
yield / gap-confirmation loop in front of that static wall end.

The first version of the braking check stopped the robot at the *end* of the
2.5 s constant-velocity rollout.  At cruise speed every candidate then ended
in an approaching mover's path, the window was empty and the legacy refuge
(no look-ahead) fled along actor A's path ahead of it (truth run 10 contact).

**Explored and rejected: waiving the legacy ego-rotation gate.**  The legacy
tracker never starts a track while the robot turns faster than 0.45 rad/s
(+0.35 s); v2's DWA turns that fast routinely, and actor C was never tracked
during such a turn (truth run 9 contact).  Two waivers were tried, and both
were removed:
* "isolated object" (both cluster ends occluding edges): replayed on the
  Webots traces it tracked C during the turn but added 3-6 % phantom
  cluster-frames (static boxes whose visible faces change while the robot
  turns and drives);
* "moved into space seen free": on the Webots traces the fraction of points
  in recently-seen-free cells does not separate phantoms (90th percentile
  0.4-0.5) from real actors (0 in 30-50 % of their frames).
v2 keeps the legacy tracker unchanged; see Known issues.

---------------------------------------------------------------------------

## 3. Validation (Linux cloud container, Webots R2025a headless)

Every Webots run below uses the unchanged robot, LiDAR, timestep, worlds,
waypoints and obstacles.  Scenario worlds copy the Extended robot node
verbatim (`tools/avoidance_scenarios.py`).  Webots runs are lockstep and
deterministic: the Extended waypoint-4 debug run reproduced the official run
exactly (335.2 s, 27/39).

| suite | v2 | legacy |
|---|---|---|
| unit tests (`python -m unittest discover -s tests`) | 153/155; the 2 failures are the pre-existing legacy `test_front_corner_stall` cases (also failing on the baseline commit); v2 module 64/64 | - |
| 2D closed-loop harness, 18 scenarios | 18/18, 0 contacts, 0 recoveries | 6/18 |
| Webots minimal regression (baseline, front-left, front-right, frontal, corridor, corner) | 6/6, 0 contacts, 0 recoveries, final NORMAL_DWA | 2/6 |
| Webots stress world (12 scenarios incl. crossing, side approach, endurance) | 12/12, 0 contacts, 0 recoveries | 4/12 |
| Practice | 11/11 waypoints, goal, 0 contacts, not stalled, 85.5 s | 11/11, 0 contacts, 104.8 s |
| Extended official (odometry) | 27/39, 1 contact at wp 27 (dynamic C), not stalled, 0 recoveries | 7/39, stalled at wp 7 |
| Extended diagnostic (truth pose) | 27/39, 1 contact at wp 27 (dynamic C) | - |

Minimal regression details (v2): baseline 13.0 s, front-left 15.3 s,
front-right 15.9 s, frontal 17.6 s, corridor 19.8 s, corner 20.2 s; minimum
centre clearance 0.226-0.228 m (corridor 0.53 m).

Practice `completed=false` for **both** planners only because the benchmark
counts a crossing commit that ends below y = 0.90 as "cancel": the v2 commit
ran its full planned distance from y = 0.03 to y = 0.80 (legacy: same flag).
The benchmark was not modified.

Extended waypoint 4 ("slalom1 past obstacle 1", `EXTENDED_DEBUG_WAYPOINT=4`,
378 `[DBG2]` frames with pose, waypoint distance, 1 s displacement, watchdog,
state, current v/w, windows, admissible/forward candidates, selected v/w,
recovery fields, contacts): passed in 12.1 s, NORMAL_DWA throughout,
0 recoveries, max no-progress 0.45 s, min 1 s displacement 0.080 m, min
clearance 0.225 m, 0 contacts, odometry error 2.8 cm.  Waypoints 6 and 7 (the
former odometry-budget stall) are passed; max odometry error 8.6 cm.

Planner time per 32 ms step (practice run, nothing else running): v2 mean
8.1 ms, max 25.2 ms (legacy 4.7 / 13.1 ms).  Extended truth run: 7.2 / 29.4 ms.
Runs made while other Webots instances were running show max values up to
0.6 s from CPU contention; those are not representative.

### 3.1 Unlimited-time Extended diagnostic

`tools/run_avoidance_extended.py --unlimited` (`EXTENDED_TIME_LIMIT=unlimited`)
ignores the official 420 s limit only; contacts and the 35 s stall still end
the run, and the result goes to its own file.  The official limit is
unchanged and is the default.

| run | reached | completed | contacts | stalled | timed_out | runtime |
|---|---|---|---|---|---|---|
| OFFICIAL (420 s) | 27/39 | false | 1 (wp 27, dynamic C) | false | false | 335.2 s |
| UNLIMITED DIAGNOSTIC | 27/39 | false | 1 (wp 27, dynamic C) | false | - | 335.2 s |

The two runs are identical: the collision happens before 420 s, so the time
limit is not what stops the course.  Waypoints 28-38 have not been reached in
any run yet.

## 4. Known issues

1. **Extended dynamic C (fast actor, 0.45 m/s, sweeps almost the whole
   1.28 m corridor, 3.5 s dwell) -> contact** in the official and the truth
   run.  Two legacy mechanisms, both unchanged in v2:
   * the tracker's ego-rotation gate: no new track while the robot turns
     faster than 0.45 rad/s (+0.35 s); DWA turned at 0.6-1.5 rad/s when C
     started, so C was handled as a moving *static* obstacle;
   * the crossing wait is released once a stopped mover's track expires
     (2.5 s); the robot then enters the lane shortly before C resumes.
   Passing C needs a crossing policy with timing and lateral position (cross
   on the side opposite to where C dwells) and a tracker robust to viewpoint
   changes.  Two tracker waivers were evaluated and rejected (section 2.7).
2. **Extended time budget.**  At the measured pace (1.59x the ideal time at
   0.24 m/s) the full course needs about 470 s against the 420 s limit.
3. **Odometry drift** (encoder + compass): up to 8.6 cm in Extended; it limits
   how close to obstacles a waypoint can be approached (category H).
4. The legacy tracker reports static faces seen from a moving viewpoint as
   movers; stationary phantoms are handed back to the static layer after 2 s,
   moving-viewpoint phantoms can still cause a dynamic refuge command
   (always statically safe).
5. Actor D (0.15 m/s) is below the tracker's 0.16 m/s minimum and is handled
   as a static obstacle (legacy behaviour).
6. `LIDAR_SPIKE_*`: confirm on event day that no real obstacle is thinner
   than 2 cm at < 0.28 m.
7. All results are from a Linux cloud container; Windows / local Webots
   re-validation is still required.
8. **Crossing a paused patrol actor (dynamic C) -- root cause analysed, not
   fixed.**  Reproduced in the 2D harness (1.28 m corridor; actor 0.45 m/s
   sweeping y in [-0.40, 0.40], 3.5 s dwell; waypoint on its line): v2
   contact, legacy stall.  Sequence: side-approach retreat, yield, the actor
   pauses at the far end, the legacy gap test keeps waiting for a *straight*
   commit that is never safe (the paused actor is 0.26 m from the straight
   path, dynamic margin 0.32 m), the wait is released only when the track
   expires, and the robot enters the lane ~1.7 s before the actor resumes.
   Planned (not implemented) fix: when a departure was observed and the
   lateral gap/timing tests pass but only the straight commit is unsafe,
   release the wait to DWA (which keeps the dynamic margin and crosses on the
   far side) instead of waiting; the legacy commit stays where it is safe.
9. **DWA heading-term aliasing -- root cause found, fix not validated.**
   The heading score is `cos(end heading - descent)` at the end of the 2.5 s
   rollout; at 1.5-1.8 rad/s a rollout turns more than pi, so a spin that
   wraps a full turn round to the goal scores as aligned (official Extended
   trace t = 330-334 s: goal on the left, robot kept spinning right for about
   2*pi next to actor C).  Proposed fix (unit-checked in isolation only):
   `heading = cos(min(|wrap(descent - theta) - turned|, pi))` with `turned`
   the rollout's unwrapped rotation.  It was reverted because it has not been
   through the full regression.

## 5. How to run

```
AVOIDANCE_PLANNER=v2 python tools/run_avoidance_practice.py
AVOIDANCE_PLANNER=v2 python tools/run_avoidance_extended.py
AVOIDANCE_PLANNER=v2 python tools/run_avoidance_extended.py --unlimited   # DIAGNOSTIC only
AVOIDANCE_PLANNER=v2 EXTENDED_DEBUG_WAYPOINT=4 python tools/run_avoidance_extended.py
EXTENDED_POSE_SOURCE=truth ...          # diagnostic only, never for scoring
python tools/run_avoidance_scenarios.py --world regression --planner v2
python tools/run_avoidance_scenarios.py --world stress --planner v2 --jobs 3
python tools/avoidance_sim.py --planner v2 --case all      # 2D harness
AVOIDANCE_TRACE=/tmp/run.jsonl ...      # record planner inputs in Webots
python tools/avoidance_trace.py /tmp/run.jsonl --from 80 --to 90
python tools/avoidance_scenarios.py     # regenerate the scenario worlds
```

The benchmark controllers still default to the legacy planner
(`AVOIDANCE_PLANNER` unset); `main.py` / `techweek_controller.py` are not
switched yet.
