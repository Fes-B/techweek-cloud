"""DWA-primary local avoidance with a pose-progress watchdog (v2).

The legacy planner in :mod:`avoidance` is kept unchanged as a reference.  This
module reuses its LiDAR tracker, crossing supervisor and collision model, but
replaces its static behaviour with a single explicit state machine::

             STUCK (watchdog)                 backoff done / unsafe
  NORMAL_DWA ----------------> RECOVERY_BACKOFF ------------------> RECOVERY_COMMIT
      ^   \______________________________________________________________/|
      |          STUCK and reverse unsafe                                  |
      +--------------- committed turn + forward done (grace period) ------+

* NORMAL_DWA      -- every frame: sample the dynamic window, reject candidates
                     with the shared static/dynamic/braking safety model, score
                     the rest by obstacle-aware cost-to-go progress.  Exit only
                     when the ProgressWatchdog reports STUCK.
* RECOVERY_BACKOFF -- straight reverse until the robot has *actually* moved
                     RECOVERY_BACKOFF_DISTANCE (odometry).  Skipped, never
                     forced, when the reverse trajectory fails the safety model.
* RECOVERY_COMMIT -- turn in the direction chosen once at STUCK time until the
                     measured heading change reaches the committed angle, then
                     drive forward the committed distance.  The direction never
                     flips inside an attempt; a vetoed command becomes STOP.

Dynamic obstacles are handled by :class:`DynamicSupervisor`, an orthogonal
layer.  It may override a frame (yield, crossing commit, imminent retreat) and
it pauses the watchdog and recovery clocks, but it never changes the static
state.  Every command from every layer passes the same final safety veto.

Coordinates: pose is world ``(x, y, theta)`` in metres/radians; ``goal`` is the
path waypoint in robot coordinates (+x forward, +y left).  Wheel speeds are in
rad/s, as in :mod:`avoidance`.
"""

import enum
import heapq
import math

import numpy as np

from avoidance import DynamicWindowAvoidance
from config import (
    DWA_ANGULAR_ACCELERATION,
    DWA_CROSSING_COMMIT_SPEED,
    DWA_CROSSING_DETECTION_DISTANCE,
    DWA_DYNAMIC_CLEARANCE_MARGIN,
    DWA_EMERGENCY_CLEARANCE_MARGIN,
    DWA_HORIZON,
    DWA_LINEAR_ACCELERATION,
    DWA_MAX_ANGULAR_SPEED,
    DWA_ROBOT_RADIUS,
    DWA_SIMULATION_STEP,
    DWA_STATIC_CLEARANCE_MARGIN,
    DWA_VELOCITY_SAMPLES,
    DWA_YAW_RATE_SAMPLES,
    DYNAMIC_ANTICIPATION_HOLD_SECONDS,
    DYNAMIC_ANTICIPATION_MIN_SPEED,
    DYNAMIC_CAUTION_MIN_CLOSING_SPEED,
    DYNAMIC_CAUTION_SPEED,
    DYNAMIC_COMMIT_STATIC_BLOCK_SECONDS,
    DYNAMIC_STATIC_PERSISTENCE_DROPOUT,
    DYNAMIC_STATIC_PERSISTENCE_FRACTION,
    DYNAMIC_STATIC_PERSISTENCE_RESOLUTION,
    DYNAMIC_STATIC_PERSISTENCE_SECONDS,
    LIDAR_FIELD_OF_VIEW,
    LIDAR_SPIKE_GAP,
    LIDAR_SPIKE_MIN_OBJECT_WIDTH,
    MAX_SPEED,
    NAV_FIELD_ANGLE_BINS,
    NAV_FIELD_DETOUR_EPSILON,
    NAV_FIELD_GOAL_EPSILON,
    NAV_FIELD_GOAL_SEARCH_RADIUS,
    NAV_FIELD_HALF_EXTENT,
    NAV_FIELD_INFLATED_COST,
    NAV_FIELD_INFLATION,
    NAV_FIELD_LOCAL_RADIUS,
    NAV_FIELD_NEAR_GOAL,
    NAV_FIELD_RESOLUTION,
    RECOVERY_BACKOFF_DISTANCE,
    RECOVERY_BACKOFF_SPEED,
    RECOVERY_BACKOFF_TIMEOUT,
    RECOVERY_BLOCKED_TIMEOUT,
    RECOVERY_COMMIT_ANGLE,
    RECOVERY_COMMIT_FORWARD_DISTANCE,
    RECOVERY_COMMIT_FORWARD_SPEED,
    RECOVERY_COMMIT_TIMEOUT,
    RECOVERY_COMMIT_YAW_RATE,
    RECOVERY_DIRECTION_CLEARANCE_TIE,
    RECOVERY_DIRECTION_PROGRESS_TIE,
    RECOVERY_ESCALATION,
    RECOVERY_MAX_ESCALATION_STEPS,
    RECOVERY_SITE_RADIUS,
    STATIC_DWA_CLEARANCE_SCALE,
    STATIC_DWA_BUFFER_TOLERANCE,
    STATIC_DWA_CLEARANCE_WEIGHT,
    STATIC_DWA_EXECUTION_BUFFER,
    STATIC_DWA_HEADING_WEIGHT,
    STATIC_DWA_MAX_SPEED,
    STATIC_DWA_PROGRESS_WEIGHT,
    STATIC_DWA_SPEED_WEIGHT,
    STATIC_DWA_TURN_HYSTERESIS_WEIGHT,
    STATIC_DWA_WAYPOINT_PASS_RADIUS,
    TIME_STEP,
    WATCHDOG_GOAL_TOLERANCE,
    WATCHDOG_GRACE_SECONDS,
    WATCHDOG_LONG_MIN_DISPLACEMENT,
    WATCHDOG_LONG_MIN_GOAL_IMPROVEMENT,
    WATCHDOG_LONG_WINDOW_SECONDS,
    WATCHDOG_MIN_DISPLACEMENT,
    WATCHDOG_MIN_GOAL_IMPROVEMENT,
    WATCHDOG_MIN_HEADING_CHANGE,
    WATCHDOG_PROJECTED_GOAL_TOLERANCE,
    WATCHDOG_WAYPOINT_JUMP,
    WATCHDOG_WINDOW_SECONDS,
    WHEEL_RADIUS,
    WHEEL_TRACK,
)


STATIC_SAFE_DISTANCE = DWA_ROBOT_RADIUS + DWA_STATIC_CLEARANCE_MARGIN
DYNAMIC_SAFE_DISTANCE = DWA_ROBOT_RADIUS + DWA_DYNAMIC_CLEARANCE_MARGIN
EMERGENCY_SAFE_DISTANCE = DWA_ROBOT_RADIUS + DWA_EMERGENCY_CLEARANCE_MARGIN
_ROLLOUT_STEPS = max(1, math.ceil(DWA_HORIZON / DWA_SIMULATION_STEP))
# Same prefix that DynamicWindowAvoidance._sensor_clearances checks statically.
_SAFETY_STEPS = max(1, round(1.2 / DWA_SIMULATION_STEP))
_MAX_PROGRESS = STATIC_DWA_MAX_SPEED * DWA_HORIZON
_MAX_CONTROL_DT = 0.25  # ignore larger sim_time gaps (pauses, resets)


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _sample_range(low, high, count):
    if count <= 1 or high <= low:
        return [low]
    step = (high - low) / (count - 1)
    return [low + index * step for index in range(count)]


def _to_world(point, pose):
    x, y, theta = pose
    cosine, sine = math.cos(theta), math.sin(theta)
    return (x + cosine * point[0] - sine * point[1],
            y + sine * point[0] + cosine * point[1])


def _velocity_from_wheels(left, right):
    return (0.5 * WHEEL_RADIUS * (left + right),
            WHEEL_RADIUS * (right - left) / WHEEL_TRACK)


def filter_isolated_returns(ranges, field_of_view=LIDAR_FIELD_OF_VIEW,
                            gap=LIDAR_SPIKE_GAP,
                            min_object_width=LIDAR_SPIKE_MIN_OBJECT_WIDTH):
    """Drop single-beam returns that no physical obstacle can produce.

    A return is removed (set to inf) only when (a) the nearest finite
    neighbour within two beams on *each* side is farther by more than ``gap``
    (or absent), and (b) it is so close that an object seen by only one beam
    would be narrower than ``min_object_width``.  Object edges keep one
    consistent neighbour and objects wider than that are hit by at least two
    beams, so both are always kept.  Returns the filtered list and the number
    of removed returns.
    """
    count = len(ranges)
    if count < 3:
        return list(ranges), 0
    spacing = field_of_view / (count - 1)
    max_range = min_object_width / (2.0 * spacing)
    wraps = field_of_view >= 2.0 * math.pi - 1e-3
    filtered = list(ranges)
    removed = 0
    for index, value in enumerate(ranges):
        if not (math.isfinite(value) and value < max_range):
            continue
        isolated = True
        for direction in (-1, 1):
            neighbour = None
            for offset in (1, 2):
                other = index + direction * offset
                if wraps:
                    other %= count
                elif not 0 <= other < count:
                    break
                if math.isfinite(ranges[other]):
                    neighbour = ranges[other]
                    break
            if neighbour is not None and neighbour - value <= gap:
                isolated = False
                break
        if isolated:
            filtered[index] = math.inf
            removed += 1
    return filtered, removed


class StaticAvoidanceState(enum.Enum):
    NORMAL_DWA = "NORMAL_DWA"
    RECOVERY_BACKOFF = "RECOVERY_BACKOFF"
    RECOVERY_COMMIT = "RECOVERY_COMMIT"


# ---------------------------------------------------------------------------
# Progress watchdog
# ---------------------------------------------------------------------------
class ProgressWatchdog:
    """Decide STUCK from pose history, never from LiDAR sectors or commands.

    Two anchors hold the pose at the last "progress event".  The short anchor
    moves on any goal-distance improvement, translation or heading change
    above its thresholds; the long anchor only on larger translation or goal
    improvement (it is what catches in-place oscillation).  STUCK is reported
    when either anchor is older than its window.  Time is *active* time: it
    only advances in update() calls with ``paused=False``.
    """

    def __init__(self, window=WATCHDOG_WINDOW_SECONDS,
                 min_goal_improvement=WATCHDOG_MIN_GOAL_IMPROVEMENT,
                 min_displacement=WATCHDOG_MIN_DISPLACEMENT,
                 min_heading_change=WATCHDOG_MIN_HEADING_CHANGE,
                 goal_tolerance=WATCHDOG_GOAL_TOLERANCE,
                 long_window=WATCHDOG_LONG_WINDOW_SECONDS,
                 long_min_goal_improvement=WATCHDOG_LONG_MIN_GOAL_IMPROVEMENT,
                 long_min_displacement=WATCHDOG_LONG_MIN_DISPLACEMENT):
        self.window = window
        self.min_goal_improvement = min_goal_improvement
        self.min_displacement = min_displacement
        self.min_heading_change = min_heading_change
        self.goal_tolerance = goal_tolerance
        self.long_window = long_window
        self.long_min_goal_improvement = long_min_goal_improvement
        self.long_min_displacement = long_min_displacement
        self.clock = 0.0
        self.grace_remaining = 0.0
        self._short = None  # (clock, x, y, theta, goal_distance)
        self._long = None
        self.stuck_reason = None

    def reset(self):
        self._short = None
        self._long = None
        self.stuck_reason = None

    def start_grace(self, seconds=WATCHDOG_GRACE_SECONDS):
        self.reset()
        self.grace_remaining = seconds

    @property
    def no_progress_seconds(self):
        return 0.0 if self._short is None else self.clock - self._short[0]

    @property
    def long_no_progress_seconds(self):
        return 0.0 if self._long is None else self.clock - self._long[0]

    @property
    def paused_or_idle(self):
        return self._short is None

    def update(self, dt, pose, goal_distance, paused=False, tolerance=None):
        """Advance by ``dt`` active seconds and return True when STUCK."""
        self.stuck_reason = None
        if paused:
            return False
        self.clock += max(0.0, dt)
        sample = (self.clock, pose[0], pose[1], pose[2], goal_distance)
        if self.grace_remaining > 0.0:
            self.grace_remaining = max(0.0, self.grace_remaining - max(0.0, dt))
            self._short = self._long = sample
            return False
        if goal_distance <= (self.goal_tolerance if tolerance is None else tolerance):
            self._short = self._long = sample
            return False
        if self._short is None:
            self._short = self._long = sample
            return False

        _, sx, sy, stheta, sgoal = self._short
        if (
            sgoal - goal_distance >= self.min_goal_improvement
            or math.hypot(pose[0] - sx, pose[1] - sy) >= self.min_displacement
            or abs(_wrap(pose[2] - stheta)) >= self.min_heading_change
        ):
            self._short = sample
        _, lx, ly, _, lgoal = self._long
        if (
            lgoal - goal_distance >= self.long_min_goal_improvement
            or math.hypot(pose[0] - lx, pose[1] - ly) >= self.long_min_displacement
        ):
            self._long = sample

        if self.clock - self._short[0] >= self.window - 1e-9:
            self.stuck_reason = "no pose progress"
        elif self.clock - self._long[0] >= self.long_window - 1e-9:
            self.stuck_reason = "no net progress (oscillation)"
        return self.stuck_reason is not None


# ---------------------------------------------------------------------------
# Obstacle-aware cost-to-go (DWA scoring aid)
# ---------------------------------------------------------------------------
class CostToGoField:
    """Distance-to-goal around the obstacles visible in the current scan.

    A world-aligned grid (fixed lattice, so cells do not jitter as the robot
    turns) is centred on the robot.  A cell is *blocked* when its centre lies
    within NAV_FIELD_INFLATION (the DWA static safety distance) of a static
    LiDAR point -- exact point distance, no snapping -- and *occupied* when it
    contains a point.  Blocked cells are traversable only at
    NAV_FIELD_INFLATED_COST times their length (a last resort, e.g. when the
    goal itself lies inside the inflation); occupied cells never.

    Free cells with line of sight to the goal get their exact Euclidean
    distance.  A Dijkstra wavefront fills shadowed free cells, but only until
    every shadowed cell the robot can reach within NAV_FIELD_LOCAL_RADIUS
    (without crossing blocked cells) is settled: rollouts never leave that
    region, so the rest of the grid is irrelevant to this frame's decision.
    Space outside the grid and unobserved space are treated as free.

    This field only ranks trajectories.  Whether a trajectory may be executed
    is decided exclusively by the swept-clearance safety model.
    """

    _SQRT2 = math.sqrt(2.0)

    def __init__(self, resolution=NAV_FIELD_RESOLUTION,
                 half_extent=NAV_FIELD_HALF_EXTENT,
                 inflation=NAV_FIELD_INFLATION,
                 inflated_cost=NAV_FIELD_INFLATED_COST,
                 angle_bins=NAV_FIELD_ANGLE_BINS,
                 local_radius=NAV_FIELD_LOCAL_RADIUS):
        self.resolution = resolution
        self.half = int(math.ceil(half_extent / resolution))
        self.size = 2 * self.half + 1
        self.inflation = inflation
        self.inflated_cost = inflated_cost
        self.angle_bins = angle_bins
        self.local_cells = int(math.ceil(local_radius / resolution))
        reach = int(math.ceil(inflation / resolution + 0.7072))
        self._inflation_offsets = np.array(
            [(dy, dx) for dy in range(-reach, reach + 1) for dx in range(-reach, reach + 1)],
            dtype=int,
        )
        n = self.size
        self._neighbours = (
            (0, 1, 1.0), (0, -1, 1.0), (1, 0, 1.0), (-1, 0, 1.0),
            (1, 1, self._SQRT2), (1, -1, self._SQRT2),
            (-1, 1, self._SQRT2), (-1, -1, self._SQRT2),
        )
        self._neighbour_shifts = [(dy, dx) for dy, dx, _ in self._neighbours]
        self._neighbour_offsets = tuple(
            (dy * n + dx, dy, dx, step * resolution) for dy, dx, step in self._neighbours
        )
        self.goal = None
        self.requested_goal = None
        self.goal_projected = False
        self.cost = None
        self.origin_index = (0, 0)
        self.blocked = None
        self.occupied = None
        self.settled_cells = 0

    # -- construction -----------------------------------------------------
    @staticmethod
    def _dilate(mask, offsets):
        n_rows, n_cols = mask.shape
        out = mask.copy()
        for dy, dx in offsets:
            if dy == 0 and dx == 0:
                continue
            out[max(dy, 0):n_rows + min(dy, 0), max(dx, 0):n_cols + min(dx, 0)] |= (
                mask[max(-dy, 0):n_rows + min(-dy, 0), max(-dx, 0):n_cols + min(-dx, 0)]
            )
        return out

    def _rasterise(self, points_world, col0, row0):
        res = self.resolution
        n = self.size
        occupied = np.zeros((n, n), dtype=bool)
        blocked = np.zeros((n, n), dtype=bool)
        if not len(points_world):
            return occupied, blocked
        points = np.asarray(points_world, dtype=float)
        cols = np.rint(points[:, 0] / res).astype(int) - col0
        rows = np.rint(points[:, 1] / res).astype(int) - row0
        inside = (cols >= 0) & (cols < n) & (rows >= 0) & (rows < n)
        occupied[rows[inside], cols[inside]] = True
        cand_rows = rows[:, None] + self._inflation_offsets[None, :, 0]
        cand_cols = cols[:, None] + self._inflation_offsets[None, :, 1]
        distance = np.hypot((col0 + cand_cols) * res - points[:, 0:1],
                            (row0 + cand_rows) * res - points[:, 1:2])
        mark = ((distance <= self.inflation)
                & (cand_rows >= 0) & (cand_rows < n) & (cand_cols >= 0) & (cand_cols < n))
        blocked[cand_rows[mark], cand_cols[mark]] = True
        return occupied, blocked | occupied

    def build(self, points_world, goal_world, center_world):
        res = self.resolution
        n = self.size
        col0 = int(round(center_world[0] / res)) - self.half
        row0 = int(round(center_world[1] / res)) - self.half
        self.origin_index = (col0, row0)
        self.requested_goal = (float(goal_world[0]), float(goal_world[1]))
        self.goal, self.goal_projected = self._project_goal(
            points_world, self.requested_goal, center_world
        )
        occupied, blocked = self._rasterise(points_world, col0, row0)
        self.occupied = occupied
        self.blocked = blocked

        indices = np.arange(n)
        grid_x, grid_y = np.meshgrid((col0 + indices) * res, (row0 + indices) * res)
        dx = grid_x - self.goal[0]
        dy = grid_y - self.goal[1]
        distance = np.hypot(dx, dy)
        visible = self._visible_from_goal(distance, np.arctan2(dy, dx), blocked)

        cost = np.where(visible, distance, np.inf)
        multiplier = np.where(occupied, np.inf,
                              np.where(blocked, self.inflated_cost, 1.0))
        required = self._local_reachable(blocked, occupied) & ~visible & ~blocked
        seeds = []
        if not visible.any():
            # Nothing sees the goal (e.g. it is enclosed): grow from the goal
            # cell itself so the field never degenerates to a flat plateau.
            goal_col = int(round(self.goal[0] / res)) - col0
            goal_row = int(round(self.goal[1] / res)) - row0
            if 0 <= goal_col < n and 0 <= goal_row < n and not occupied[goal_row, goal_col]:
                seeds.append((0.0, goal_row * n + goal_col))
        cost = self._fill_shadows(cost, visible, multiplier, required, seeds)
        self.cost = self._relax_blocked(cost, blocked)
        finite = np.isfinite(self.cost)
        ceiling = (float(self.cost[finite].max()) if finite.any() else 0.0) + 5.0
        self._filled = np.where(finite, self.cost, ceiling)
        grad_y, grad_x = np.gradient(self._filled, res)
        self._grad_x = grad_x
        self._grad_y = grad_y

    def _project_goal(self, points_world, goal, center):
        """Nearest point to ``goal`` that keeps the static safety distance.

        A waypoint closer than NAV_FIELD_INFLATION to a scan point cannot be
        reached by an admissible trajectory; guiding DWA to the nearest
        admissible point instead keeps the field informative.  Returns
        (goal, projected).

        First the goal is pushed radially out of the safety disk of its
        nearest point, repeatedly (exact for a corner or a straight wall).
        If that does not converge (concave pockets), a ring search around
        the goal is used, refined to millimetres along the best direction;
        among near-equal candidates the one on the robot's side wins.
        """
        if not len(points_world):
            return goal, False
        points = np.asarray(points_world, dtype=float)
        required = self.inflation + NAV_FIELD_GOAL_EPSILON
        reach = required + NAV_FIELD_GOAL_SEARCH_RADIUS
        near = points[np.hypot(points[:, 0] - goal[0], points[:, 1] - goal[1]) <= reach]
        if not len(near):
            return goal, False

        def clearance(x, y):
            return np.hypot(np.asarray(x)[..., None] - near[:, 0],
                            np.asarray(y)[..., None] - near[:, 1]).min(axis=-1)

        if float(clearance(goal[0], goal[1])) >= self.inflation:
            return goal, False
        qx, qy = goal
        for _ in range(12):
            distance = np.hypot(near[:, 0] - qx, near[:, 1] - qy)
            index = int(np.argmin(distance))
            if distance[index] >= required - 1e-6:
                if math.hypot(qx - goal[0], qy - goal[1]) <= NAV_FIELD_GOAL_SEARCH_RADIUS:
                    return (float(qx), float(qy)), True
                break
            px, py = near[index]
            if distance[index] < 1e-9:
                break
            scale = required / distance[index]
            qx, qy = px + (qx - px) * scale, py + (qy - py) * scale

        radii = np.arange(1, int(NAV_FIELD_GOAL_SEARCH_RADIUS / 0.01) + 1) * 0.01
        angles = np.linspace(0.0, 2.0 * math.pi, 96, endpoint=False)
        cx = goal[0] + radii[:, None] * np.cos(angles)[None, :]
        cy = goal[1] + radii[:, None] * np.sin(angles)[None, :]
        admissible = clearance(cx, cy) >= required
        if not admissible.any():
            return goal, False
        score = radii[:, None] + 0.1 * np.hypot(cx - center[0], cy - center[1])
        score = np.where(admissible, score, np.inf)
        row, col = np.unravel_index(int(np.argmin(score)), score.shape)
        low, high = (radii[row - 1] if row else 0.0), radii[row]
        direction = (math.cos(angles[col]), math.sin(angles[col]))
        for _ in range(8):  # bisection: smallest admissible radius on that ray
            middle = 0.5 * (low + high)
            if float(clearance(goal[0] + middle * direction[0],
                               goal[1] + middle * direction[1])) >= required:
                high = middle
            else:
                low = middle
        return (goal[0] + high * direction[0], goal[1] + high * direction[1]), True

    def _local_reachable(self, blocked, occupied):
        """Free cells within the local radius connected to the robot cell."""
        n = self.size
        centre = self.half
        k = self.local_cells
        low, high = max(0, centre - k), min(n, centre + k + 1)
        window_blocked = blocked[low:high, low:high]
        rr, cc = np.mgrid[low:high, low:high]
        inside = (rr - centre) ** 2 + (cc - centre) ** 2 <= k * k
        passable = inside & ~window_blocked
        seed = np.zeros_like(passable)
        c = centre - low
        seed[max(0, c - 1):c + 2, max(0, c - 1):c + 2] = True
        reach = seed & passable
        if not reach.any():
            # Robot squeezed inside the inflation: grow through non-occupied cells.
            passable = inside & ~occupied[low:high, low:high]
            reach = seed & passable
        while True:
            grown = self._dilate(reach, self._neighbour_shifts) & passable
            if np.array_equal(grown, reach):
                break
            reach = grown
        out = np.zeros((n, n), dtype=bool)
        out[low:high, low:high] = reach
        return out

    def _visible_from_goal(self, distance, angle, blocked):
        """Shadow casting from the goal over blocked cells (polar binning)."""
        res = self.resolution
        cell_radius = 0.7072 * res
        occluders = blocked & (distance > res)
        visible = ~blocked
        if not occluders.any():
            return visible
        bins = self.angle_bins
        width = 2.0 * math.pi / bins
        occluder_distance = distance[occluders]
        occluder_angle = angle[occluders]
        half_angle = np.arcsin(np.clip(cell_radius / occluder_distance, 0.0, 1.0))
        low = np.floor((occluder_angle - half_angle + math.pi) / width).astype(int)
        high = np.floor((occluder_angle + half_angle + math.pi) / width).astype(int)
        span = high - low + 1
        starts = np.repeat(low, span)
        offsets = np.arange(int(span.sum())) - np.repeat(np.cumsum(span) - span, span)
        nearest = np.full(bins, np.inf)
        np.minimum.at(
            nearest,
            (starts + offsets) % bins,
            np.repeat(occluder_distance - cell_radius, span),
        )
        cell_bins = np.floor((angle + math.pi) / width).astype(int) % bins
        return visible & (distance <= nearest[cell_bins])

    def _fill_shadows(self, cost, visible, multiplier, required, seeds=()):
        """Dijkstra from the visible frontier until all required cells settle."""
        n = self.size
        remaining = set(np.flatnonzero(required).tolist())
        self.settled_cells = 0
        if not remaining:
            return cost
        flat_cost = cost.ravel().tolist()
        flat_multiplier = multiplier.ravel().tolist()
        frontier = visible & self._dilate(~visible, self._neighbour_shifts)
        heap = [(flat_cost[index], int(index)) for index in np.flatnonzero(frontier)]
        for value, index in seeds:
            flat_cost[index] = min(flat_cost[index], value)
            heap.append((flat_cost[index], index))
        heapq.heapify(heap)
        pop = heapq.heappop
        push = heapq.heappush
        offsets = self._neighbour_offsets
        settled = 0
        while heap and remaining:
            value, index = pop(heap)
            if value > flat_cost[index]:
                continue
            settled += 1
            remaining.discard(index)
            row, col = divmod(index, n)
            for offset, dy, dx, step in offsets:
                r = row + dy
                c = col + dx
                if r < 0 or r >= n or c < 0 or c >= n:
                    continue
                neighbour = index + offset
                weight = flat_multiplier[neighbour]
                if weight == math.inf:
                    continue
                candidate = value + step * weight
                if candidate < flat_cost[neighbour]:
                    flat_cost[neighbour] = candidate
                    push(heap, (candidate, neighbour))
        self.settled_cells = settled
        return np.asarray(flat_cost, dtype=float).reshape(n, n)

    def _relax_blocked(self, cost, blocked):
        """Extend the free-space field one cell into the blocked band.

        Only free neighbours contribute (plain step length, no inflation
        penalty), so interpolating at a robot that legitimately sits at the
        safety distance sees a smooth continuation of the free-side field
        instead of the routing penalty.  Deeper blocked cells keep their
        (penalised) Dijkstra value or fall back to the ceiling.
        """
        n = self.size
        res = self.resolution
        free_cost = np.where(blocked, np.inf, cost)
        best = np.full((n, n), np.inf)
        for dy, dx, step in self._neighbours:
            shifted = np.full((n, n), np.inf)
            shifted[max(dy, 0):n + min(dy, 0), max(dx, 0):n + min(dx, 0)] = (
                free_cost[max(-dy, 0):n + min(-dy, 0), max(-dx, 0):n + min(-dx, 0)]
            )
            best = np.minimum(best, shifted + step * res)
        return np.where(blocked & np.isfinite(best), best, cost)

    # -- queries ------------------------------------------------------------
    def _fractional_index(self, x, y):
        col0, row0 = self.origin_index
        fx = np.clip(np.asarray(x, dtype=float) / self.resolution - col0, 0.0, self.size - 1.0)
        fy = np.clip(np.asarray(y, dtype=float) / self.resolution - row0, 0.0, self.size - 1.0)
        return fx, fy

    def _bilinear(self, grid, x, y):
        fx, fy = self._fractional_index(x, y)
        c0 = np.minimum(np.floor(fx).astype(int), self.size - 2)
        r0 = np.minimum(np.floor(fy).astype(int), self.size - 2)
        tx = fx - c0
        ty = fy - r0
        return (
            grid[r0, c0] * (1 - tx) * (1 - ty)
            + grid[r0, c0 + 1] * tx * (1 - ty)
            + grid[r0 + 1, c0] * (1 - tx) * ty
            + grid[r0 + 1, c0 + 1] * tx * ty
        )

    def cost_at(self, x, y):
        """Cost-to-go; exact Euclidean within NAV_FIELD_NEAR_GOAL of the goal.

        Bilinear interpolation smooths the cone apex at the goal, which left
        DWA without a usable signal for the last few centimetres.  The goal
        region is free (the goal is projected out of the inflation), so the
        exact distance is the true cost there.
        """
        cost = self._bilinear(self._filled, x, y)
        gx, gy = self.goal
        near = np.hypot(np.asarray(x) - gx, np.asarray(y) - gy)
        return np.where(near < NAV_FIELD_NEAR_GOAL, near, cost)

    def blocked_at(self, x, y):
        """Nearest-cell lookup of the blocked (inflated) mask."""
        fx, fy = self._fractional_index(x, y)
        return self.blocked[np.rint(fy).astype(int), np.rint(fx).astype(int)]

    def descent_heading_at(self, x, y):
        gx = self._bilinear(self._grad_x, x, y)
        gy = self._bilinear(self._grad_y, x, y)
        heading = np.arctan2(-gy, -gx)
        # Near the goal, or where the field is flat, head for the goal itself.
        dx = self.goal[0] - np.asarray(x)
        dy = self.goal[1] - np.asarray(y)
        use_goal = (np.hypot(gx, gy) < 1e-6) | (np.hypot(dx, dy) < NAV_FIELD_NEAR_GOAL)
        return np.where(use_goal, np.arctan2(dy, dx), heading)


# ---------------------------------------------------------------------------
# Dynamic supervisor (adapter around the unchanged legacy crossing logic)
# ---------------------------------------------------------------------------
class StaticPersistenceFilter:
    """Hands tracker "movers" that have not actually moved to the static layer.

    Keeps, per world cell, the time since which the cell has been hit by the
    scan without a gap longer than ``dropout``.  A cluster the tracker labels
    as moving is demoted to static points when at least ``fraction`` of its
    points lie in cells occupied for ``static_seconds`` or longer.  The tracker
    state itself is not touched, so a demoted cluster that starts to move is a
    mover again as soon as its points reach new cells.
    """

    def __init__(self, resolution=DYNAMIC_STATIC_PERSISTENCE_RESOLUTION,
                 static_seconds=DYNAMIC_STATIC_PERSISTENCE_SECONDS,
                 dropout=DYNAMIC_STATIC_PERSISTENCE_DROPOUT,
                 fraction=DYNAMIC_STATIC_PERSISTENCE_FRACTION):
        self.resolution = resolution
        self.static_seconds = static_seconds
        self.dropout = dropout
        self.fraction = fraction
        self._first_seen = {}
        self._last_seen = {}
        self._time = None

    def reset(self):
        self._first_seen.clear()
        self._last_seen.clear()
        self._time = None

    def filter(self, points, obstacles, pose, sim_time):
        """Return ``(obstacles, demoted_clusters)``.

        ``points`` are the robot-frame scan points that ``obstacles`` was
        built from, index-aligned; any other ``obstacles`` (the tracker's
        held-track output for an empty scan) is returned unchanged.
        """
        if self._time is not None and sim_time < self._time:
            self.reset()
        self._time = sim_time
        cosine, sine = math.cos(pose[2]), math.sin(pose[2])
        cells = [
            (math.floor((pose[0] + cosine * x - sine * y) / self.resolution),
             math.floor((pose[1] + sine * x + cosine * y) / self.resolution))
            for x, y in points
        ]
        for cell in set(cells):
            last = self._last_seen.get(cell)
            if last is None or sim_time - last > self.dropout:
                self._first_seen[cell] = sim_time
            self._last_seen[cell] = sim_time
        for cell in [c for c, t in self._last_seen.items() if sim_time - t > self.dropout]:
            del self._last_seen[cell]
            del self._first_seen[cell]

        if len(points) != len(obstacles):
            return obstacles, 0
        clusters = {}
        for index, obstacle in enumerate(obstacles):
            if len(obstacle) == 4:  # the tracker gives a cluster one velocity
                clusters.setdefault((obstacle[2], obstacle[3]), []).append(index)
        if not clusters:
            return obstacles, 0
        result = list(obstacles)
        demoted = 0
        for indices in clusters.values():
            persistent = sum(
                1 for index in indices
                if sim_time - self._first_seen[cells[index]] >= self.static_seconds
            )
            if persistent >= self.fraction * len(indices):
                for index in indices:
                    result[index] = obstacles[index][:2]
                demoted += 1
        return result, demoted


class DynamicSupervisor:
    """Moving-obstacle layer, orthogonal to the static state machine.

    Wraps the legacy tracker and crossing logic of
    :class:`avoidance.DynamicWindowAvoidance` without changing them: moving
    cluster tracking, side-approach retreat, crossing yield, gap confirmation,
    crossing commit and clearing.  It never activates static recovery.
    """

    def __init__(self, toolkit):
        self._toolkit = toolkit
        self.persistence = StaticPersistenceFilter()
        self.commit_static_block = 0.0
        self.commits_cancelled = 0
        self.demoted_clusters = 0

    @property
    def commit_active(self):
        return self._toolkit.crossing_commit_active

    @property
    def waiting(self):
        return self._toolkit.crossing_waiting

    def reset(self):
        toolkit = self._toolkit
        toolkit.previous_scan_clusters = None
        toolkit.previous_cluster_histories = []
        toolkit.previous_scan_time = None
        toolkit.previous_scan_heading = None
        toolkit.last_dynamic_obstacles = []
        toolkit.last_obstacles = None
        toolkit._reset_crossing_state()
        self.persistence.reset()

    def observe(self, ranges, pose, sim_time):
        toolkit = self._toolkit
        obstacles = toolkit._scan_with_dynamic_obstacles(ranges, pose, sim_time)
        points = toolkit._scan_to_points(ranges, toolkit.lidar_field_of_view, toolkit.lidar_pose)
        obstacles, self.demoted_clusters = self.persistence.filter(
            points, obstacles, pose, sim_time
        )
        toolkit.last_dynamic_obstacles = [point for point in obstacles if len(point) == 4]
        toolkit.last_obstacles = obstacles
        toolkit._update_crossing_commit_progress(pose)
        return obstacles

    def override(self, moving, obstacles, rear_distance, pose, current_v, current_w, dt=0.0):
        """Return a wheel command when a moving obstacle owns this frame.

        The only addition to the legacy logic: a crossing commit that stays
        vetoed by static geometry for DYNAMIC_COMMIT_STATIC_BLOCK_SECONDS is
        cancelled and the frame is handed back to the static layer.
        """
        toolkit = self._toolkit
        toolkit.current_v = current_v
        toolkit.current_w = current_w
        action = toolkit._evaluate_crossing_hazard(moving, rear_distance, obstacles, pose)
        if not (toolkit.crossing_commit_active and action is not None and action[:2] == (0.0, 0.0)):
            self.commit_static_block = 0.0
            return action
        fixed = [point for point in obstacles if len(point) == 2]
        if toolkit._velocity_is_safe(DWA_CROSSING_COMMIT_SPEED, 0.0, fixed):
            self.commit_static_block = 0.0  # vetoed by a mover: keep waiting
            return action
        self.commit_static_block += dt
        if self.commit_static_block < DYNAMIC_COMMIT_STATIC_BLOCK_SECONDS:
            return action
        toolkit._reset_crossing_state()
        self.commit_static_block = 0.0
        self.commits_cancelled += 1
        return None

    def escape(self, obstacles, goal):
        """Short-term refuge when no DWA candidate is admissible near movers."""
        return self._toolkit._escape_from_dynamic(obstacles, goal)


# ---------------------------------------------------------------------------
# Planner
# ---------------------------------------------------------------------------
class _RecoveryAttempt:
    """Everything one recovery attempt needs; replaced, never partially reset."""

    __slots__ = (
        "sign", "number", "site_attempt", "backoff_distance", "commit_angle",
        "forward_distance", "phase_origin", "phase_time", "commit_theta",
        "forward_origin", "segment", "blocked_time", "backoff_travelled",
        "turned", "forward_travelled", "direction_scores",
    )

    def __init__(self, sign, number, site_attempt, scale, pose, direction_scores):
        self.sign = sign
        self.number = number
        self.site_attempt = site_attempt
        self.backoff_distance = RECOVERY_BACKOFF_DISTANCE * scale
        self.commit_angle = RECOVERY_COMMIT_ANGLE * scale
        self.forward_distance = RECOVERY_COMMIT_FORWARD_DISTANCE * scale
        self.phase_origin = tuple(pose)
        self.phase_time = 0.0
        self.commit_theta = pose[2]
        self.forward_origin = None
        self.segment = "turn"
        self.blocked_time = 0.0
        self.backoff_travelled = 0.0
        self.turned = 0.0
        self.forward_travelled = 0.0
        self.direction_scores = direction_scores


class ProgressDWA:
    """DWA-primary local planner with pose-progress based static recovery.

    Call :meth:`choose_action` once per control step and send the returned
    wheel speeds directly to RobotIO; no separate arbitration with a path
    follower is needed (the path provides the local goal).

    Dynamic-window policy (single place): ``current_v``/``current_w`` always
    hold the velocity of the command actually returned in the previous frame,
    whichever layer produced it (DWA, recovery, dynamic supervisor, or the
    STOP of the final safety veto).  The next dynamic window is centred on it.
    """

    def __init__(self, lidar_field_of_view=LIDAR_FIELD_OF_VIEW,
                 lidar_pose=(0.0, 0.0, 0.0)):
        self._toolkit = DynamicWindowAvoidance(lidar_field_of_view, lidar_pose)
        self.dynamic = DynamicSupervisor(self._toolkit)
        self.watchdog = ProgressWatchdog()
        self.field = CostToGoField()
        self.state = StaticAvoidanceState.NORMAL_DWA
        self.current_v = 0.0
        self.current_w = 0.0
        self.recovery = None
        self.recovery_count = 0
        self.last_recovery_sign = None
        self.control_label = "STOP"
        self.layer = "INIT"
        self.diagnostics = {}
        self._last_time = None
        self._waypoint_key = None
        self._goal_world = None
        self._site = None  # (x, y, waypoint_key, attempts, sign)

    # -- public helpers -----------------------------------------------------
    @property
    def static_state(self):
        return self.state

    @property
    def crossing_commit_active(self):
        return self.dynamic.commit_active

    @property
    def crossing_waiting(self):
        return self.dynamic.waiting

    @property
    def last_obstacles(self):
        return self._toolkit.last_obstacles

    @property
    def last_dynamic_obstacles(self):
        return self._toolkit.last_dynamic_obstacles

    def is_command_safe(self, left, right):
        """Same swept static/dynamic check as the legacy planner."""
        return self._toolkit.is_command_safe(left, right)

    # -- main entry -----------------------------------------------------------
    def choose_action(self, ranges, goal=(2.0, 0.0), pose=None, sim_time=None,
                      waypoint_id=None):
        """Return ``(left, right, description)`` for this control step."""
        if pose is None or sim_time is None:
            raise ValueError("ProgressDWA requires pose and sim_time")
        dt = self._advance_time(sim_time)
        self.diagnostics = {"state": None, "layer": None}
        if not ranges:
            self.dynamic.reset()
            self.watchdog.reset()
            self.layer = "SENSOR"
            return self._finish_frame(self._stop("LiDAR missing - STOP"), avoid=True)

        ranges, spikes = filter_isolated_returns(
            [float(value) if value is not None else math.inf for value in ranges],
            self._toolkit.lidar_field_of_view,
        )
        obstacles = self.dynamic.observe(ranges, pose, sim_time)
        fixed = [point for point in obstacles if len(point) == 2]
        moving = [point for point in obstacles if len(point) == 4]
        goal_world = _to_world(goal, pose)
        self._track_waypoint(waypoint_id, goal_world)
        self._goal_world = goal_world
        cosine, sine = math.cos(pose[2]), math.sin(pose[2])
        fixed_world = [
            (pose[0] + cosine * px - sine * py, pose[1] + sine * px + cosine * py)
            for px, py in fixed
        ]
        self.field.build(fixed_world, goal_world, pose[:2])
        goal_distance = math.hypot(goal[0], goal[1])
        start_cost = float(self.field.cost_at(pose[0], pose[1]))
        detour = start_cost - goal_distance
        watchdog_distance = goal_distance
        watchdog_tolerance = None
        if self.field.goal_projected:
            # The waypoint lies inside the static safety distance: the best
            # the robot can do is the admissible point nearest to it, so the
            # watchdog measures progress against that point and only accepts
            # actually reaching it.
            watchdog_distance = math.dist(pose[:2], self.field.goal)
            watchdog_tolerance = WATCHDOG_PROJECTED_GOAL_TOLERANCE
            detour = start_cost - watchdog_distance
        self.diagnostics.update(
            goal_distance=goal_distance, watchdog_distance=watchdog_distance,
            cost_to_go=start_cost, detour=detour,
            fixed=len(fixed), moving=len(moving), lidar_spikes=spikes,
            demoted_movers=self.dynamic.demoted_clusters,
            goal_projected=self.field.goal_projected,
        )

        # 1. Dynamic supervisor: may own this frame; static clocks pause.
        dynamic_action = self.dynamic.override(
            moving, obstacles, self._toolkit._rear_distance(ranges), pose,
            self.current_v, self.current_w, dt,
        )
        if dynamic_action is not None:
            self.layer = "DYNAMIC"
            self.watchdog.update(dt, pose, watchdog_distance, paused=True)
            if dynamic_action[:2] == (0.0, 0.0) and self._stopping_is_unsafe(obstacles):
                # A mover is predicted to reach a stationary robot: look for
                # a refuge instead of waiting in its path.
                _, info = self._evaluate_window(pose, fixed, moving, start_cost)
                refuge = self._dynamic_refuge(info, obstacles, goal)
                if refuge is not None:
                    dynamic_action = refuge
            return self._finish_frame(self._emit(dynamic_action, obstacles), avoid=True)

        # 2. Static state machine.
        if self.state is not StaticAvoidanceState.NORMAL_DWA:
            result = self._recovery_step(pose, obstacles, fixed, dt)
            if result is not None:
                self.layer = "RECOVERY"
                return self._finish_frame(result, avoid=True)
            dt = 0.0  # recovery ended this frame; grace already started
        return self._normal_dwa(goal, pose, obstacles, fixed, moving, watchdog_distance,
                                watchdog_tolerance, start_cost, detour, dt)

    # -- NORMAL_DWA -------------------------------------------------------------
    def _normal_dwa(self, goal, pose, obstacles, fixed, moving, goal_distance,
                    goal_tolerance, start_cost, detour, dt):
        best, info = self._evaluate_window(pose, fixed, moving, start_cost)
        self.diagnostics["dwa"] = info

        if best is None and moving:
            escape = self._dynamic_refuge(info, obstacles, goal)
            if escape is not None:
                self.layer = "DYNAMIC"
                self.watchdog.update(dt, pose, goal_distance, paused=True)
                return self._finish_frame(self._emit(escape, obstacles), avoid=True)

        stuck = self.watchdog.update(
            dt, pose, goal_distance, paused=info["dynamic_limited"], tolerance=goal_tolerance
        )
        self.diagnostics["watchdog"] = self._watchdog_diagnostics()
        if stuck:
            self.layer = "RECOVERY"
            return self._finish_frame(
                self._start_recovery(pose, obstacles, fixed, start_cost), avoid=True
            )

        self.layer = "DWA"
        if best is None:
            return self._finish_frame(
                self._stop("DWA no admissible trajectory - STOP"), avoid=True
            )
        constrained = (
            info["admissible"] < info["candidates"]
            or detour > NAV_FIELD_DETOUR_EPSILON
        )
        result = self._apply(
            best["v"], best["w"], obstacles,
            f"DWA v={best['v']:.2f} m/s, w={best['w']:.2f} rad/s, "
            f"clearance={best['clearance']:.2f} m",
        )
        return self._finish_frame(result, avoid=constrained)

    @staticmethod
    def _approaching_mover(moving):
        """True if a tracked mover in crossing range is closing on the robot."""
        for ox, oy, vx, vy in moving:
            distance = math.hypot(ox, oy)
            if distance < 1e-6 or distance > DWA_CROSSING_DETECTION_DISTANCE:
                continue
            if -(ox * vx + oy * vy) / distance >= DYNAMIC_CAUTION_MIN_CLOSING_SPEED:
                return True
        return False

    def _dynamic_window(self, speed_cap=STATIC_DWA_MAX_SPEED):
        control_dt = TIME_STEP / 1000.0
        v_low = max(0.0, self.current_v - DWA_LINEAR_ACCELERATION * control_dt)
        v_high = min(speed_cap,
                     self.current_v + DWA_LINEAR_ACCELERATION * control_dt)
        if v_high < v_low:
            v_high = v_low
        w_low = max(-DWA_MAX_ANGULAR_SPEED,
                    self.current_w - DWA_ANGULAR_ACCELERATION * control_dt)
        w_high = min(DWA_MAX_ANGULAR_SPEED,
                     self.current_w + DWA_ANGULAR_ACCELERATION * control_dt)
        velocities = _sample_range(v_low, v_high, DWA_VELOCITY_SAMPLES)
        yaw_rates = _sample_range(w_low, w_high, DWA_YAW_RATE_SAMPLES)
        if v_low <= 0.0 <= v_high and 0.0 not in velocities:
            velocities.append(0.0)
        if w_low <= 0.0 <= w_high and 0.0 not in yaw_rates:
            yaw_rates.append(0.0)
        return (v_low, v_high), (w_low, w_high), velocities, yaw_rates

    @staticmethod
    def _rollouts(requested):
        """Vectorised copy of DynamicWindowAvoidance._simulate for many (v, w)."""
        achieved = [DynamicWindowAvoidance._achievable_velocity(v, w) for v, w in requested]
        v = np.array([item[0] for item in achieved])[:, None]
        w = np.array([item[1] for item in achieved])[:, None]
        steps = np.arange(_ROLLOUT_STEPS)
        heading_before = w * DWA_SIMULATION_STEP * steps[None, :]
        x = np.cumsum(v * np.cos(heading_before) * DWA_SIMULATION_STEP, axis=1)
        y = np.cumsum(v * np.sin(heading_before) * DWA_SIMULATION_STEP, axis=1)
        heading_after = heading_before + w * DWA_SIMULATION_STEP
        return np.stack((x, y, heading_after), axis=2), achieved

    @staticmethod
    def _rollout_tail_clearance(trajectories, moving):
        """Clearance to movers' constant-velocity paths over the rollout tail.

        The legacy dynamic check predicts movers for 1.2 s only.  The rest of
        the 2.5 s rollout is compared too, but only for *scoring* (it is part
        of the clearance term): a constant-velocity rollout is not what the
        robot will do -- it can still brake -- so rejecting on it would leave
        no admissible candidate at cruise speed near any mover.
        """
        count = len(trajectories)
        if not moving:
            return np.full(count, np.inf)
        objects = np.asarray(moving, dtype=float)
        times = np.arange(_SAFETY_STEPS + 1, _ROLLOUT_STEPS + 1) * DWA_SIMULATION_STEP
        projected = objects[None, :, :2] + times[:, None, None] * objects[None, :, 2:4]
        paths = trajectories[:, _SAFETY_STEPS:_ROLLOUT_STEPS, None, :2]
        return np.hypot(paths[..., 0] - projected[None, :, :, 0],
                        paths[..., 1] - projected[None, :, :, 1]).min(axis=(1, 2))

    @staticmethod
    def _approaching_movers(moving):
        objects = np.asarray(moving, dtype=float).reshape(-1, 4)
        speed = np.hypot(objects[:, 2], objects[:, 3])
        distance = np.maximum(np.hypot(objects[:, 0], objects[:, 1]), 1e-6)
        closing = -(objects[:, 0] * objects[:, 2] + objects[:, 1] * objects[:, 3]) / distance
        return objects[(speed >= DYNAMIC_ANTICIPATION_MIN_SPEED)
                       & (closing >= DYNAMIC_CAUTION_MIN_CLOSING_SPEED)]

    @classmethod
    def _braking_gap(cls, velocity, yaw_rate, moving):
        """Inevitable-collision check: can the robot still stop out of the way?

        For each candidate the robot executes it for one step, then brakes at
        the DWA acceleration limits and stays stopped for
        DYNAMIC_ANTICIPATION_HOLD_SECONDS.  Returned is the smallest distance
        to every *approaching* mover's constant-velocity prediction over that
        time (inf without such movers).  If the maximum-braking candidate was
        clear in the previous frame, it is (up to prediction changes) clear
        again now, so this check does not dead-end at cruise speed the way a
        "stop at the end of the 2.5 s rollout" check does, and it rejects
        fleeing ahead of a mover or parking in its path.
        """
        velocity = np.asarray(velocity, dtype=float)
        yaw_rate = np.asarray(yaw_rate, dtype=float)
        approaching = cls._approaching_movers(moving) if len(moving) else np.zeros((0, 4))
        if not len(approaching):
            return np.full(len(velocity), np.inf)
        step = DWA_SIMULATION_STEP
        stop_time = max(float(np.max(np.abs(velocity))) / DWA_LINEAR_ACCELERATION,
                        float(np.max(np.abs(yaw_rate))) / DWA_ANGULAR_ACCELERATION)
        count = int(math.ceil((step + stop_time + DYNAMIC_ANTICIPATION_HOLD_SECONDS) / step))
        v, w = velocity.copy(), yaw_rate.copy()
        x = np.zeros_like(v)
        y = np.zeros_like(v)
        theta = np.zeros_like(v)
        gap = np.full(len(v), np.inf)
        for index in range(1, count + 1):
            x = x + v * np.cos(theta) * step
            y = y + v * np.sin(theta) * step
            theta = theta + w * step
            future = approaching[:, :2] + index * step * approaching[:, 2:4]
            distance = np.hypot(x[:, None] - future[None, :, 0], y[:, None] - future[None, :, 1])
            gap = np.minimum(gap, distance.min(axis=1))
            v = np.maximum(0.0, v - DWA_LINEAR_ACCELERATION * step)
            w = np.sign(w) * np.maximum(0.0, np.abs(w) - DWA_ANGULAR_ACCELERATION * step)
        return gap

    @staticmethod
    def _static_admissible(trajectories, fixed, buffer=STATIC_DWA_EXECUTION_BUFFER):
        """Vectorised DynamicWindowAvoidance._static_motion_is_safe (+ buffer).

        With ``buffer=0`` this is exactly the final veto's rule, for every
        candidate at once: over the 1.2 s safety prefix a point may never come
        within the robot radius; a point outside the static safety distance
        must stay outside it; a point already inside it (the robot drifted in
        by a few mm) must be left behind -- no step closer, the last step
        farther.  Using only "min clearance > safety distance" here would
        reject every candidate once the robot is inside the margin, including
        the ones the veto accepts, and freeze NORMAL_DWA.

        ``buffer`` (STATIC_DWA_EXECUTION_BUFFER) only makes it stricter, on the
        robot's overall clearance c (closest point over the prefix): from
        clearance c0 > safety distance + buffer a candidate must keep c above
        that; with c0 inside the buffer it may not lower the clearance
        (beyond STATIC_DWA_BUFFER_TOLERANCE), so it can drive along a wall but
        not towards it.  DWA thus never plans along the exact veto boundary.
        """
        count = len(trajectories)
        if not fixed:
            return np.ones(count, dtype=bool)
        points = np.asarray(fixed, dtype=float)[:, :2]
        initial = np.hypot(points[:, 0], points[:, 1])
        paths = trajectories[:, :_SAFETY_STEPS, None, :2]
        distance = np.hypot(paths[..., 0] - points[None, None, :, 0],
                            paths[..., 1] - points[None, None, :, 1])
        closest = distance.min(axis=1)  # candidates x points
        inside = initial <= STATIC_SAFE_DISTANCE
        outside_ok = closest > STATIC_SAFE_DISTANCE
        leaving_ok = (closest >= initial[None, :] - 1e-6) & (
            distance[:, -1, :] > initial[None, :] + 1e-3)
        ok = np.where(inside[None, :], leaving_ok, outside_ok) & (closest > DWA_ROBOT_RADIUS)
        ok = ok.all(axis=1)
        if buffer > 0.0:
            clearance = closest.min(axis=1)
            current = float(initial.min())
            ok &= (clearance > STATIC_SAFE_DISTANCE + buffer) | (
                clearance >= current - STATIC_DWA_BUFFER_TOLERANCE)
        return ok

    def _evaluate_window(self, pose, fixed, moving, start_cost):
        cautious = self._approaching_mover(moving)
        v_window, w_window, velocities, yaw_rates = self._dynamic_window(
            DYNAMIC_CAUTION_SPEED if cautious else STATIC_DWA_MAX_SPEED
        )
        requested = [(v, w) for v in velocities for w in yaw_rates]
        trajectories, achieved = self._rollouts(requested)
        static, dynamic = DynamicWindowAvoidance._sensor_clearances(
            trajectories, fixed, moving
        )
        static = np.asarray(static, dtype=float)
        dynamic = np.asarray(dynamic, dtype=float)
        requested_v = np.array([item[0] for item in requested])
        clearance = np.minimum(static, dynamic)
        static_ok = self._static_admissible(trajectories, fixed)
        dynamic_ok = dynamic > DYNAMIC_SAFE_DISTANCE
        braking_ok = requested_v ** 2 / (2.0 * DWA_LINEAR_ACCELERATION) <= clearance - DWA_ROBOT_RADIUS
        actual_v = np.array([item[0] for item in achieved])
        actual_w = np.array([item[1] for item in achieved])
        stoppable = self._braking_gap(actual_v, actual_w, moving)
        admissible = static_ok & dynamic_ok & braking_ok & (stoppable > DYNAMIC_SAFE_DISTANCE)
        without_dynamic = static_ok & (
            requested_v ** 2 / (2.0 * DWA_LINEAR_ACCELERATION) <= static - DWA_ROBOT_RADIUS
        )
        clearance = np.minimum(clearance, self._rollout_tail_clearance(trajectories, moving))
        near_mover = any(
            math.hypot(point[0], point[1]) <= DWA_CROSSING_DETECTION_DISTANCE
            for point in moving
        )
        info = {
            "v_window": v_window,
            "w_window": w_window,
            "candidates": len(requested),
            "dynamic_caution": cautious,
            "static_ok": int(static_ok.sum()),
            "admissible": int(admissible.sum()),
            "forward_admissible": 0,
            "dynamic_limited": bool(
                near_mover and int(without_dynamic.sum()) > int(admissible.sum())
            ),
            "best": None,
        }
        if not admissible.any():
            if len(moving):
                # Least-bad fallback for the dynamic layer: the statically safe
                # candidate that keeps the most room from movers, counting both
                # the legacy 1.2 s check and the braking check.
                room = np.where(without_dynamic, np.minimum(dynamic, stoppable), -np.inf)
                index = int(np.argmax(room))
                if np.isfinite(room[index]):
                    info["refuge"] = {"v": float(actual_v[index]), "w": float(actual_w[index]),
                                      "room": float(room[index])}
            return None, info

        info["forward_admissible"] = int((admissible & (actual_v > 0.005)).sum())
        x, y, theta = pose
        cosine, sine = math.cos(theta), math.sin(theta)
        world_x = x + cosine * trajectories[:, :, 0] - sine * trajectories[:, :, 1]
        world_y = y + sine * trajectories[:, :, 0] + cosine * trajectories[:, :, 1]
        path_cost = self.field.cost_at(world_x, world_y)
        # Only the first _SAFETY_STEPS of a rollout are safety checked.  A tail
        # that runs into inflated cells is cut there, so no rollout can earn
        # progress from the far side of an obstacle it would have to cross.
        tail_blocked = self.field.blocked_at(world_x, world_y)
        tail_blocked[:, :_SAFETY_STEPS] = False
        cut = np.maximum.accumulate(tail_blocked, axis=1)
        path_cost = np.where(cut, np.inf, path_cost)
        progress = (start_cost - path_cost.min(axis=1)) / _MAX_PROGRESS
        last = (~cut).sum(axis=1) - 1
        rows = np.arange(len(last))
        end_heading = theta + trajectories[rows, last, 2]
        descent = self.field.descent_heading_at(world_x[rows, last], world_y[rows, last])
        heading = np.cos(end_heading - descent)
        if not self.field.goal_projected:
            # Pass-through waypoint: reaching it is full progress, and where the
            # rollout ends afterwards (pointing "back" at it) is irrelevant.
            goal_x, goal_y = self.field.goal
            passes = (np.hypot(world_x - goal_x, world_y - goal_y)
                      <= STATIC_DWA_WAYPOINT_PASS_RADIUS) & ~cut
            reached = passes.any(axis=1)
            progress = np.where(reached, start_cost / _MAX_PROGRESS, progress)
            heading = np.where(reached, 1.0, heading)
            info["passes_waypoint"] = int((reached & admissible).sum())
        finite_clearance = np.where(np.isfinite(clearance), clearance, np.inf)
        clearance_score = np.clip(
            (finite_clearance - DWA_ROBOT_RADIUS) / STATIC_DWA_CLEARANCE_SCALE, 0.0, 1.0
        )
        speed = np.maximum(actual_v, 0.0) / STATIC_DWA_MAX_SPEED
        hysteresis = np.where(
            (abs(self.current_w) > 0.05) & (actual_w * self.current_w > 0.0), 1.0, 0.0
        )
        score = (
            STATIC_DWA_PROGRESS_WEIGHT * progress
            + STATIC_DWA_HEADING_WEIGHT * heading
            + STATIC_DWA_CLEARANCE_WEIGHT * clearance_score
            + STATIC_DWA_SPEED_WEIGHT * speed
            + STATIC_DWA_TURN_HYSTERESIS_WEIGHT * hysteresis
        )
        score = np.where(admissible, score, -np.inf)
        index = int(np.argmax(score))
        best = {
            "v": float(actual_v[index]),
            "w": float(actual_w[index]),
            "score": float(score[index]),
            "progress": float(progress[index]),
            "heading": float(heading[index]),
            "clearance": float(clearance[index]),
            "static_clearance": float(static[index]),
            "dynamic_clearance": float(dynamic[index]),
        }
        info["best"] = best
        return best, info

    # -- recovery -----------------------------------------------------------
    def _start_recovery(self, pose, obstacles, fixed, start_cost):
        """ENTER: watchdog STUCK in NORMAL_DWA.  Chooses the direction once."""
        site_attempt = 1
        site = self._site
        if (
            site is not None
            and site[2] == self._waypoint_key
            and math.hypot(pose[0] - site[0], pose[1] - site[1]) <= RECOVERY_SITE_RADIUS
        ):
            site_attempt = site[3] + 1
        scale = 1.0 + RECOVERY_ESCALATION * min(site_attempt - 1, RECOVERY_MAX_ESCALATION_STEPS)
        reverse_ok = self._is_safe(-RECOVERY_BACKOFF_SPEED, 0.0, obstacles)
        sign, scores = self._choose_recovery_direction(
            pose, obstacles, fixed, start_cost, scale, reverse_ok, site_attempt
        )
        self.recovery_count += 1
        self.last_recovery_sign = sign
        self.recovery = _RecoveryAttempt(
            sign, self.recovery_count, site_attempt, scale, pose, scores
        )
        self._site = (pose[0], pose[1], self._waypoint_key, site_attempt, sign)
        self.watchdog.reset()
        self.state = (
            StaticAvoidanceState.RECOVERY_BACKOFF if reverse_ok
            else StaticAvoidanceState.RECOVERY_COMMIT
        )
        return self._recovery_step(pose, obstacles, fixed, 0.0)

    def _primitive(self, sign, scale, reverse_ok):
        """Robot-frame (x, y, theta) samples of backoff + committed turn + forward."""
        samples = []
        x = y = theta = 0.0
        segments = []
        if reverse_ok:
            segments.append((-RECOVERY_BACKOFF_SPEED, 0.0,
                             RECOVERY_BACKOFF_DISTANCE * scale / RECOVERY_BACKOFF_SPEED, "backoff"))
        segments.append((0.0, sign * RECOVERY_COMMIT_YAW_RATE,
                         RECOVERY_COMMIT_ANGLE * scale / RECOVERY_COMMIT_YAW_RATE, "turn"))
        segments.append((RECOVERY_COMMIT_FORWARD_SPEED, 0.0,
                         RECOVERY_COMMIT_FORWARD_DISTANCE * scale / RECOVERY_COMMIT_FORWARD_SPEED,
                         "forward"))
        for velocity, yaw_rate, duration, name in segments:
            for _ in range(max(1, math.ceil(duration / DWA_SIMULATION_STEP))):
                x += velocity * math.cos(theta) * DWA_SIMULATION_STEP
                y += velocity * math.sin(theta) * DWA_SIMULATION_STEP
                theta += yaw_rate * DWA_SIMULATION_STEP
                samples.append((x, y, theta, name))
        return samples

    def _choose_recovery_direction(self, pose, obstacles, fixed, start_cost, scale,
                                   reverse_ok, site_attempt):
        """Roll out the left and right primitives once; pick the safer/better one.

        Order: feasibility (static and dynamic clearance of the committed turn
        and forward segments) > expected cost-to-go progress > minimum static
        clearance > tie-breakers (previous direction at this site, last DWA
        turn sign, wider side, left).
        """
        moving = [point for point in obstacles if len(point) == 4]
        x, y, theta = pose
        cosine, sine = math.cos(theta), math.sin(theta)
        scores = {}
        for sign in (1.0, -1.0):
            samples = self._primitive(sign, scale, reverse_ok)
            committed = [(sx, sy, st) for sx, sy, st, name in samples if name != "backoff"]
            static = DynamicWindowAvoidance._trajectory_clearance(committed, fixed)
            dynamic = DynamicWindowAvoidance._dynamic_reachable_clearance(
                [(sx, sy, st) for sx, sy, st, _ in samples], moving
            )
            end_x, end_y, _, _ = samples[-1]
            world_end = (x + cosine * end_x - sine * end_y, y + sine * end_x + cosine * end_y)
            progress = start_cost - float(self.field.cost_at(*world_end))
            scores[sign] = {
                "feasible": static > STATIC_SAFE_DISTANCE and dynamic > DYNAMIC_SAFE_DISTANCE,
                "static": static,
                "dynamic": dynamic,
                "progress": progress,
            }

        left, right = scores[1.0], scores[-1.0]
        if left["feasible"] != right["feasible"]:
            return (1.0 if left["feasible"] else -1.0), scores
        if left["feasible"]:
            if abs(left["progress"] - right["progress"]) > RECOVERY_DIRECTION_PROGRESS_TIE:
                return (1.0 if left["progress"] > right["progress"] else -1.0), scores
        if abs(left["static"] - right["static"]) > RECOVERY_DIRECTION_CLEARANCE_TIE:
            return (1.0 if left["static"] > right["static"] else -1.0), scores
        if site_attempt > 1 and self._site is not None:
            return self._site[4], scores
        if abs(self.current_w) > 0.05:
            return (1.0 if self.current_w > 0.0 else -1.0), scores
        left_open, right_open = self._side_openness(fixed)
        if abs(left_open - right_open) > RECOVERY_DIRECTION_CLEARANCE_TIE:
            return (1.0 if left_open > right_open else -1.0), scores
        return 1.0, scores

    @staticmethod
    def _side_openness(fixed):
        """Tie-breaker only: nearest static point in the left/right half planes."""
        left = min((math.hypot(px, py) for px, py in fixed if py > 0.0), default=math.inf)
        right = min((math.hypot(px, py) for px, py in fixed if py < 0.0), default=math.inf)
        return min(left, 4.0), min(right, 4.0)

    def _recovery_step(self, pose, obstacles, fixed, dt):
        """ACTION/EXIT of RECOVERY_BACKOFF and RECOVERY_COMMIT.

        Returns a command, or None after switching back to NORMAL_DWA.
        """
        attempt = self.recovery
        attempt.phase_time += dt

        if self.state is StaticAvoidanceState.RECOVERY_BACKOFF:
            attempt.backoff_travelled = math.hypot(
                pose[0] - attempt.phase_origin[0], pose[1] - attempt.phase_origin[1]
            )
            reverse_ok = self._is_safe(-RECOVERY_BACKOFF_SPEED, 0.0, obstacles)
            if (
                attempt.backoff_travelled < attempt.backoff_distance
                and attempt.phase_time < RECOVERY_BACKOFF_TIMEOUT
                and reverse_ok
            ):
                self._record_recovery_diagnostics()
                return self._apply(*self._ramp(-RECOVERY_BACKOFF_SPEED, 0.0), obstacles,
                                   "RECOVERY_BACKOFF 안전 후진 복구")
            # EXIT: travelled far enough, timed out, or reverse became unsafe.
            self.state = StaticAvoidanceState.RECOVERY_COMMIT
            attempt.phase_origin = tuple(pose)
            attempt.phase_time = 0.0
            attempt.commit_theta = pose[2]

        attempt.turned = abs(_wrap(pose[2] - attempt.commit_theta))
        if attempt.phase_time >= RECOVERY_COMMIT_TIMEOUT:
            return self._end_recovery("commit timeout")
        if attempt.segment == "turn":
            if attempt.turned >= attempt.commit_angle:
                attempt.segment = "forward"
                attempt.forward_origin = tuple(pose)
            else:
                yaw_rate = attempt.sign * RECOVERY_COMMIT_YAW_RATE
                self._record_recovery_diagnostics()
                if self._is_safe(0.0, yaw_rate, obstacles):
                    attempt.blocked_time = 0.0
                    return self._apply(*self._ramp(0.0, yaw_rate), obstacles,
                                       "RECOVERY_COMMIT 복구 제자리 회전")
                attempt.blocked_time += dt
                if attempt.blocked_time >= RECOVERY_BLOCKED_TIMEOUT:
                    return self._end_recovery("committed turn blocked")
                return self._stop("RECOVERY_COMMIT 회전 차단 - STOP")

        attempt.forward_travelled = math.hypot(
            pose[0] - attempt.forward_origin[0], pose[1] - attempt.forward_origin[1]
        )
        if attempt.forward_travelled >= attempt.forward_distance:
            return self._end_recovery("commit complete")
        if not self._is_safe(RECOVERY_COMMIT_FORWARD_SPEED, 0.0, obstacles):
            return self._end_recovery("committed forward blocked")
        self._record_recovery_diagnostics()
        return self._apply(*self._ramp(RECOVERY_COMMIT_FORWARD_SPEED, 0.0), obstacles,
                           "RECOVERY_COMMIT 복구 전진")

    def _ramp(self, target_v, target_w):
        """Approach a recovery target within the DWA acceleration limits.

        Recovery primitives used to step straight to their target speed;
        in Webots such steps make the chassis skid and corrupt encoder
        odometry, which the watchdog and every distance-based exit rely on.
        """
        control_dt = TIME_STEP / 1000.0
        dv = DWA_LINEAR_ACCELERATION * control_dt
        dw = DWA_ANGULAR_ACCELERATION * control_dt
        velocity = min(max(target_v, self.current_v - dv), self.current_v + dv)
        yaw_rate = min(max(target_w, self.current_w - dw), self.current_w + dw)
        return velocity, yaw_rate

    def _end_recovery(self, reason):
        """EXIT to NORMAL_DWA: reset watchdog history and start the grace period.

        current_v/current_w keep the last applied recovery command, so the
        next dynamic window starts from the robot's real motion.
        """
        self._record_recovery_diagnostics(reason)
        self.state = StaticAvoidanceState.NORMAL_DWA
        self.recovery = None
        self.watchdog.start_grace(WATCHDOG_GRACE_SECONDS)
        return None

    def _record_recovery_diagnostics(self, end_reason=None):
        attempt = self.recovery
        if attempt is None:
            return
        self.diagnostics["recovery"] = {
            "attempt": attempt.number,
            "site_attempt": attempt.site_attempt,
            "sign": attempt.sign,
            "segment": attempt.segment,
            "backoff_travelled": attempt.backoff_travelled,
            "turned": attempt.turned,
            "forward_travelled": attempt.forward_travelled,
            "direction_scores": attempt.direction_scores,
            "end_reason": end_reason,
        }

    # -- waypoint / time bookkeeping ------------------------------------------
    def _advance_time(self, sim_time):
        previous = self._last_time
        self._last_time = sim_time
        if previous is None:
            return 0.0
        elapsed = sim_time - previous
        return elapsed if 0.0 < elapsed <= _MAX_CONTROL_DT else 0.0

    def _track_waypoint(self, waypoint_id, goal_world):
        if waypoint_id is not None:
            changed = waypoint_id != self._waypoint_key
            key = waypoint_id
        else:
            changed = (
                self._goal_world is None
                or math.dist(goal_world, self._goal_world) > WATCHDOG_WAYPOINT_JUMP
            )
            key = (round(goal_world[0], 2), round(goal_world[1], 2)) if changed else self._waypoint_key
        if changed:
            self._waypoint_key = key
            self.watchdog.reset()
            self._site = None

    def _watchdog_diagnostics(self):
        return {
            "no_progress_s": self.watchdog.no_progress_seconds,
            "long_no_progress_s": self.watchdog.long_no_progress_seconds,
            "grace_s": self.watchdog.grace_remaining,
            "stuck_reason": self.watchdog.stuck_reason,
        }

    # -- command output (single place for current_v/current_w) ---------------
    def _is_safe(self, velocity, yaw_rate, obstacles):
        actual_v, actual_w, _, _ = DynamicWindowAvoidance._achievable_velocity(velocity, yaw_rate)
        return self._toolkit._velocity_is_safe(actual_v, actual_w, obstacles)

    def _stop(self, description):
        self.current_v = 0.0
        self.current_w = 0.0
        return 0.0, 0.0, description

    def _apply(self, velocity, yaw_rate, obstacles, description):
        """Final safety veto for a (v, w) request; STOP if it fails."""
        actual_v, actual_w, left, right = DynamicWindowAvoidance._achievable_velocity(
            velocity, yaw_rate
        )
        if not self._toolkit._velocity_is_safe(actual_v, actual_w, obstacles):
            return self._stop(description + " -> safety veto STOP")
        self.current_v = actual_v
        self.current_w = actual_w
        return left, right, description

    def _emit(self, action, obstacles):
        """Final safety veto for a wheel command from another layer."""
        left, right, description = action
        if not (
            math.isfinite(left) and math.isfinite(right)
            and abs(left) <= MAX_SPEED and abs(right) <= MAX_SPEED
        ):
            return self._stop(description + " -> invalid command STOP")
        if left == 0.0 and right == 0.0:
            return self._stop(description)
        velocity, yaw_rate = _velocity_from_wheels(left, right)
        if not self._toolkit._velocity_is_safe(velocity, yaw_rate, obstacles):
            if not self._safer_than_stopping(velocity, yaw_rate, obstacles):
                return self._stop(description + " -> safety veto STOP")
            description += " (safer than stopping)"
        self.current_v = velocity
        self.current_w = yaw_rate
        return left, right, description

    def _dynamic_refuge(self, info, obstacles, goal):
        """Wheel command for "no admissible candidate while movers are near".

        First the window's least-bad candidate (see _evaluate_window), if it
        keeps more room from the movers than stopping here would; otherwise
        the legacy short-term refuge search (it can also reverse).  Either
        goes through _emit, i.e. the shared veto / safer-than-stopping rule.
        """
        refuge = info.get("refuge")
        moving = [point for point in obstacles if len(point) == 4]
        if refuge is not None and moving:
            stay = min(
                self._toolkit._dynamic_reachable_clearance(self._toolkit._simulate(0.0, 0.0), moving),
                float(self._braking_gap([0.0], [0.0], moving)[0]),
            )
            if refuge["room"] > stay + 0.02:
                _, _, left, right = DynamicWindowAvoidance._achievable_velocity(
                    refuge["v"], refuge["w"])
                return left, right, "DWA dynamic refuge (braking check)"
        return self.dynamic.escape(obstacles, goal)

    def _stopping_is_unsafe(self, obstacles):
        moving = [point for point in obstacles if len(point) == 4]
        if not moving:
            return False
        toolkit = self._toolkit
        return toolkit._dynamic_reachable_clearance(
            toolkit._simulate(0.0, 0.0), moving
        ) <= DYNAMIC_SAFE_DISTANCE

    def _safer_than_stopping(self, velocity, yaw_rate, obstacles):
        """True when STOP is itself predicted to be less safe than the command.

        The veto normally replaces an unsafe command by STOP.  With a mover
        approaching the robot's side, however, a stationary robot is exactly
        what gets hit.  A dynamic-layer command is then kept only if it
        passes the full static check, keeps the legacy emergency clearance
        from every mover, and has strictly more predicted clearance than
        STOP.  No margin is lowered for normal operation: this only chooses
        between two options that both already violate the dynamic margin.
        """
        fixed = [point for point in obstacles if len(point) == 2]
        moving = [point for point in obstacles if len(point) == 4]
        if not moving:
            return False
        toolkit = self._toolkit
        trajectory = toolkit._simulate(velocity, yaw_rate)
        if not toolkit._static_motion_is_safe(trajectory[:_SAFETY_STEPS], fixed):
            return False
        stop_clearance = toolkit._dynamic_reachable_clearance(toolkit._simulate(0.0, 0.0), moving)
        if stop_clearance > DYNAMIC_SAFE_DISTANCE:
            return False
        clearance = toolkit._dynamic_reachable_clearance(trajectory, moving)
        return clearance > EMERGENCY_SAFE_DISTANCE and clearance > stop_clearance + 0.02

    def _finish_frame(self, result, avoid):
        """Record the integration label; diagnostics only, no control effect."""
        left, right, description = result
        if self.layer == "DYNAMIC":
            label = "AVOID DYNAMIC"
        elif self.layer == "RECOVERY":
            label = f"AVOID {self.state.value}"
        elif avoid:
            label = "AVOID DWA"
        else:
            label = "PATH_FOLLOW"
        self.control_label = label
        self.diagnostics["state"] = self.state.value
        self.diagnostics["layer"] = self.layer
        self.diagnostics["current_vw"] = (self.current_v, self.current_w)
        return left, right, description


__all__ = [
    "CostToGoField",
    "filter_isolated_returns",
    "DynamicSupervisor",
    "ProgressDWA",
    "ProgressWatchdog",
    "StaticAvoidanceState",
]
