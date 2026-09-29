"""Progress-watchdog DWA running beside the legacy avoidance controller.

The legacy :mod:`avoidance` module is intentionally left unchanged.  This
planner reuses its scan, trajectory and collision primitives, but owns a much
smaller static state machine: NORMAL_DWA -> RECOVERY_BACKOFF ->
RECOVERY_COMMIT -> NORMAL_DWA.
"""

from collections import deque
import math

from avoidance import DynamicWindowAvoidance
from config import (
    DWA_DYNAMIC_CLEARANCE_MARGIN,
    DWA_MAX_LINEAR_SPEED,
    DWA_ROBOT_RADIUS,
    DWA_STATIC_CLEARANCE_MARGIN,
    DWA_WALL_FOLLOW_TURN_RATE,
    MAX_SPEED,
    TIME_STEP,
    WHEEL_RADIUS,
    WHEEL_TRACK,
)


NORMAL_DWA = "NORMAL_DWA"
RECOVERY_BACKOFF = "RECOVERY_BACKOFF"
RECOVERY_COMMIT = "RECOVERY_COMMIT"

WATCHDOG_SECONDS = 2.5
WATCHDOG_MIN_GOAL_IMPROVEMENT = 0.03
WATCHDOG_GOAL_TOLERANCE = 0.14
RECOVERY_BACKOFF_DISTANCE = 0.12
RECOVERY_BACKOFF_TIMEOUT = 1.2
RECOVERY_BACKOFF_SPEED = 0.10
RECOVERY_COMMIT_YAW_RATE = 0.90
RECOVERY_COMMIT_MIN_ANGLE = math.radians(50.0)
RECOVERY_COMMIT_TIMEOUT = 2.5
RECOVERY_GRACE_SECONDS = 1.0


def _angle_delta(current, origin):
    return abs(math.atan2(math.sin(current - origin), math.cos(current - origin)))


class ProgressWatchdogDWA(DynamicWindowAvoidance):
    """DWA primary planner with one odometry-driven static recovery."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.static_state = NORMAL_DWA
        self.dynamic_yield_active = False
        self.recovery_turn_sign = None
        self.recovery_start_pose = None
        self.recovery_phase_pose = None
        self.recovery_phase_time = None
        self.recovery_count = 0
        self._watchdog = deque()
        self._watchdog_grace_until = 0.0
        self.debug_goal_improvement = 0.0
        self.debug_watchdog_span = 0.0

    def _sensor_score_terms(
        self, v, w, trajectory, clearance, goal, prefer_forward=False,
    ):
        """Reward actual goal progress so safe pure rotation cannot dominate."""
        target_x, target_y = goal
        start_distance = math.hypot(target_x, target_y)
        end_x, end_y, end_theta = trajectory[-1]
        end_distance = math.hypot(target_x - end_x, target_y - end_y)
        progress = start_distance - end_distance
        target_heading = math.atan2(target_y - end_y, target_x - end_x)
        heading_error = math.atan2(
            math.sin(target_heading - end_theta),
            math.cos(target_heading - end_theta),
        )
        goal_contribution = 8.0 * progress
        heading_contribution = 0.6 * math.cos(heading_error)
        clearance_contribution = 0.25 * min(1.0, clearance)
        speed_contribution = 0.5 * max(0.0, v) / DWA_MAX_LINEAR_SPEED
        reverse_penalty = -0.6 * max(0.0, -v) / DWA_MAX_LINEAR_SPEED
        stop_penalty = -0.15 if abs(v) < 0.005 and start_distance > 0.20 else 0.0
        return {
            "goal_distance": goal_contribution,
            "heading": heading_contribution,
            "clearance": clearance_contribution,
            "speed": speed_contribution,
            "stop_penalty": stop_penalty + reverse_penalty,
            "total": (
                goal_contribution
                + heading_contribution
                + clearance_contribution
                + speed_contribution
                + reverse_penalty
                + stop_penalty
            ),
        }

    def _update_watchdog(self, pose, goal, sim_time):
        if sim_time < self._watchdog_grace_until:
            self._watchdog.clear()
            return False
        distance = math.hypot(goal[0], goal[1])
        if distance <= WATCHDOG_GOAL_TOLERANCE:
            self._watchdog.clear()
            return False
        self._watchdog.append((sim_time, pose[0], pose[1], distance))
        cutoff = sim_time - WATCHDOG_SECONDS
        while len(self._watchdog) > 1 and self._watchdog[0][0] < cutoff:
            self._watchdog.popleft()
        first = self._watchdog[0]
        self.debug_watchdog_span = sim_time - first[0]
        self.debug_goal_improvement = first[3] - min(item[3] for item in self._watchdog)
        return (
            self.debug_watchdog_span >= WATCHDOG_SECONDS - TIME_STEP / 1000.0
            and self.debug_goal_improvement < WATCHDOG_MIN_GOAL_IMPROVEMENT
        )

    def _primitive_clearance(self, velocity, yaw_rate, fixed):
        trajectory = self._simulate(velocity, yaw_rate)
        return self._trajectory_clearance(trajectory, fixed)

    def _choose_recovery_side(self, fixed):
        candidates = []
        for sign in (1.0, -1.0):
            clearance = self._primitive_clearance(
                0.06, sign * DWA_WALL_FOLLOW_TURN_RATE, fixed
            )
            candidates.append((clearance, sign))
        candidates.sort(key=lambda item: (item[0], item[1] == self.last_turn_sign), reverse=True)
        return candidates[0][1]

    def _start_recovery(self, pose, sim_time, fixed, obstacles):
        self.recovery_count += 1
        self.recovery_start_pose = tuple(pose)
        self.recovery_phase_pose = tuple(pose)
        self.recovery_phase_time = sim_time
        self.recovery_turn_sign = self._choose_recovery_side(fixed)
        self.last_turn_sign = self.recovery_turn_sign
        self._watchdog.clear()
        if self._velocity_is_safe(-RECOVERY_BACKOFF_SPEED, 0.0, obstacles):
            self.static_state = RECOVERY_BACKOFF
            return self._command(-RECOVERY_BACKOFF_SPEED, 0.0, obstacles, "STUCK safe backoff")
        self.static_state = RECOVERY_COMMIT
        return self._command(
            0.0,
            self.recovery_turn_sign * RECOVERY_COMMIT_YAW_RATE,
            obstacles,
            "STUCK committed turn",
        )

    def _finish_recovery(self, sim_time):
        self.static_state = NORMAL_DWA
        self.recovery_start_pose = None
        self.recovery_phase_pose = None
        self.recovery_phase_time = None
        self.recovery_turn_sign = None
        self.current_v = 0.0
        self.current_w = 0.0
        self._watchdog.clear()
        self._watchdog_grace_until = sim_time + RECOVERY_GRACE_SECONDS

    def _command(self, velocity, yaw_rate, obstacles, description):
        left, right = self._wheel_speeds(velocity, yaw_rate)
        actual_v = 0.5 * (left + right) * WHEEL_RADIUS
        actual_w = (right - left) * WHEEL_RADIUS / WHEEL_TRACK
        if not self._velocity_is_safe(actual_v, actual_w, obstacles):
            self.current_v = 0.0
            self.current_w = 0.0
            return 0.0, 0.0, f"{description} blocked - STOP"
        self.current_v = actual_v
        self.current_w = actual_w
        return left, right, description

    def _recovery_action(self, pose, sim_time, obstacles):
        elapsed = sim_time - self.recovery_phase_time
        if self.static_state == RECOVERY_BACKOFF:
            travelled = math.dist(pose[:2], self.recovery_phase_pose[:2])
            if (
                travelled < RECOVERY_BACKOFF_DISTANCE
                and elapsed < RECOVERY_BACKOFF_TIMEOUT
                and self._velocity_is_safe(-RECOVERY_BACKOFF_SPEED, 0.0, obstacles)
            ):
                return self._command(
                    -RECOVERY_BACKOFF_SPEED, 0.0, obstacles, "STUCK safe backoff"
                )
            self.static_state = RECOVERY_COMMIT
            self.recovery_phase_pose = tuple(pose)
            self.recovery_phase_time = sim_time

        turned = _angle_delta(pose[2], self.recovery_phase_pose[2])
        if turned >= RECOVERY_COMMIT_MIN_ANGLE or elapsed >= RECOVERY_COMMIT_TIMEOUT:
            self._finish_recovery(sim_time)
            return None
        return self._command(
            0.0,
            self.recovery_turn_sign * RECOVERY_COMMIT_YAW_RATE,
            obstacles,
            "STUCK committed turn",
        )

    def _dynamic_requires_yield(self, moving):
        safety = DWA_ROBOT_RADIUS + DWA_DYNAMIC_CLEARANCE_MARGIN
        for ox, oy, vx, vy in moving:
            relative_speed_sq = vx * vx + vy * vy
            if relative_speed_sq < 1e-6:
                continue
            closest_time = max(0.0, min(1.5, -(ox * vx + oy * vy) / relative_speed_sq))
            separation = math.hypot(ox + vx * closest_time, oy + vy * closest_time)
            if ox > -0.10 and separation <= safety:
                return True
        return False

    def choose_action(self, ranges, goal=(2.0, 0.0), pose=None, sim_time=None):
        if pose is None or sim_time is None:
            raise ValueError("simple planner requires pose and sim_time")
        if not ranges:
            self.current_v = self.current_w = 0.0
            self.last_obstacles = None
            self._watchdog.clear()
            return 0.0, 0.0, "LiDAR missing - STOP"

        obstacles = self._scan_with_dynamic_obstacles(ranges, pose, sim_time)
        self.last_obstacles = obstacles
        fixed = [obstacle for obstacle in obstacles if len(obstacle) == 2]
        moving = [obstacle for obstacle in obstacles if len(obstacle) == 4]

        self.dynamic_yield_active = self._dynamic_requires_yield(moving)
        if self.dynamic_yield_active:
            self.current_v = self.current_w = 0.0
            self._watchdog.clear()
            return 0.0, 0.0, "DYNAMIC YIELD"

        if self.static_state != NORMAL_DWA:
            action = self._recovery_action(pose, sim_time, obstacles)
            if action is not None:
                return action

        self.side_speed_limit = DWA_MAX_LINEAR_SPEED
        best = self._rollout_best_candidate(
            goal, obstacles, fixed, moving, sensor_mode=True
        )
        if self._update_watchdog(pose, goal, sim_time):
            return self._start_recovery(pose, sim_time, fixed, obstacles)
        if best is None:
            self.current_v = self.current_w = 0.0
            return 0.0, 0.0, "DWA no safe trajectory - STOP"

        _, self.current_v, self.current_w, clearance, left, right = best
        if abs(self.current_w) > 0.05:
            self.last_turn_sign = 1.0 if self.current_w > 0.0 else -1.0
        return self._finalize_action(
            (left, right, f"DWA primary clearance={clearance:.3f}"), obstacles
        )


__all__ = [
    "NORMAL_DWA",
    "RECOVERY_BACKOFF",
    "RECOVERY_COMMIT",
    "ProgressWatchdogDWA",
]
