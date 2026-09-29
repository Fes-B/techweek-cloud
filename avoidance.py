"""LiDAR 기반 로컬 Dynamic Window Approach 장애물 회피."""

import math
try:
    import numpy as np
except ImportError:
    np = None

from config import (
    DWA_BACKOFF_CLEAR_DISTANCE,
    DWA_BACKOFF_MAX_SECONDS,
    DWA_BACKOFF_REAR_DISTANCE,
    DWA_BACKOFF_SPEED,
    DWA_BACKOFF_TRIGGER_DISTANCE,
    DWA_DYNAMIC_MATCH_DISTANCE,
    DWA_DYNAMIC_MAX_SPEED,
    DWA_DYNAMIC_CLUSTER_GAP,
    DWA_DYNAMIC_CLEARANCE_MARGIN,
    DWA_EMERGENCY_CLEARANCE_MARGIN,
    DWA_DYNAMIC_ACCELERATION_BOUND,
    DWA_DYNAMIC_MIN_CLUSTER_POINTS,
    DWA_DYNAMIC_MIN_SPEED,
    DWA_DYNAMIC_TRACK_HOLD_SECONDS,
    DWA_EGO_ROTATION_SUPPRESS_RATE,
    DWA_ANGULAR_ACCELERATION,
    DWA_CLEARANCE_WEIGHT,
    DWA_HEADING_WEIGHT,
    DWA_HORIZON,
    DWA_LINEAR_ACCELERATION,
    DWA_MAX_ANGULAR_SPEED,
    DWA_MAX_LINEAR_SPEED,
    DWA_MAX_REVERSE_SPEED,
    DWA_MIN_RANGE,
    DWA_PROGRESS_WEIGHT,
    DWA_RECOVERY_CLEAR_DISTANCE,
    DWA_PATH_RELEASE_CONFIRM_STEPS,
    DWA_RECOVERY_ESCAPE_CONFIRM_STEPS,
    DWA_RECOVERY_TRIGGER_DISTANCE,
    DWA_RECOVERY_YAW_RATE,
    DWA_ROBOT_RADIUS,
    DWA_STATIC_CLEARANCE_MARGIN,
    DWA_SIMULATION_STEP,
    DWA_SIDE_CAUTION_DISTANCE,
    DWA_SIDE_CAUTION_SPEED,
    DWA_SIDE_CLEAR_STEPS,
    DWA_SIDE_STOP_DISTANCE,
    DWA_SIDE_YIELD_SECONDS,
    DWA_SPEED_WEIGHT,
    DWA_TURN_SWITCH_MARGIN,
    DWA_VELOCITY_SAMPLES,
    DWA_YAW_RATE_SAMPLES,
    DWA_WALL_CLEAR_STEPS,
    DWA_WALL_EMERGENCY_DISTANCE,
    DWA_WALL_FOLLOW_DISTANCE,
    DWA_WALL_FOLLOW_GAIN,
    DWA_WALL_FOLLOW_SPEED,
    DWA_WALL_FOLLOW_TURN_RATE,
    DWA_WALL_LOST_DISTANCE,
    DWA_WALL_RELEASE_DISTANCE,
    DWA_SIDE_APPROACH_DISTANCE,
    DWA_CROSSING_YIELD_DISTANCE,
    DWA_CROSSING_BACKOFF_DISTANCE,
    DWA_CROSSING_CLEARING_DISTANCE,
    DWA_CROSSING_CLEARING_LATERAL_DISTANCE,
    DWA_CROSSING_CLEARING_X_RANGE,
    DWA_CROSSING_DETECTION_DISTANCE,
    DWA_CROSSING_COMMIT_CANCEL_SECONDS,
    DWA_CROSSING_COMMIT_MAX_DISTANCE,
    DWA_CROSSING_COMMIT_MIN_DISTANCE,
    DWA_CROSSING_COMMIT_SPEED,
    DWA_CROSSING_GAP_CONFIRM_STEPS,
    DWA_CROSSING_GAP_LATERAL_DISTANCE,
    DWA_CROSSING_GAP_MIN_SECONDS,
    DWA_CROSSING_IMMINENT_X_MARGIN,
    DWA_CROSSING_LANE_HALF_WIDTH,
    DWA_CROSSING_LATERAL_SPEED,
    DWA_CROSSING_PREDICTION_SECONDS,
    DWA_CROSSING_STATIONARY_SPEED,
    DWA_CROSSING_YIELD_X_RANGE,
    LIDAR_FIELD_OF_VIEW,
    MAX_SPEED,
    TIME_STEP,
    WHEEL_RADIUS,
    WHEEL_TRACK,
)


def _sample_range(low, high, count):
    if count <= 1 or high <= low:
        return [low]
    step = (high - low) / (count - 1)
    return [low + index * step for index in range(count)]


# Include obstacles that can move into the robot's path during the prediction horizon.
_SCAN_RELEVANT_DISTANCE = (
    (DWA_MAX_LINEAR_SPEED + DWA_DYNAMIC_MAX_SPEED) * DWA_HORIZON
    + DWA_ROBOT_RADIUS
    + 0.8
)


class DynamicWindowAvoidance:
    """로봇 좌표계의 목표점을 향해 후보 속도를 평가하는 로컬 DWA 플래너."""

    def __init__(self, lidar_field_of_view=LIDAR_FIELD_OF_VIEW,
                 lidar_pose=(0.0, 0.0, 0.0)):
        if not 0.0 < lidar_field_of_view <= 2.0 * math.pi:
            raise ValueError("LiDAR field of view must be in (0, 2*pi]")
        self.lidar_field_of_view = lidar_field_of_view
        self.lidar_pose = tuple(lidar_pose)
        self.current_v = 0.0
        self.current_w = 0.0
        self.last_turn_sign = 1.0
        self.recovery_turn_sign = None
        self.recovery_phase = None
        self.wall_side = None
        self.wall_clear_steps = 0
        self.wall_escape_active = False
        self.wall_front_turn_sign = None
        self.backoff_steps = 0
        self.side_yield_elapsed = 0.0
        self.side_clear_steps = 0
        self.side_speed_limit = DWA_MAX_LINEAR_SPEED
        self.previous_scan_clusters = None
        self.previous_cluster_histories = []
        self.previous_scan_time = None
        self.previous_scan_heading = None
        self._ego_rotation_cooldown = 0.0
        self._last_rollout_had_no_safe_candidate = False
        self.debug_safe_candidate_count = 0
        self.debug_best_vw = None
        self.debug_total_candidates = 0
        self.debug_static_ok_count = 0
        self.debug_dynamic_ok_count = 0
        self.debug_braking_ok_count = 0
        self.debug_best_static_clearance = None
        self.debug_best_dynamic_clearance = None
        self.debug_best_score = None
        self.debug_v_window = None
        self.debug_w_window = None
        self.debug_valid_forward_count = 0
        self.debug_best_candidate = None
        self.debug_best_forward_candidate = None
        self._best_positive_rollout = None
        self._front_corner_escape_active = False
        self._front_corner_escape_origin = None
        self._wall_escape_cooldown = 0.0
        self._recovery_escape_streak = 0
        self.last_dynamic_obstacles = []
        self.last_obstacles = None
        self._rollout_cache = None
        self.crossing_waiting = False
        self.crossing_gap_steps = 0
        self.crossing_commit_active = False
        self.crossing_commit_remaining = 0.0
        self.crossing_commit_last_pose = None
        self.crossing_departure_observed = False
        self._crossing_exit_distance = DWA_CROSSING_COMMIT_MIN_DISTANCE
        # Start "already released" so a path that was never interrupted
        # incurs no delay; only an actual unsafe frame resets this to zero,
        # after which DWA_PATH_RELEASE_CONFIRM_STEPS consecutive safe frames
        # are required again. See is_release_ready().
        self._path_release_streak = DWA_PATH_RELEASE_CONFIRM_STEPS

    @staticmethod
    def _sector_values(ranges, center, half_width):
        count = len(ranges)
        values = []
        for offset in range(-half_width, half_width + 1):
            value = ranges[(center + offset) % count]
            if math.isfinite(value):
                values.append(value)
        return values

    @classmethod
    def _front_distance(cls, ranges):
        values = cls._sector_values(ranges, len(ranges) // 2, max(2, len(ranges) // 36))
        return min(values) if values else float("inf")

    @classmethod
    def _front_arc_distance(cls, ranges):
        values = cls._sector_values(ranges, len(ranges) // 2, max(2, len(ranges) // 7))
        return min(values) if values else float("inf")

    @classmethod
    def _rear_distance(cls, ranges):
        values = cls._sector_values(ranges, 0, max(2, len(ranges) // 36))
        return min(values) if values else float("inf")

    @staticmethod
    def _front_point_distance(obstacles, half_angle):
        """Return the closest obstacle point inside a robot-frame front sector."""
        distances = [
            math.hypot(point[0], point[1])
            for point in obstacles
            if abs(math.atan2(point[1], point[0])) <= half_angle
        ]
        return min(distances, default=float("inf"))

    def _select_turn_sign(self, ranges):
        left_open, right_open = self._side_distances(ranges)
        if left_open == right_open or abs(left_open - right_open) < 0.05:
            return self.last_turn_sign
        return 1.0 if left_open > right_open else -1.0

    def _update_turn_sign(self, ranges, turn_sign):
        if turn_sign is None:
            return self._select_turn_sign(ranges)

        left_open, right_open = self._side_distances(ranges)
        current_open = left_open if turn_sign > 0.0 else right_open
        alternate_open = right_open if turn_sign > 0.0 else left_open
        if alternate_open > current_open + DWA_TURN_SWITCH_MARGIN:
            return -turn_sign
        return turn_sign

    def _recovery_turn(self, ranges):
        if self._front_corner_escape_active:
            # Sector membership changes during an in-place turn even though
            # the obstacle has not moved.  Keep the escape direction fixed
            # until translation clears the latched corner episode.
            if self.recovery_turn_sign is None:
                self.recovery_turn_sign = self._select_turn_sign(ranges)
        else:
            self.recovery_turn_sign = self._update_turn_sign(
                ranges, self.recovery_turn_sign
            )
        self.last_turn_sign = self.recovery_turn_sign
        self.current_v = min(DWA_WALL_FOLLOW_SPEED, self.side_speed_limit)
        self.current_w = self.recovery_turn_sign * DWA_WALL_FOLLOW_TURN_RATE
        left, right = self._wheel_speeds(self.current_v, self.current_w)
        direction = "좌회전" if self.recovery_turn_sign > 0.0 else "우회전"
        return left, right, f"DWA 회피 복구 - 저속 전진 {direction}"

    def _start_recovery(self, ranges, phase="turn"):
        # Deliberately do NOT reset recovery_turn_sign here. A recovery
        # episode that gets released (is_release_ready) and then
        # re-triggered a moment later (e.g. best is None -> emergency_turn,
        # repeatedly) must not re-pick a direction from a bare left/right
        # clearance snapshot each time -- that snapshot flips easily between
        # two nearby static obstacles and produces a fresh LEFT/RIGHT choice
        # on every re-entry. _recovery_turn() below calls _update_turn_sign(),
        # which keeps the existing direction unless the other side is
        # DWA_TURN_SWITCH_MARGIN clearer, and only falls back to a bare
        # clearance comparison when recovery_turn_sign is still None (a
        # genuinely fresh start, not a re-entry).
        self.recovery_phase = phase
        self.wall_side = None
        self.wall_clear_steps = 0
        self.wall_escape_active = False
        self.wall_front_turn_sign = None
        return self._recovery_turn(ranges)

    @staticmethod
    def _representative_distance(values):
        if not values:
            return float("inf")
        ordered = sorted(values)
        return ordered[len(ordered) // 2]

    def _side_distances(self, ranges):
        count = len(ranges)
        half_width = max(2, count // 18)
        right = self._representative_distance(
            self._sector_values(ranges, count // 4, half_width)
        )
        left = self._representative_distance(
            self._sector_values(ranges, 3 * count // 4, half_width)
        )
        return left, right

    def _update_side_speed_limit(self, ranges):
        count = len(ranges)
        side_width = max(2, count // 18)
        left_side = self._sector_values(ranges, 3 * count // 4, side_width)
        right_side = self._sector_values(ranges, count // 4, side_width)
        side_clearance = min(left_side + right_side, default=float("inf"))

        if side_clearance < DWA_SIDE_STOP_DISTANCE:
            self.side_yield_elapsed += TIME_STEP / 1000.0
            self.side_clear_steps = 0
        elif side_clearance >= DWA_SIDE_CAUTION_DISTANCE:
            if self.side_yield_elapsed:
                self.side_clear_steps += 1
                if self.side_clear_steps >= DWA_SIDE_CLEAR_STEPS:
                    self.side_yield_elapsed = 0.0
                    self.side_clear_steps = 0

        if self.side_yield_elapsed:
            if (
                side_clearance < DWA_SIDE_STOP_DISTANCE
                and self.side_yield_elapsed <= DWA_SIDE_YIELD_SECONDS
            ):
                return DWA_SIDE_CAUTION_SPEED
            if side_clearance < DWA_SIDE_CAUTION_DISTANCE:
                return DWA_SIDE_CAUTION_SPEED

        if side_clearance < DWA_SIDE_CAUTION_DISTANCE:
            return DWA_SIDE_CAUTION_SPEED
        return DWA_MAX_LINEAR_SPEED

    def _follow_wall(self, ranges, front_distance, front_arc_distance=None):
        left_distance, right_distance = self._side_distances(ranges)
        if self.wall_side is None:
            if left_distance == right_distance == float("inf"):
                self.wall_side = "right" if self.recovery_turn_sign > 0.0 else "left"
            else:
                self.wall_side = "left" if left_distance < right_distance else "right"

        # If the followed wall disappears and the opposite side becomes close,
        # update the wall side before choosing a turn away from it.
        if (
            self.wall_side == "left"
            and left_distance > DWA_WALL_LOST_DISTANCE
            and right_distance < DWA_WALL_FOLLOW_DISTANCE
        ):
            self.wall_side = "right"
            self.wall_clear_steps = 0
        elif (
            self.wall_side == "right"
            and right_distance > DWA_WALL_LOST_DISTANCE
            and left_distance < DWA_WALL_FOLLOW_DISTANCE
        ):
            self.wall_side = "left"
            self.wall_clear_steps = 0

        side_distance = left_distance if self.wall_side == "left" else right_distance
        away_sign = -1.0 if self.wall_side == "left" else 1.0

        if self.wall_escape_active:
            if side_distance >= DWA_WALL_RELEASE_DISTANCE:
                self.wall_escape_active = False
            else:
                self.current_v = 0.0
                self.current_w = away_sign * DWA_RECOVERY_YAW_RATE
                left, right = self._wheel_speeds(self.current_v, self.current_w)
                return left, right, "DWA 벽 안전거리 복구 중"
        elif side_distance < DWA_WALL_EMERGENCY_DISTANCE:
            self.wall_escape_active = True
            self.current_v = 0.0
            self.current_w = away_sign * DWA_RECOVERY_YAW_RATE
            left, right = self._wheel_speeds(self.current_v, self.current_w)
            return left, right, "DWA 벽 안전거리 복구 중"

        if front_arc_distance is None:
            front_arc_distance = self._front_arc_distance(ranges)
        if front_arc_distance < DWA_RECOVERY_CLEAR_DISTANCE:
            self.wall_front_turn_sign = self._update_turn_sign(
                ranges, self.wall_front_turn_sign
            )
            self.last_turn_sign = self.wall_front_turn_sign
            self.current_v = min(DWA_WALL_FOLLOW_SPEED, self.side_speed_limit)
            self.current_w = (
                self.wall_front_turn_sign * DWA_WALL_FOLLOW_TURN_RATE
            )
            left, right = self._wheel_speeds(self.current_v, self.current_w)
            return left, right, "DWA 벽 추종 - 저속 전진 회피"
        self.wall_front_turn_sign = None

        if side_distance > DWA_WALL_LOST_DISTANCE:
            self.wall_clear_steps += 1
        else:
            self.wall_clear_steps = 0

        if self.wall_clear_steps >= DWA_WALL_CLEAR_STEPS:
            self.recovery_phase = None
            self.recovery_turn_sign = None
            self.wall_side = None
            self.wall_clear_steps = 0
            self.wall_escape_active = False
            self.current_v = 0.0
            self.current_w = 0.0
            return None

        if math.isfinite(side_distance):
            error = DWA_WALL_FOLLOW_DISTANCE - side_distance
            correction = away_sign * DWA_WALL_FOLLOW_GAIN * error
            self.current_w = max(-0.6, min(0.6, correction))
        else:
            self.current_w = 0.0
        self.current_v = min(DWA_WALL_FOLLOW_SPEED, self.side_speed_limit)
        left, right = self._wheel_speeds(self.current_v, self.current_w)
        return left, right, f"DWA 벽 추종 - {self.wall_side} 벽"

    def _recovery_action(self, ranges, front_distance, front_arc_distance=None):
        if front_arc_distance is None:
            front_arc_distance = self._front_arc_distance(ranges)
        if self.recovery_phase == "emergency_turn":
            if front_arc_distance < DWA_RECOVERY_CLEAR_DISTANCE:
                return self._recovery_turn(ranges)
            self.recovery_phase = None
            self.recovery_turn_sign = None
            self.current_v = 0.0
            self.current_w = 0.0
            return None
        if self.recovery_phase == "wall_follow":
            return self._follow_wall(
                ranges, front_distance, front_arc_distance
            )
        if front_arc_distance < DWA_RECOVERY_CLEAR_DISTANCE:
            return self._recovery_turn(ranges)

        left_distance, right_distance = self._side_distances(ranges)
        if min(left_distance, right_distance) < DWA_WALL_LOST_DISTANCE:
            self.recovery_phase = "wall_follow"
            self.recovery_turn_sign = None
            return self._follow_wall(
                ranges, front_distance, front_arc_distance
            )

        self.recovery_phase = None
        self.recovery_turn_sign = None
        self.current_v = 0.0
        self.current_w = 0.0
        return None

    @staticmethod
    def _scan_to_points(ranges, field_of_view=LIDAR_FIELD_OF_VIEW,
                        sensor_pose=(0.0, 0.0, 0.0)):
        points = []
        count = len(ranges)
        if count == 0:
            return points

        denominator = max(1, count - 1)
        for index, distance in enumerate(ranges):
            if (
                not math.isfinite(distance)
                or distance < DWA_MIN_RANGE
                or distance > _SCAN_RELEVANT_DISTANCE
            ):
                continue
            angle = sensor_pose[2] - 0.5 * field_of_view + (
                field_of_view * index / denominator
            )
            points.append((sensor_pose[0] + distance * math.cos(angle),
                           sensor_pose[1] + distance * math.sin(angle)))
        return points

    @staticmethod
    def _cluster_world_points(world_points):
        clusters = []
        for point_index, point in enumerate(world_points):
            if not clusters:
                clusters.append([point_index])
                continue
            previous = world_points[clusters[-1][-1]]
            if math.hypot(point[0] - previous[0], point[1] - previous[1]) > DWA_DYNAMIC_CLUSTER_GAP:
                clusters.append([point_index])
            else:
                clusters[-1].append(point_index)
        return clusters

    def _scan_with_dynamic_obstacles(self, ranges, pose, sim_time):
        points = self._scan_to_points(ranges, self.lidar_field_of_view, self.lidar_pose)
        if pose is None or sim_time is None:
            self.previous_scan_clusters = None
            self.previous_scan_time = None
            self.previous_scan_heading = None
            self.last_dynamic_obstacles = []
            return points

        pose_x, pose_y, heading = pose
        cosine = math.cos(heading)
        sine = math.sin(heading)

        observed_yaw_rate = 0.0
        if self.previous_scan_heading is not None and self.previous_scan_time is not None:
            elapsed_for_yaw = sim_time - self.previous_scan_time
            if 0.0 < elapsed_for_yaw <= 0.25:
                delta_heading = math.atan2(
                    math.sin(heading - self.previous_scan_heading),
                    math.cos(heading - self.previous_scan_heading),
                )
                observed_yaw_rate = delta_heading / elapsed_for_yaw
                if abs(observed_yaw_rate) >= DWA_EGO_ROTATION_SUPPRESS_RATE:
                    # Match the velocity-estimate history window (up to
                    # ~0.32 s): the *velocity estimate* still reflects a fast
                    # turn for as long as that turn is inside its averaging
                    # window, even on the very frame the turn itself stops.
                    self._ego_rotation_cooldown = 0.35
                else:
                    self._ego_rotation_cooldown = max(
                        0.0, self._ego_rotation_cooldown - elapsed_for_yaw
                    )
        if (
            not points
            and self.previous_scan_clusters is not None
            and self.previous_scan_time is not None
            and 0.0 < sim_time - self.previous_scan_time <= DWA_DYNAMIC_TRACK_HOLD_SECONDS
        ):
            elapsed = sim_time - self.previous_scan_time
            held = []
            for world_x, world_y, _, _, velocity_x, velocity_y, hits, last_motion in self.previous_scan_clusters:
                if hits < 2 or sim_time - last_motion > 2.5:
                    continue
                delta_x = world_x + velocity_x * elapsed - pose_x
                delta_y = world_y + velocity_y * elapsed - pose_y
                held.append((
                    cosine * delta_x + sine * delta_y,
                    -sine * delta_x + cosine * delta_y,
                    cosine * velocity_x + sine * velocity_y,
                    -sine * velocity_x + cosine * velocity_y,
                ))
            if held:
                self.last_dynamic_obstacles = held
                return held
        world_points = [
            (
                pose_x + cosine * point_x - sine * point_y,
                pose_y + sine * point_x + cosine * point_y,
            )
            for point_x, point_y in points
        ]

        world_clusters = self._cluster_world_points(world_points)
        cluster_centers = []
        for indices in world_clusters:
            first = world_points[indices[0]]
            last = world_points[indices[-1]]
            cluster_centers.append(
                (
                    sum(world_points[index][0] for index in indices) / len(indices),
                    sum(world_points[index][1] for index in indices) / len(indices),
                    len(indices),
                    math.hypot(last[0] - first[0], last[1] - first[1]),
                    0.0, 0.0, 0, float('-inf'),
                )
            )

        histories = [[(sim_time, center[0], center[1])] for center in cluster_centers]

        dynamic_velocities = {}
        if self.previous_scan_clusters is not None and self.previous_scan_time is not None:
            delta_time = sim_time - self.previous_scan_time
            if 0.0 < delta_time <= 0.25:
                maximum_displacement = (
                    DWA_DYNAMIC_MAX_SPEED * delta_time
                    + DWA_DYNAMIC_MATCH_DISTANCE
                )
                matched_previous = set()
                for cluster_index, (world_x, world_y, point_count, span, _, _, _, _) in enumerate(
                    cluster_centers
                ):
                    if not (
                        DWA_DYNAMIC_MIN_CLUSTER_POINTS
                        <= point_count
                        <= 64 and span <= 0.55
                    ):
                        continue
                    nearest_index = None
                    nearest_distance_squared = maximum_displacement**2
                    for previous_index, (previous_x, previous_y, previous_count,
                                         previous_span, _, _, _, _) in enumerate(
                        self.previous_scan_clusters
                    ):
                        if previous_index in matched_previous:
                            continue
                        if not (
                            DWA_DYNAMIC_MIN_CLUSTER_POINTS
                            <= previous_count
                            <= 64 and previous_span <= 0.55
                        ):
                            continue
                        count_ratio = point_count / previous_count
                        if count_ratio < 0.35 or count_ratio > 3.0:
                            continue
                        distance_squared = (world_x - previous_x) ** 2 + (
                            world_y - previous_y
                        ) ** 2
                        if distance_squared < nearest_distance_squared:
                            nearest_index = previous_index
                            nearest_distance_squared = distance_squared
                    if nearest_index is None:
                        continue

                    matched_previous.add(nearest_index)
                    old_history = (self.previous_cluster_histories[nearest_index]
                                   if nearest_index < len(self.previous_cluster_histories) else [])
                    history = [sample for sample in old_history if sim_time - sample[0] <= .32]
                    history.append((sim_time, world_x, world_y))
                    histories[cluster_index] = history
                    if len(history) < 3 or history[-1][0] - history[0][0] < .16:
                        continue
                    mean_t = sum(sample[0] for sample in history) / len(history)
                    mean_x = sum(sample[1] for sample in history) / len(history)
                    mean_y = sum(sample[2] for sample in history) / len(history)
                    variance = sum((sample[0] - mean_t)**2 for sample in history)
                    velocity_world_x = sum((t - mean_t) * (x - mean_x) for t, x, y in history) / variance
                    velocity_world_y = sum((t - mean_t) * (y - mean_y) for t, x, y in history) / variance
                    speed = math.hypot(velocity_world_x, velocity_world_y)
                    previous_vx, previous_vy, previous_hits, last_motion = (
                        self.previous_scan_clusters[nearest_index][4:]
                    )
                    was_moving = previous_hits >= 2 and sim_time - last_motion <= 2.5
                    # A fast in-place turn (recovery/wall-follow) can make a
                    # static corner's visible surface point slide across
                    # frames, reconstructing a plausible-looking world-frame
                    # velocity for an obstacle that never moved. The velocity
                    # estimate keeps reflecting that turn for as long as it
                    # remains inside its averaging history window (up to
                    # ~0.32 s), even on the frame the turn itself stops, so
                    # the gate uses a matching cooldown rather than the
                    # instantaneous yaw rate. Only gate the *first* promotion
                    # on this -- an already-confirmed mover (was_moving)
                    # keeps being tracked regardless of the robot's own
                    # rotation, so a real crossing actor is never lost
                    # mid-turn.
                    ego_rotating_fast = self._ego_rotation_cooldown > 0.0
                    if speed <= DWA_DYNAMIC_MAX_SPEED and (
                        (speed >= DWA_DYNAMIC_MIN_SPEED and not ego_rotating_fast)
                        or was_moving
                    ):
                        if previous_hits and not was_moving and (
                            velocity_world_x * previous_vx
                            + velocity_world_y * previous_vy < 0.0
                        ):
                            previous_hits = 0
                        hits = min(3, previous_hits + 1)
                        if speed >= DWA_DYNAMIC_MIN_SPEED:
                            last_motion = sim_time
                        cluster_centers[cluster_index] = (
                            world_x, world_y, point_count, span,
                            velocity_world_x, velocity_world_y, hits, last_motion,
                        )
                        if hits < 2 and not was_moving:
                            continue
                        local_velocity = (
                            cosine * velocity_world_x + sine * velocity_world_y,
                            -sine * velocity_world_x + cosine * velocity_world_y,
                        )
                        for point_index in world_clusters[cluster_index]:
                            dynamic_velocities[point_index] = local_velocity

        self.previous_scan_clusters = cluster_centers
        self.previous_cluster_histories = histories
        self.previous_scan_time = sim_time
        self.previous_scan_heading = heading
        obstacles = [
            (point_x, point_y, *dynamic_velocities[index])
            if index in dynamic_velocities
            else (point_x, point_y)
            for index, (point_x, point_y) in enumerate(points)
        ]
        self.last_dynamic_obstacles = [
            obstacle for obstacle in obstacles if len(obstacle) == 4
        ]
        return obstacles

    @staticmethod
    def _wheel_speeds(v, w):
        # 이 practice 로봇에서 +w(좌회전)는 오른쪽 바퀴가 더 빨라야 한다.
        left = (v - 0.5 * WHEEL_TRACK * w) / WHEEL_RADIUS
        right = (v + 0.5 * WHEEL_TRACK * w) / WHEEL_RADIUS

        peak = max(abs(left), abs(right))
        if peak > MAX_SPEED:
            scale = MAX_SPEED / peak
            left *= scale
            right *= scale
        left = max(-MAX_SPEED, min(MAX_SPEED, left))
        right = max(-MAX_SPEED, min(MAX_SPEED, right))
        return left, right

    @staticmethod
    def _achievable_velocity(v, w):
        """Use the velocity the motors can actually command in rollouts."""
        left, right = DynamicWindowAvoidance._wheel_speeds(v, w)
        actual_v = 0.5 * WHEEL_RADIUS * (left + right)
        actual_w = WHEEL_RADIUS * (right - left) / WHEEL_TRACK
        return actual_v, actual_w, left, right

    @staticmethod
    def _simulate(v, w):
        x = 0.0
        y = 0.0
        heading = 0.0
        trajectory = []
        steps = max(1, math.ceil(DWA_HORIZON / DWA_SIMULATION_STEP))
        for _ in range(steps):
            x += v * math.cos(heading) * DWA_SIMULATION_STEP
            y += v * math.sin(heading) * DWA_SIMULATION_STEP
            heading += w * DWA_SIMULATION_STEP
            trajectory.append((x, y, heading))
        return trajectory

    @staticmethod
    def _trajectory_clearance(trajectory, obstacles, dynamic_horizon=1.2):
        if not obstacles:
            return float("inf")

        minimum_squared = float("inf")
        collision_squared = DWA_ROBOT_RADIUS * DWA_ROBOT_RADIUS
        for step_index, (x, y, _) in enumerate(trajectory):
            future_time = (step_index + 1) * DWA_SIMULATION_STEP
            for obstacle in obstacles:
                obstacle_x, obstacle_y = obstacle[:2]
                if len(obstacle) == 4:
                    # A measured velocity is useful for an imminent crossing,
                    # but it is not a known route. Reobserve before planning
                    # farther than this short interval.
                    if future_time > dynamic_horizon:
                        continue
                    obstacle_x += obstacle[2] * future_time
                    obstacle_y += obstacle[3] * future_time
                distance_squared = (obstacle_x - x) ** 2 + (obstacle_y - y) ** 2
                if distance_squared <= collision_squared:
                    return 0.0
                if distance_squared < minimum_squared:
                    minimum_squared = distance_squared
        return math.sqrt(minimum_squared)

    @staticmethod
    def _dynamic_reachable_clearance(trajectory, moving):
        """Distance to the LiDAR velocity estimate plus turn uncertainty."""
        if not moving:
            return float("inf")
        closest = float("inf")
        for step, (x, y, _) in enumerate(trajectory, 1):
            future = step * DWA_SIMULATION_STEP
            if future > 1.2:
                break
            uncertainty = 0.5 * DWA_DYNAMIC_ACCELERATION_BOUND * future**2
            for ox, oy, vx, vy in moving:
                projected = math.hypot(
                    ox + vx * future - x, oy + vy * future - y
                )
                observed = math.hypot(ox - x, oy - y)
                separation = min(projected, observed) - uncertainty
                closest = min(closest, separation)
        return closest

    def is_command_safe(self, left, right):
        """Check a path follower wheel command against the latest LiDAR scan.

        Call choose_action first on every control step so moving-cluster tracks
        stay current. The integration layer can then select the normal command
        when it is safe and the DWA command otherwise.
        """
        if self.last_obstacles is None:
            return False
        if (
            not math.isfinite(left)
            or not math.isfinite(right)
            or abs(left) > MAX_SPEED
            or abs(right) > MAX_SPEED
        ):
            return False
        velocity = 0.5 * (left + right) * WHEEL_RADIUS
        yaw_rate = (right - left) * WHEEL_RADIUS / WHEEL_TRACK
        return self._velocity_is_safe(velocity, yaw_rate, self.last_obstacles)

    def is_release_ready(self, left, right):
        """Path-follow release gate with hysteresis.

        A bare is_command_safe() check on its own can ping-pong control
        every single frame near an obstacle corner: the nominal turn is
        judged safe, gets applied, that turn changes the heading enough that
        the very next scan judges it unsafe again, avoidance turns back, and
        the cycle repeats without the robot ever clearing the corner. This
        requires DWA_PATH_RELEASE_CONFIRM_STEPS *consecutive* safe frames
        before releasing control back to the path follower. The streak
        starts pre-filled (see __init__) so a path that was never
        interrupted incurs no delay; only an actual unsafe frame resets it,
        so only a real avoidance episode pays the confirmation delay on the
        way back out.

        A single-frame is_command_safe() check is not enough on its own,
        though: while the scripted "turn" recovery phase or a backoff is
        active, the nominal turn toward the goal can be transiently judged
        "safe" for one frame while the corner has not actually been cleared
        yet (e.g. mid-turn, before the turn has actually opened up the
        corner). Releasing on that coincidence hands control back before the
        corner is cleared, which is exactly the ping-pong this gate exists
        to prevent. So while "turn" or a backoff is active, never release --
        and keep the streak at zero so confirmation starts fresh once they
        actually end.

        Plain wall-follow cruising (wall_side set, wall_escape_active False)
        is deliberately NOT included in that gate: it can end up tracing the
        perimeter of a square obstacle indefinitely (each face looks like "a
        wall is still here" to its own exit condition), and the plain
        hysteresis below is what lets a nominal command that has been safe
        for DWA_PATH_RELEASE_CONFIRM_STEPS consecutive frames break out of
        that loop rather than orbiting forever. wall_escape_active (the
        "too close to the wall, turning away right now" sub-state) is
        included, though, for the same corner-flicker reason as "turn" --
        via a short cooldown rather than the instantaneous flag, since
        wall_escape_active itself can flip True/False every few frames right
        at its own distance thresholds when the robot is pinned close to an
        obstacle, and releasing on a single False frame just re-triggers it
        one frame later. So is
        _last_rollout_had_no_safe_candidate: the normal DWA rollout finding
        zero safe candidates and falling back to an emergency turn is the
        same "not actually clear yet" situation as recovery_phase=="turn",
        just reached from the rollout instead of the scripted recovery state
        machine.
        """
        active_recovery = (
            self.recovery_phase in ("turn", "emergency_turn")
            or self.backoff_steps > 0
            or self._last_rollout_had_no_safe_candidate
            or self._wall_escape_cooldown > 0.0
            or self._front_corner_escape_active
        )
        if active_recovery or not self.is_command_safe(left, right):
            self._path_release_streak = 0
            return False
        self._path_release_streak = min(
            DWA_PATH_RELEASE_CONFIRM_STEPS, self._path_release_streak + 1
        )
        return self._path_release_streak >= DWA_PATH_RELEASE_CONFIRM_STEPS

    def _velocity_is_safe(self, velocity, yaw_rate, obstacles):
        """Apply the common static/dynamic collision model to one command."""
        if not math.isfinite(velocity) or not math.isfinite(yaw_rate):
            return False
        if obstacles is None:
            return False
        trajectory = self._simulate(velocity, yaw_rate)
        nearby = trajectory[:max(1, round(1.2 / DWA_SIMULATION_STEP))]
        fixed = [point for point in obstacles if len(point) == 2]
        moving = [point for point in obstacles if len(point) == 4]
        static_safe = self._static_motion_is_safe(nearby, fixed)
        dynamic_clearance = self._dynamic_reachable_clearance(trajectory, moving)
        return (
            static_safe
            and dynamic_clearance > DWA_ROBOT_RADIUS + DWA_DYNAMIC_CLEARANCE_MARGIN
        )

    @staticmethod
    def _static_motion_is_safe(trajectory, fixed):
        """Check swept static clearance, including safe motion out of a margin."""
        if not fixed:
            return True
        margin_distance = DWA_ROBOT_RADIUS + DWA_STATIC_CLEARANCE_MARGIN
        for obstacle_x, obstacle_y in fixed:
            initial = math.hypot(obstacle_x, obstacle_y)
            distances = [
                math.hypot(obstacle_x - x, obstacle_y - y)
                for x, y, _ in trajectory
            ]
            if min(distances) <= DWA_ROBOT_RADIUS:
                return False
            if initial <= margin_distance:
                # A command may leave a caution margin only when every sampled
                # step moves away. Rotation or motion toward any close point is
                # rejected; this makes a fully enclosed 0.20 m scan stop.
                if min(distances) < initial - 1e-6:
                    return False
                if distances[-1] <= initial + 1e-3:
                    return False
            elif min(distances) <= margin_distance:
                return False
        return True

    def _finalize_action(self, result, obstacles, unsafe_message="DWA 안전 경로 없음 - 정지"):
        """Last gate for every command that can reach the motor integration."""
        left, right, description = result
        if (
            math.isfinite(left)
            and math.isfinite(right)
            and abs(left) <= MAX_SPEED
            and abs(right) <= MAX_SPEED
        ):
            velocity = 0.5 * (left + right) * WHEEL_RADIUS
            yaw_rate = (right - left) * WHEEL_RADIUS / WHEEL_TRACK
            if self._velocity_is_safe(velocity, yaw_rate, obstacles):
                return left, right, description
        self.current_v = 0.0
        self.current_w = 0.0
        return 0.0, 0.0, unsafe_message

    @staticmethod
    def _sensor_clearances(trajectories, fixed, moving):
        """Evaluate the same collision model for all candidates in one batch."""
        near_steps = max(1, round(1.2 / DWA_SIMULATION_STEP))
        if np is None:
            return (
                [DynamicWindowAvoidance._trajectory_clearance(t[:near_steps], fixed)
                 for t in trajectories],
                [DynamicWindowAvoidance._dynamic_reachable_clearance(t, moving)
                 for t in trajectories],
            )
        paths = np.asarray(trajectories, dtype=float)[:, :, :2]
        static = np.full(len(paths), np.inf)
        dynamic = np.full(len(paths), np.inf)
        if fixed:
            delta = paths[:, :near_steps, None, :] - np.asarray(fixed)[None, None, :, :2]
            static = np.hypot(delta[..., 0], delta[..., 1]).min(axis=(1, 2))
        if moving:
            steps = min(paths.shape[1], int(1.2 / DWA_SIMULATION_STEP))
            times = np.arange(1, steps + 1) * DWA_SIMULATION_STEP
            objects = np.asarray(moving)
            projected = objects[None, :, :2] + times[:, None, None] * objects[None, :, 2:4]
            delta = paths[:, :steps, None, :] - projected[None, :, :, :]
            distance = np.hypot(delta[..., 0], delta[..., 1])
            current_delta = paths[:, :steps, None, :] - objects[None, None, :, :2]
            current_distance = np.hypot(current_delta[..., 0], current_delta[..., 1])
            distance = np.minimum(distance, current_distance)
            distance -= (0.5 * DWA_DYNAMIC_ACCELERATION_BOUND * times**2)[None, :, None]
            dynamic = distance.min(axis=(1, 2))
        return static, dynamic

    @staticmethod
    def _score(v, trajectory, clearance, goal):
        x, y, heading = trajectory[-1]
        goal_x, goal_y = goal
        goal_heading = math.atan2(goal_y - y, goal_x - x)
        heading_error = math.atan2(
            math.sin(goal_heading - heading), math.cos(goal_heading - heading)
        )
        heading_score = 0.5 * (math.cos(heading_error) + 1.0)
        speed_score = v / DWA_MAX_LINEAR_SPEED
        max_progress = max(0.001, DWA_MAX_LINEAR_SPEED * DWA_HORIZON)
        goal_distance = max(0.001, math.hypot(goal_x, goal_y))
        progress = (x * goal_x + y * goal_y) / goal_distance
        progress_score = max(-1.0, min(1.0, progress / max_progress))
        if math.isfinite(clearance):
            clearance_score = max(
                0.0, min(1.0, (clearance - DWA_ROBOT_RADIUS) / 0.8)
            )
        else:
            clearance_score = 1.0
        return (
            DWA_HEADING_WEIGHT * heading_score
            + DWA_CLEARANCE_WEIGHT * clearance_score
            + DWA_SPEED_WEIGHT * speed_score
            + DWA_PROGRESS_WEIGHT * progress_score
        )

    @staticmethod
    def _sensor_score_terms(
        v, w, trajectory, clearance, goal, prefer_forward=False
    ):
        """Return the sensor-mode score and its independently logged terms."""
        target_x, target_y = goal
        target_heading = math.atan2(target_y, target_x)
        step = min(
            len(trajectory) - 1,
            round(1.0 / DWA_SIMULATION_STEP) - 1,
        )
        px, py, ph = trajectory[step]
        heading_error = math.atan2(
            math.sin(target_heading - ph),
            math.cos(target_heading - ph),
        )
        goal_distance_contribution = -5.0 * math.hypot(
            px - target_x, py - target_y
        )
        heading_contribution = 0.3 * math.cos(heading_error)
        clearance_contribution = 0.1 * min(1.0, clearance)
        speed_contribution = (
            DWA_SPEED_WEIGHT * v / DWA_MAX_LINEAR_SPEED
            if prefer_forward
            else 0.0
        )
        stop_penalty = (
            -10.0
            if v == 0.0 and w == 0.0 and math.hypot(target_x, target_y) > 0.20
            else 0.0
        )
        return {
            "goal_distance": goal_distance_contribution,
            "heading": heading_contribution,
            "clearance": clearance_contribution,
            "speed": speed_contribution,
            "stop_penalty": stop_penalty,
            "total": (
                goal_distance_contribution
                + heading_contribution
                + clearance_contribution
                + speed_contribution
                + stop_penalty
            ),
        }

    def _emergency_turn(self, ranges):
        return self._start_recovery(ranges, "emergency_turn")

    def _front_corner_backoff_action(self, obstacles):
        """Prefer an away-curving retreat, falling back to straight reverse."""
        turn_sign = self.recovery_turn_sign or 1.0
        candidates = (
            (-DWA_BACKOFF_SPEED, turn_sign * DWA_WALL_FOLLOW_TURN_RATE),
            (-DWA_BACKOFF_SPEED, -turn_sign * DWA_WALL_FOLLOW_TURN_RATE),
            (-DWA_BACKOFF_SPEED, 0.0),
        )
        for velocity, yaw_rate in candidates:
            if not self._velocity_is_safe(velocity, yaw_rate, obstacles):
                continue
            self.current_v = velocity
            self.current_w = yaw_rate
            if yaw_rate != 0.0:
                self.recovery_turn_sign = 1.0 if yaw_rate > 0.0 else -1.0
            left, right = self._wheel_speeds(velocity, yaw_rate)
            return left, right, "DWA front-corner zero-velocity escape"
        self.current_v = 0.0
        self.current_w = 0.0
        return None

    def _start_front_corner_escape(self, ranges, obstacles, pose):
        """Create clearance for the strictly detected zero-forward trap."""
        self._front_corner_escape_active = True
        if self._front_corner_escape_origin is None:
            self._front_corner_escape_origin = pose[:2]
        if self.recovery_turn_sign is None:
            self.recovery_turn_sign = self._select_turn_sign(ranges)
        self.recovery_phase = "turn"
        action = self._front_corner_backoff_action(obstacles)
        # Count displacement-producing backoff commands, never requested
        # commands that the common safety gate would replace with STOP.
        self.backoff_steps = 1 if action is not None else 0
        if action is not None:
            return action
        return 0.0, 0.0, "DWA front-corner escape blocked - stop"

    def _validate_recovery_action(self, action, obstacles):
        action_left, action_right, action_description = action
        action_v = 0.5 * (action_left + action_right) * WHEEL_RADIUS
        action_w = (action_right - action_left) * WHEEL_RADIUS / WHEEL_TRACK
        turn_sign = 1.0 if action_w >= 0.0 else -1.0
        candidates = (
            (action_v, action_w, action_description),
            (action_v, -action_w, "DWA 복구 반대 곡선"),
            (0.0, turn_sign * DWA_RECOVERY_YAW_RATE, "DWA 안전 제자리 회전"),
            (0.0, -turn_sign * DWA_RECOVERY_YAW_RATE, "DWA 안전 제자리 회전"),
        )
        for velocity, yaw_rate, description in candidates:
            actual_v, actual_w, left, right = self._achievable_velocity(
                velocity, yaw_rate
            )
            if not self._velocity_is_safe(actual_v, actual_w, obstacles):
                continue

            self.current_v = actual_v
            self.current_w = actual_w
            selected_sign = 1.0 if actual_w >= 0.0 else -1.0
            if self.wall_front_turn_sign is not None:
                self.wall_front_turn_sign = selected_sign
            elif self.recovery_turn_sign is not None:
                self.recovery_turn_sign = selected_sign
            self.last_turn_sign = selected_sign
            return left, right, description

        self.current_v = 0.0
        self.current_w = 0.0
        if any(len(obstacle) == 4 for obstacle in obstacles):
            return 0.0, 0.0, "DWA 횡단 장애물 양보"
        return 0.0, 0.0, "DWA 복구 안전 경로 없음 - 정지"

    def _reset_crossing_state(self):
        self.crossing_waiting = False
        self.crossing_gap_steps = 0
        self.crossing_commit_active = False
        self.crossing_commit_remaining = 0.0
        self.crossing_commit_last_pose = None
        self.crossing_departure_observed = False
        self._crossing_exit_distance = DWA_CROSSING_COMMIT_MIN_DISTANCE

    def _record_crossing_zone(self, obstacle_x):
        distance = obstacle_x - DWA_CROSSING_CLEARING_X_RANGE[0]
        self._crossing_exit_distance = max(
            DWA_CROSSING_COMMIT_MIN_DISTANCE,
            min(DWA_CROSSING_COMMIT_MAX_DISTANCE, distance),
        )

    def _start_crossing_commit(self, pose):
        self.crossing_waiting = False
        self.crossing_gap_steps = 0
        self.crossing_commit_active = True
        self.crossing_commit_remaining = self._crossing_exit_distance
        self.crossing_commit_last_pose = pose

    def _update_crossing_commit_progress(self, pose):
        """Consume committed distance using forward odometry, not elapsed time."""
        if not self.crossing_commit_active or pose is None:
            return
        previous = self.crossing_commit_last_pose
        self.crossing_commit_last_pose = pose
        if previous is None:
            return
        dx = pose[0] - previous[0]
        dy = pose[1] - previous[1]
        forward = dx * math.cos(previous[2]) + dy * math.sin(previous[2])
        self.crossing_commit_remaining -= max(0.0, forward)
        if self.crossing_commit_remaining <= 0.0:
            self._reset_crossing_state()

    def _crossing_gap_is_acceptable(self, moving_obstacles, obstacles):
        """Require a stable, observed gap before entering a crossing."""
        relevant = [
            obstacle for obstacle in moving_obstacles
            if (
                DWA_CROSSING_CLEARING_X_RANGE[0] <= obstacle[0]
                <= DWA_CROSSING_YIELD_X_RANGE[1]
            )
        ]
        for _, oy, _, vy in relevant:
            lateral_gap = abs(oy) - DWA_CROSSING_LANE_HALF_WIDTH
            if abs(oy) < DWA_CROSSING_GAP_LATERAL_DISTANCE or lateral_gap <= 0.0:
                return False
            lateral_speed = max(abs(vy), DWA_CROSSING_LATERAL_SPEED)
            if lateral_gap / lateral_speed < DWA_CROSSING_GAP_MIN_SECONDS:
                return False
            if oy * vy > 0.0:
                self.crossing_departure_observed = True
            elif oy * vy < 0.0:
                # A second obstacle may approach, provided it cannot reach the
                # lane inside the configured accepted-gap interval.
                time_to_lane = lateral_gap / lateral_speed
                if time_to_lane <= DWA_CROSSING_GAP_MIN_SECONDS:
                    return False

        if not self.crossing_departure_observed:
            return False
        check_obstacles = moving_obstacles if obstacles is None else obstacles
        return self._velocity_is_safe(
            DWA_CROSSING_COMMIT_SPEED, 0.0, check_obstacles
        )

    def _crossing_commit_collision_is_imminent(self, obstacles):
        """Abort only when braking cannot avoid the emergency envelope."""
        if obstacles is None:
            return True
        steps = max(
            1,
            math.ceil(
                DWA_CROSSING_COMMIT_CANCEL_SECONDS / DWA_SIMULATION_STEP
            ),
        )
        x = 0.0
        velocity = max(0.0, self.current_v)
        imminent_path = []
        for _ in range(steps):
            velocity = max(
                0.0,
                velocity - DWA_LINEAR_ACCELERATION * DWA_SIMULATION_STEP,
            )
            x += velocity * DWA_SIMULATION_STEP
            imminent_path.append((x, 0.0, 0.0))
        fixed = [point for point in obstacles if len(point) == 2]
        moving = [point for point in obstacles if len(point) == 4]
        emergency_clearance = DWA_ROBOT_RADIUS + DWA_EMERGENCY_CLEARANCE_MARGIN
        return (
            self._trajectory_clearance(imminent_path, fixed)
            <= emergency_clearance
            or self._dynamic_reachable_clearance(imminent_path, moving)
            <= emergency_clearance
        )

    def _crossing_commit_action(self, obstacles):
        actual_v, actual_w, left, right = self._achievable_velocity(
            DWA_CROSSING_COMMIT_SPEED, 0.0
        )
        self.current_v = actual_v
        self.current_w = actual_w
        return self._finalize_action(
            (left, right, "DWA crossing commit"), obstacles
        )

    def _evaluate_crossing_hazard(
        self, moving_obstacles, rear_distance, obstacles=None, pose=None,
    ):
        """측면 접근 및 전방 횡단 동적 장애물에 대한 선제적 회피 및 대기 판단."""
        check_obstacles = moving_obstacles if obstacles is None else obstacles
        if self.crossing_commit_active:
            if not self._crossing_commit_collision_is_imminent(check_obstacles):
                return self._crossing_commit_action(check_obstacles)
            self._reset_crossing_state()

        clearing_threat = None
        imminent_threat = None
        crossing_threat = None
        min_imminent_dist = float("inf")
        min_crossing_dist = float("inf")

        for ox, oy, vx, vy in moving_obstacles:
            dist = math.hypot(ox, oy)
            if dist > DWA_CROSSING_DETECTION_DISTANCE:
                continue

            speed = math.hypot(vx, vy)
            # 속도가 매우 낮을 때(반전 구간)는 차선 내에 위치한 경우 대기/후진 판단
            if speed < DWA_CROSSING_STATIONARY_SPEED:
                if (
                    DWA_CROSSING_CLEARING_X_RANGE[1]
                    <= ox < DWA_CROSSING_BACKOFF_DISTANCE
                    and abs(oy) < DWA_CROSSING_LANE_HALF_WIDTH
                ):
                    imminent_threat = (ox, oy, vx, vy)
                elif DWA_CROSSING_BACKOFF_DISTANCE <= ox <= DWA_CROSSING_YIELD_DISTANCE and abs(oy) < DWA_CROSSING_LANE_HALF_WIDTH:
                    crossing_threat = (ox, oy, vx, vy)
                continue

            # 측면 접근/이탈 상태 판정
            is_lateral_closing = (
                oy * vy < 0.0
                and abs(vy) >= DWA_CROSSING_LATERAL_SPEED
                and (abs(oy) < DWA_CROSSING_LANE_HALF_WIDTH or dist <= DWA_SIDE_APPROACH_DISTANCE)
            )
            is_moving_away = oy * vy > 0.0 and abs(vy) >= DWA_CROSSING_LATERAL_SPEED

            # 로봇 차선(폭 0.25) 점유 여부:
            # 장애물이 차선 내에 있더라도 중심선을 지나 바깥으로 이동 중(is_moving_away and abs(oy) > 0.08)이면
            # 차선이 곧 완전히 열리므로 블로킹으로 취급하지 않음.
            blocking_lane = abs(oy) < DWA_CROSSING_LANE_HALF_WIDTH and not (
                is_moving_away and abs(oy) > DWA_CROSSING_LANE_HALF_WIDTH / 3.0
            )

            # 교차 예상 시간 및 교차점 x 좌표 계산
            if is_lateral_closing:
                t_cross = -oy / vy
                x_cross = ox + vx * t_cross
            else:
                t_cross = float("inf")
                x_cross = ox

            # 1. 횡단 구역 진입 완료 후 신속 통과:
            # 로봇 앞범퍼가 이미 장애물 중심선(-0.25 <= ox < 0.05)을 통과한 경우:
            # 후진하면 교차선상으로 되돌아가 충돌하므로, 전방으로 신속히 탈출하여 통과
            if DWA_CROSSING_CLEARING_X_RANGE[0] <= ox < DWA_CROSSING_CLEARING_X_RANGE[1] and dist < DWA_CROSSING_CLEARING_DISTANCE:
                if is_lateral_closing or abs(oy) < DWA_CROSSING_CLEARING_LATERAL_DISTANCE:
                    clearing_threat = (ox, oy, vx, vy)

            # 2. 초근접 전측방 충돌 위협:
            # - 장애물이 로봇 바로 앞(0.05 <= ox < 0.36)에 있고,
            #   (a) 차선을 막고 있거나,
            #   (b) 곧(t_cross <= 1.2s) 로봇 전방으로 진입하는 경우
            # - 로봇 전방에 안전 정지 거리를 확보하기 위해 직선 후진 회피
            elif DWA_CROSSING_CLEARING_X_RANGE[1] <= ox < DWA_CROSSING_BACKOFF_DISTANCE:
                if (
                    blocking_lane
                    or (is_lateral_closing and 0.0 <= t_cross <= DWA_CROSSING_PREDICTION_SECONDS and 0.0 <= x_cross < DWA_CROSSING_IMMINENT_X_MARGIN)
                ):
                    if dist < min_imminent_dist:
                        min_imminent_dist = dist
                        imminent_threat = (ox, oy, vx, vy)

            # 3. 전방 정지선 대기 위협:
            # - 로봇이 정지선(0.36m ~ 0.70m)에 있고,
            #   (a) 장애물이 차선을 막고 있거나,
            #   (b) 곧(t_cross <= 1.5s) 전방 안전 구간으로 진입하는 경우
            # - 장애물이 지나갈 때까지 정지선에서 대기 후 이탈 시 즉시 출발
            elif DWA_CROSSING_BACKOFF_DISTANCE <= ox <= DWA_CROSSING_YIELD_DISTANCE:
                if (
                    blocking_lane
                    or (is_lateral_closing and 0.0 <= t_cross <= DWA_CROSSING_PREDICTION_SECONDS and DWA_CROSSING_YIELD_X_RANGE[0] <= x_cross <= DWA_CROSSING_YIELD_X_RANGE[1])
                ):
                    if dist < min_crossing_dist:
                        min_crossing_dist = dist
                        crossing_threat = (ox, oy, vx, vy)

        # A. 초근접 전측방 위협이 다른 clearing/yield 판단보다 우선한다.
        if imminent_threat is not None:
            self.crossing_waiting = True
            self.crossing_gap_steps = 0
            self.crossing_departure_observed = False
            self._record_crossing_zone(imminent_threat[0])
            if rear_distance > DWA_BACKOFF_REAR_DISTANCE:
                self.current_v = -DWA_MAX_REVERSE_SPEED
                self.current_w = 0.0  # 직진 후진으로 복도 내 자세 유지
                left, right = self._wheel_speeds(self.current_v, self.current_w)
                return self._finalize_action(
                    (left, right, "DWA 측면 접근 장애물 회피 후진"),
                    moving_obstacles if obstacles is None else obstacles,
                )
            self.current_v = 0.0
            self.current_w = 0.0
            return 0.0, 0.0, "DWA 전후방 차단 - 정지"

        # B. 이미 교차 구역에 진입한 경우: 검증된 전방 trajectory로만 탈출
        if clearing_threat is not None:
            self.current_v = DWA_MAX_LINEAR_SPEED
            self.current_w = 0.0
            left, right = self._wheel_speeds(self.current_v, self.current_w)
            return self._finalize_action(
                (left, right, "DWA 횡단 구역 신속 통과"),
                moving_obstacles if obstacles is None else obstacles,
            )

        # C. 전방 횡단 위협 시: 교차 구역 진입 전 안전 거리에서 대기
        if crossing_threat is not None:
            self.crossing_waiting = True
            self.crossing_gap_steps = 0
            self.crossing_departure_observed = False
            self._record_crossing_zone(crossing_threat[0])
            self.current_v = 0.0
            self.current_w = 0.0
            return self._finalize_action(
                (0.0, 0.0, "DWA 횡단 장애물 안전 거리 대기"),
                moving_obstacles if obstacles is None else obstacles,
            )

        if self.crossing_waiting:
            zone_has_tracked_mover = any(
                DWA_CROSSING_CLEARING_X_RANGE[0] <= obstacle[0]
                <= DWA_CROSSING_YIELD_X_RANGE[1]
                for obstacle in moving_obstacles
            )
            if not zone_has_tracked_mover:
                # The track that triggered the wait is no longer tracked as a
                # mover (it may never have been a real one - e.g. a transient
                # LiDAR cluster near a static edge during a sharp turn). There
                # is nothing left to wait on a "departure" from, so release
                # the wait and let the normal DWA rollout/recovery handle
                # whatever obstacle remains, rather than stalling forever or
                # forcing a straight-line commit dash into it.
                self._reset_crossing_state()
                return None
            if self._crossing_gap_is_acceptable(
                moving_obstacles, check_obstacles
            ):
                self.crossing_gap_steps += 1
            else:
                self.crossing_gap_steps = 0
            if self.crossing_gap_steps >= DWA_CROSSING_GAP_CONFIRM_STEPS:
                self._start_crossing_commit(pose)
                return self._crossing_commit_action(check_obstacles)
            self.current_v = 0.0
            self.current_w = 0.0
            return self._finalize_action(
                (0.0, 0.0, "DWA crossing gap confirmation"),
                check_obstacles,
            )

        return None



    def _escape_from_dynamic(self, obstacles, goal):
        """Move toward a short term refuge when normal rollouts are blocked."""
        moving = [point for point in obstacles if len(point) == 4]
        if not moving:
            return None
        fixed = [point for point in obstacles if len(point) == 2]
        duration_steps = max(1, round(0.8 / DWA_SIMULATION_STEP))
        best = None
        for velocity in (-DWA_MAX_REVERSE_SPEED, -0.10, 0.0, 0.10, 0.20):
            for yaw_rate in (-DWA_MAX_ANGULAR_SPEED, -0.9, 0.0,
                             0.9, DWA_MAX_ANGULAR_SPEED):
                actual_v, actual_w, left, right = self._achievable_velocity(
                    velocity, yaw_rate
                )
                trajectory = self._simulate(actual_v, actual_w)[:duration_steps]
                if self._trajectory_clearance(
                    trajectory[:max(1, round(1.2 / DWA_SIMULATION_STEP))], fixed
                ) <= (
                    DWA_ROBOT_RADIUS + DWA_STATIC_CLEARANCE_MARGIN
                ):
                    continue
                closest = self._dynamic_reachable_clearance(trajectory, moving)
                initial_gap = min(math.hypot(p[0], p[1]) for p in moving)
                if initial_gap < .6:
                    observed_gap = min(
                        math.hypot(p[0] - x, p[1] - y)
                        for x, y, _ in trajectory for p in moving)
                    if observed_gap < initial_gap - .01:
                        continue
                # A fallback must pass collision checks too. Previously it
                # returned the least-bad candidate even when all would collide.
                if closest <= DWA_ROBOT_RADIUS + DWA_EMERGENCY_CLEARANCE_MARGIN:
                    continue
                end_x, end_y, _ = trajectory[-1]
                end_distance = min(
                    math.hypot(ox + vx * 0.8 - end_x, oy + vy * 0.8 - end_y)
                    for ox, oy, vx, vy in moving
                )
                goal_direction = math.atan2(goal[1], goal[0])
                progress = end_x * math.cos(goal_direction) + end_y * math.sin(goal_direction)
                score = 3.0 * closest + 0.25 * end_distance + 0.8 * progress
                if best is None or score > best[0]:
                    best = (score, actual_v, actual_w, left, right)
        if best is None:
            return None
        _, self.current_v, self.current_w, left, right = best
        return left, right, "DWA 센서 기반 긴급 회피"

    def _rollout_best_candidate(
        self, goal, obstacles, fixed_obstacles, moving_obstacles, sensor_mode
    ):
        """Sample the dynamic window and return the best safe (score, v, w,
        clearance, left, right) candidate, or None if none clears the safety
        margins. Also updates the debug_* introspection fields.

        Pulled out of choose_action() so recovery/wall-follow can re-run
        this evaluation on demand (see the recovery-escape check in
        choose_action) instead of only ever running it when no recovery
        phase is active -- recovery_phase used to make choose_action return
        early without ever reconsidering whether a normal, safe forward
        candidate already exists.
        """
        speed_limit = self.side_speed_limit
        control_dt = TIME_STEP / 1000.0
        v_low = max(
            0.0 if not sensor_mode else -DWA_MAX_REVERSE_SPEED,
            self.current_v - DWA_LINEAR_ACCELERATION * control_dt,
        )
        v_high = min(
            speed_limit,
            self.current_v + DWA_LINEAR_ACCELERATION * control_dt,
        )
        if v_low > v_high:
            v_high = v_low
        w_low = max(
            -DWA_MAX_ANGULAR_SPEED,
            self.current_w - DWA_ANGULAR_ACCELERATION * control_dt,
        )
        w_high = min(
            DWA_MAX_ANGULAR_SPEED,
            self.current_w + DWA_ANGULAR_ACCELERATION * control_dt,
        )
        self.debug_v_window = (v_low, v_high)
        self.debug_w_window = (w_low, w_high)
        velocities = _sample_range(v_low, v_high, DWA_VELOCITY_SAMPLES)
        yaw_rates = _sample_range(w_low, w_high, DWA_YAW_RATE_SAMPLES)
        if v_low <= 0.0 <= v_high:
            velocities.append(0.0)
        if w_low <= 0.0 <= w_high:
            yaw_rates.append(0.0)

        if sensor_mode:
            key = (tuple(velocities), tuple(yaw_rates))
            if self._rollout_cache is None or self._rollout_cache[0] != key:
                records = []
                for v in velocities:
                    for w in yaw_rates:
                        av, aw, left, right = self._achievable_velocity(v, w)
                        records.append((av, aw, left, right, self._simulate(av, aw)))
                self._rollout_cache = (key, records)
            records = self._rollout_cache[1]
            static_values, dynamic_values = self._sensor_clearances(
                [record[4] for record in records], fixed_obstacles, moving_obstacles)
            candidate_index = 0

        best = None
        safe_candidate_count = 0
        debug_total = 0
        debug_static_ok = 0
        debug_dynamic_ok = 0
        debug_braking_ok = 0
        best_static_clearance = None
        best_dynamic_clearance = None
        best_score = None
        valid_forward_count = 0
        best_candidate_debug = None
        best_forward_debug = None
        best_positive = None
        for v in velocities:
            for w in yaw_rates:
                debug_total += 1
                if sensor_mode:
                    actual_v, actual_w, left, right, trajectory = records[candidate_index]
                else:
                    actual_v, actual_w = v, w
                    left, right = self._wheel_speeds(v, w)
                    trajectory = self._simulate(actual_v, actual_w)
                if sensor_mode:
                    static_clearance = float(static_values[candidate_index])
                    dynamic_clearance = float(dynamic_values[candidate_index])
                    candidate_index += 1
                    clearance = min(static_clearance, dynamic_clearance)
                else:
                    static_clearance = dynamic_clearance = clearance = (
                        self._trajectory_clearance(trajectory, obstacles)
                    )
                static_safe = static_clearance > DWA_ROBOT_RADIUS + DWA_STATIC_CLEARANCE_MARGIN
                dynamic_safe = dynamic_clearance > DWA_ROBOT_RADIUS + DWA_DYNAMIC_CLEARANCE_MARGIN
                if static_safe:
                    debug_static_ok += 1
                if static_safe and dynamic_safe:
                    debug_dynamic_ok += 1
                if not (static_safe and dynamic_safe):
                    continue

                free_distance = clearance - DWA_ROBOT_RADIUS
                stopping_distance = v * v / (2.0 * DWA_LINEAR_ACCELERATION)
                if stopping_distance > free_distance:
                    continue

                debug_braking_ok += 1
                safe_candidate_count += 1
                if actual_v > 0.02:
                    valid_forward_count += 1
                if not sensor_mode:
                    score = self._score(actual_v, trajectory, clearance, goal)
                    score_terms = None
                else:
                    score_terms = self._sensor_score_terms(
                        actual_v,
                        actual_w,
                        trajectory,
                        clearance,
                        goal,
                        prefer_forward=self._front_corner_escape_active,
                    )
                    score = score_terms["total"]
                if self.current_w != 0.0 and w * self.current_w > 0.0:
                    score += 0.001
                candidate_debug = {
                    "v": actual_v,
                    "w": actual_w,
                    "score": score,
                    "goal_distance": (
                        score_terms["goal_distance"] if score_terms else None
                    ),
                    "heading": score_terms["heading"] if score_terms else None,
                    "clearance": score_terms["clearance"] if score_terms else None,
                    "speed": score_terms["speed"] if score_terms else None,
                    "stop_penalty": (
                        score_terms["stop_penalty"] if score_terms else None
                    ),
                    "static_clearance": static_clearance,
                    "dynamic_clearance": dynamic_clearance,
                }
                if (
                    actual_v > 0.02
                    and (
                        best_forward_debug is None
                        or score > best_forward_debug["score"]
                    )
                ):
                    best_forward_debug = candidate_debug
                if (
                    actual_v > 0.5 * DWA_LINEAR_ACCELERATION * control_dt
                    and (best_positive is None or score > best_positive[0])
                ):
                    best_positive = (
                        score, actual_v, actual_w, clearance, left, right
                    )
                if best is None or score > best[0]:
                    best = (score, actual_v, actual_w, clearance, left, right)
                    best_static_clearance = static_clearance
                    best_dynamic_clearance = dynamic_clearance
                    best_score = score
                    best_candidate_debug = candidate_debug

        self.debug_safe_candidate_count = safe_candidate_count
        self.debug_total_candidates = debug_total
        self.debug_static_ok_count = debug_static_ok
        self.debug_dynamic_ok_count = debug_dynamic_ok
        self.debug_braking_ok_count = debug_braking_ok
        self.debug_best_static_clearance = best_static_clearance
        self.debug_best_dynamic_clearance = best_dynamic_clearance
        self.debug_best_score = best_score
        self.debug_best_vw = (best[1], best[2]) if best is not None else None
        self.debug_valid_forward_count = valid_forward_count
        self.debug_best_candidate = best_candidate_debug
        self.debug_best_forward_candidate = best_forward_debug
        self._best_positive_rollout = best_positive
        return best

    def choose_action(
        self, ranges, goal=(2.0, 0.0), pose=None, sim_time=None,
    ):
        """(왼쪽 바퀴속도, 오른쪽 바퀴속도, 상태 설명)을 반환한다.

        Priority: imminent dynamic threat/backoff, safe crossing yield, normal
        DWA rollout, then recovery when no collision-free rollout exists.
        """
        if (pose is None) != (sim_time is None):
            raise ValueError("pose and sim_time must be provided together")
        sensor_mode = pose is not None and sim_time is not None
        # wall_escape_active flips True/False from frame to frame right at
        # its own distance thresholds when the robot is pinned very close to
        # an obstacle: a brief False frame releases control (is_release_ready)
        # back to the path follower, whose turn immediately re-triggers
        # wall_escape_active, ping-ponging control every few frames. Track a
        # short cooldown instead of the instantaneous flag so the release
        # gate stays closed through that flicker.
        if self.wall_escape_active:
            self._wall_escape_cooldown = 0.5
        else:
            self._wall_escape_cooldown = max(
                0.0, self._wall_escape_cooldown - TIME_STEP / 1000.0
            )
        if not ranges:
            self.current_v = 0.0
            self.current_w = 0.0
            self.recovery_turn_sign = None
            self.recovery_phase = None
            self.wall_side = None
            self.wall_escape_active = False
            self._wall_escape_cooldown = 0.0
            self.wall_front_turn_sign = None
            self.backoff_steps = 0
            self.side_yield_elapsed = 0.0
            self.side_clear_steps = 0
            self.previous_scan_clusters = None
            self.previous_scan_time = None
            self.last_dynamic_obstacles = []
            self.last_obstacles = None
            self._front_corner_escape_active = False
            self._front_corner_escape_origin = None
            self._reset_crossing_state()
            return 0.0, 0.0, "LiDAR 없음 - 정지"

        obstacles = self._scan_with_dynamic_obstacles(ranges, pose, sim_time)
        self.last_obstacles = obstacles
        self._update_crossing_commit_progress(pose)
        fixed_obstacles = [obstacle for obstacle in obstacles if len(obstacle) == 2]
        moving_obstacles = [obstacle for obstacle in obstacles if len(obstacle) == 4]

        front_distance = self._front_distance(ranges)
        front_arc_distance = self._front_arc_distance(ranges)
        rear_distance = self._rear_distance(ranges)
        self.side_speed_limit = self._update_side_speed_limit(ranges)
        if sensor_mode:
            denominator = max(1, len(ranges) - 1)
            front_half_angle = (
                self.lidar_field_of_view * max(2, len(ranges) // 36) / denominator
            )
            front_arc_half_angle = (
                self.lidar_field_of_view * max(2, len(ranges) // 7) / denominator
            )
            static_front_distance = self._front_point_distance(
                fixed_obstacles, front_half_angle
            )
            static_front_arc_distance = self._front_point_distance(
                fixed_obstacles, front_arc_half_angle
            )
        else:
            static_front_distance = front_distance
            static_front_arc_distance = front_arc_distance
        if (
            self._front_corner_escape_active
            and self._front_corner_escape_origin is not None
            and static_front_arc_distance >= DWA_RECOVERY_CLEAR_DISTANCE
            and math.dist(pose[:2], self._front_corner_escape_origin)
            >= DWA_ROBOT_RADIUS
        ):
            self._front_corner_escape_active = False
            self._front_corner_escape_origin = None
        if sensor_mode:
            crossing_action = self._evaluate_crossing_hazard(
                moving_obstacles, rear_distance, obstacles, pose
            )
            if crossing_action is not None:
                return crossing_action

        if (
            self.backoff_steps
            or (
                static_front_distance < DWA_BACKOFF_TRIGGER_DISTANCE
                and rear_distance > DWA_BACKOFF_REAR_DISTANCE
            )
        ):
            if self.backoff_steps == 0:
                if self.recovery_turn_sign is None:
                    self.recovery_turn_sign = self._select_turn_sign(ranges)
                if self.recovery_phase is None:
                    self.recovery_phase = "turn"
            max_steps = math.ceil(DWA_BACKOFF_MAX_SECONDS * 1000.0 / TIME_STEP)
            front_is_clear = static_front_distance >= DWA_BACKOFF_CLEAR_DISTANCE
            if self._front_corner_escape_active:
                # Rotation alone can move a corner return out of the narrow
                # front sector.  During the scoped corner escape, require a
                # short translational retreat before declaring backoff done.
                minimum_corner_backoff_steps = math.ceil(0.5 * 1000.0 / TIME_STEP)
                front_is_clear = self.backoff_steps >= minimum_corner_backoff_steps
            if (
                rear_distance <= DWA_BACKOFF_REAR_DISTANCE
                or front_is_clear
                or self.backoff_steps >= max_steps
            ):
                self.backoff_steps = 0
                if self.recovery_phase is None:
                    self.recovery_phase = "turn"
                    self.recovery_turn_sign = self._select_turn_sign(ranges)
                self.current_v = 0.0
                self.current_w = 0.0
                return 0.0, 0.0, "DWA 후진 복구 완료"
            if self._front_corner_escape_active:
                action = self._front_corner_backoff_action(obstacles)
                if action is None:
                    return 0.0, 0.0, "DWA front-corner escape blocked - stop"
                self.backoff_steps += 1
                return action
            self.backoff_steps += 1
            self.current_v = -DWA_BACKOFF_SPEED
            self.current_w = self.recovery_turn_sign * DWA_WALL_FOLLOW_TURN_RATE
            left, right = self._wheel_speeds(self.current_v, self.current_w)
            return self._finalize_action(
                (left, right, "DWA 안전 후진 회피"), obstacles
            )
        if self.recovery_phase is not None:
            # recovery_phase being active used to make this branch return
            # unconditionally, without ever reconsidering whether a normal,
            # already-safe forward rollout candidate exists. That is a
            # structural deadlock for wall_escape_active in particular: its
            # command is a pure in-place turn (v=0), which cannot change the
            # side clearance that would let it self-release when the robot
            # is pinned between two nearby obstacles -- even though dozens
            # of safe forward candidates exist every single frame. Re-run
            # the normal rollout here and abandon the scripted recovery
            # state machine once a safe, forward-moving (v > 0) candidate
            # has been available for DWA_RECOVERY_ESCAPE_CONFIRM_STEPS
            # consecutive frames (not on one lucky frame, for the same
            # ping-pong reason as is_release_ready()'s hysteresis).
            escape_best = self._rollout_best_candidate(
                goal, obstacles, fixed_obstacles, moving_obstacles, sensor_mode
            )
            recovery_zero_forward_trap = (
                sensor_mode
                and not self._front_corner_escape_active
                and escape_best is not None
                and escape_best[1] <= 0.02
                and self.debug_valid_forward_count == 0
                and static_front_arc_distance < DWA_RECOVERY_CLEAR_DISTANCE
            )
            if recovery_zero_forward_trap:
                return self._start_front_corner_escape(
                    ranges, obstacles, pose
                )
            # A pure-turn recovery writes current_v=0 every frame, so its
            # next legal dynamic window tops out at 0.0192 m/s.  Requiring
            # v>0.02 here made recovery mathematically unable to select the
            # first acceleration step.  Bootstrap with the best safe positive
            # rollout; after it is applied, the normal window can expand past
            # 0.02 on the following frame.
            escape_candidate = (
                escape_best
                if escape_best is not None and escape_best[1] > 0.02
                else self._best_positive_rollout
            )
            if (
                self._front_corner_escape_active
                and self._best_positive_rollout is not None
            ):
                # The corner-escape latch already prevents an unsafe release
                # to path following.  Apply the first safe acceleration step
                # immediately: waiting for three identical frames while the
                # scripted pure turn keeps changing the scan makes that
                # candidate disappear again before it can ever be used.
                self.recovery_phase = None
                self.recovery_turn_sign = None
                self.wall_side = None
                self.wall_clear_steps = 0
                self.wall_escape_active = False
                self._wall_escape_cooldown = 0.0
                self.wall_front_turn_sign = None
                self._recovery_escape_streak = 0
                _, actual_v, actual_w, clearance, left, right = (
                    self._best_positive_rollout
                )
                self.current_v = actual_v
                self.current_w = actual_w
                if abs(actual_w) > 0.05:
                    self.last_turn_sign = 1.0 if actual_w > 0.0 else -1.0
                clearance_text = (
                    "∞" if not math.isfinite(clearance) else f"{clearance:.2f} m"
                )
                return self._finalize_action(
                    (
                        left,
                        right,
                        "DWA front-corner escape "
                        f"v={actual_v:.2f} m/s, ω={actual_w:.2f} rad/s, "
                        f"clearance={clearance_text}",
                    ),
                    obstacles,
                )
            if escape_candidate is not None:
                self._recovery_escape_streak += 1
            else:
                self._recovery_escape_streak = 0
            if self._recovery_escape_streak >= DWA_RECOVERY_ESCAPE_CONFIRM_STEPS:
                self.recovery_phase = None
                self.recovery_turn_sign = None
                self.wall_side = None
                self.wall_clear_steps = 0
                self.wall_escape_active = False
                self._wall_escape_cooldown = 0.0
                self.wall_front_turn_sign = None
                self._recovery_escape_streak = 0
                _, actual_v, actual_w, clearance, left, right = escape_candidate
                self.current_v = actual_v
                self.current_w = actual_w
                if abs(actual_w) > 0.05:
                    self.last_turn_sign = 1.0 if actual_w > 0.0 else -1.0
                clearance_text = "∞" if not math.isfinite(clearance) else f"{clearance:.2f} m"
                return self._finalize_action(
                    (
                        left,
                        right,
                        "DWA 복구 이탈 - 정상 경로 확보 "
                        f"v={actual_v:.2f} m/s, ω={actual_w:.2f} rad/s, "
                        f"여유={clearance_text}",
                    ),
                    obstacles,
                )
            recovery_action = self._recovery_action(
                ranges, static_front_distance, static_front_arc_distance
            )
            if recovery_action is not None:
                return self._validate_recovery_action(
                    recovery_action, obstacles
                )
        elif (
            not self._front_corner_escape_active
            and static_front_distance < DWA_RECOVERY_TRIGGER_DISTANCE
        ):
            recovery_action = self._start_recovery(ranges)
            return self._validate_recovery_action(
                recovery_action, obstacles
            )

        best = self._rollout_best_candidate(
            goal, obstacles, fixed_obstacles, moving_obstacles, sensor_mode
        )

        # A front-corner obstacle can sit outside the narrow front sector but
        # inside the swept front arc.  In that geometry the rollout commonly
        # reaches the 0.22 m static boundary with many safe rotate/near-zero
        # candidates but no candidate capable of v > 0.02 m/s.  Treating that
        # as a successful normal rollout leaves current_v inside the one-step
        # acceleration window around zero forever.  Re-enter the existing
        # (validated) recovery turn; this changes neither the footprint nor
        # either clearance margin and is limited to a static front-arc block.
        front_corner_local_minimum = (
            sensor_mode
            and best is not None
            and best[1] <= 0.02
            and self.debug_valid_forward_count == 0
            and static_front_arc_distance < DWA_RECOVERY_CLEAR_DISTANCE
        )
        if front_corner_local_minimum:
            # Keep avoidance ownership until the corner has actually left the
            # front arc.  Otherwise the first 0.0192 m/s bootstrap candidate
            # can be followed immediately by a nominal in-place turn, putting
            # the robot back on the same boundary without position progress.
            return self._start_front_corner_escape(ranges, obstacles, pose)

        if best is None:
            # No candidate cleared the safety margins at all: whatever
            # fallback below fires (escape/backoff/emergency turn), the
            # robot has not actually cleared the obstacle. Gate release in
            # is_release_ready() on this too, or a nominal command that
            # coincidentally looks safe for one frame mid-fallback ping-pongs
            # control exactly like the recovery_phase=="turn" case.
            self._last_rollout_had_no_safe_candidate = True
        else:
            self._last_rollout_had_no_safe_candidate = False

        if best is None and sensor_mode:
            escape = self._escape_from_dynamic(obstacles, goal)
            if escape is not None:
                return self._finalize_action(escape, obstacles)
            rear_threat = any(
                len(p) == 4 and p[0] < 0.10 and abs(p[1]) < 0.35 for p in obstacles
            )
            if rear_distance > DWA_BACKOFF_REAR_DISTANCE and not rear_threat:
                self.current_v = -DWA_BACKOFF_SPEED
                self.current_w = 0.0
                left, right = self._wheel_speeds(self.current_v, self.current_w)
                return self._finalize_action(
                    (left, right, "DWA 후방 안전 후진 회피"), obstacles
                )
            recovery_turn = self._emergency_turn(ranges)
            validated = self._validate_recovery_action(recovery_turn, obstacles)
            if validated is not None:
                return validated
            self.current_v = self.current_w = 0.0
            return 0.0, 0.0, "DWA 안전 경로 없음"
        if best is None:
            return self._validate_recovery_action(
                self._emergency_turn(ranges), obstacles
            )

        _, self.current_v, self.current_w, clearance, left, right = best
        if abs(self.current_w) > 0.05:
            self.last_turn_sign = 1.0 if self.current_w > 0.0 else -1.0
        clearance_text = "∞" if not math.isfinite(clearance) else f"{clearance:.2f} m"
        return self._finalize_action(
            (
                left,
                right,
                f"DWA v={self.current_v:.2f} m/s, ω={self.current_w:.2f} rad/s, 여유={clearance_text}",
            ),
            obstacles,
        )


# 이전 import를 사용하는 코드와의 호환성을 유지한다.
SimpleObstacleAvoidance = DynamicWindowAvoidance
