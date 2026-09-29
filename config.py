"""TECH WEEK 로봇에서 자주 바꿀 설정값을 모아 둔 파일입니다."""

# Webots world의 WorldInfo.basicTimeStep과 같은 값이어야 합니다. (밀리초)
TIME_STEP = 32

# Occupancy Grid 공통 상태값입니다.
UNKNOWN = -1
FREE = 0
OCCUPIED = 100

# ACTIVE: wheel motor hard limit, in rad/s.
MAX_SPEED = 6.0

# LEGACY (no active consumers): old rule/timed controller speed presets.
NORMAL_SPEED = 6.0
SLOW_SPEED = 3
TURN_SPEED = 4.0

# LEGACY (no active consumers): superseded rule-based avoidance settings.
# They remain documented for now so an event-day fallback can be restored
# without guessing its old tuning.  The active controller below is DWA-based.
SAFE_DISTANCE = 1
STOP_DISTANCE = 1

# 회전 중에는 전방이 이 거리보다 넓어져야 다시 직진합니다.
# SAFE_DISTANCE보다 크게 두면 직진/회전이 빠르게 반복되는 현상을 막습니다.
TURN_CLEAR_DISTANCE = 1.1

# 회전을 시작하면 센서 값이 잠깐 바뀌어도 이 시간 동안 방향을 유지합니다.
MIN_TURN_SECONDS = 0.8

# 전방 값이 계속 막혀 있어도 무한히 제자리 회전하지 않도록 제한합니다.
MAX_TURN_SECONDS = 1.5

# 회전 직후 장애물 옆에서 빠져나오기 위한 짧은 직진 구간입니다.
ESCAPE_FORWARD_SECONDS = 2.0
ESCAPE_SPEED = 1.5

# 이 거리보다 가까운 물체가 정면에 있으면 탈출 직진을 취소합니다.
EMERGENCY_DISTANCE = 0.5

# 좌우 거리 차이가 이 값보다 작으면 사실상 같은 공간으로 판단합니다.
TURN_DIRECTION_MARGIN = 0.10

# UNUSED (no active consumers): old STEP 2 timed-demo settings.
DEMO_FORWARD_SECONDS = 3.0
DEMO_STOP_SECONDS = 1.0
DEMO_TURN_SECONDS = 1.6  # practice 로봇에서 약 90도
DEMO_SECOND_FORWARD_SECONDS = 2.0

# 실제 대회 장치 이름이 공개되면 이 목록의 앞부분만 수정하면 됩니다.
# 알고리즘 파일에서는 Webots 장치 이름을 직접 사용하지 않습니다.
DEVICE_NAME_CANDIDATES = {
    "left_motor": ["left wheel motor", "left_motor", "motor_left"],
    "right_motor": ["right wheel motor", "right_motor", "motor_right"],
    "rear_left_motor": ["rear left wheel motor", "rear_left_motor", "motor_rear_left"],
    "rear_right_motor": ["rear right wheel motor", "rear_right_motor", "motor_rear_right"],
    "lidar": ["lidar", "laser", "laser scanner"],
    "camera": ["camera", "front camera", "front_camera"],
}

# practice.wbt에서 확인한 LiDAR 배열은 좌우 방향이 제어 좌표계와 반대입니다.
LIDAR_REVERSED = True

# 4륜 스키드 스티어 기구 치수입니다.
WHEEL_RADIUS = 0.04
WHEEL_TRACK = 0.23

# ACTIVE: local Dynamic Window Approach settings.
LIDAR_FIELD_OF_VIEW = 2.0 * 3.141592653589793
DWA_MIN_RANGE = 0.055
DWA_ROBOT_RADIUS = 0.17
DWA_STATIC_CLEARANCE_MARGIN = 0.05  # sparse LiDAR returns and body corners
DWA_MAX_LINEAR_SPEED = WHEEL_RADIUS * MAX_SPEED
DWA_MAX_ANGULAR_SPEED = 1.8
DWA_LINEAR_ACCELERATION = 0.60
DWA_ANGULAR_ACCELERATION = 4.0
DWA_HORIZON = 2.5
DWA_MAX_REVERSE_SPEED = 0.18
DWA_SIMULATION_STEP = 0.10
DWA_VELOCITY_SAMPLES = 6
DWA_YAW_RATE_SAMPLES = 15
DWA_HEADING_WEIGHT = 1.2
DWA_CLEARANCE_WEIGHT = 1.8
DWA_SPEED_WEIGHT = 0.8
DWA_PROGRESS_WEIGHT = 1.0
# Sensor-mode scoring only (never admissibility): rollouts whose full-horizon
# static clearance falls below this distance are ranked lower, reaching
# -DWA_STATIC_HEADROOM_WEIGHT at the static collision margin.  The distance
# matches the recovery trigger so DWA keeps out of the zone that would start a
# recovery turn instead of driving into it and handing over to recovery.
DWA_STATIC_HEADROOM_DISTANCE = 0.50
DWA_STATIC_HEADROOM_WEIGHT = 1.0
# Sensor-mode scoring: a candidate that does not translate (turning in place)
# while the local goal is within this bearing is idling and is ranked lower by
# DWA_IDLE_PENALTY.  Rotation remains free when the goal needs a real turn.
# Sensor-mode scoring: while a recovery episode is latched (handed back to DWA
# with the obstacle still in the front arc) rollouts turning toward the
# episode's chosen side gain up to this much, so a symmetric blocker does not
# leave DWA undecided.  Ranking only; admissibility is unchanged.
DWA_EPISODE_SIDE_WEIGHT = 0.3
DWA_IDLE_HEADING_TOLERANCE = 0.35
DWA_IDLE_PENALTY = 0.5
# Below this local-goal distance the planner stops asking for progress.  Keep
# it smaller than any waypoint acceptance radius used by the integration.
DWA_GOAL_REACHED_DISTANCE = 0.05
DWA_RECOVERY_TRIGGER_DISTANCE = 0.50
DWA_RECOVERY_CLEAR_DISTANCE = 0.75
# Path-follow release hysteresis: consecutive is_command_safe() frames
# required before ceding control back to the nominal path follower once an
# avoidance episode is active. Without this, a single-frame safety flicker
# right at an obstacle corner (the nominal turn is briefly judged safe, then
# unsafe again one step later once it has actually turned) ping-pongs control
# between path-follow and avoidance every frame, oscillating in place instead
# of clearing the corner.
DWA_PATH_RELEASE_CONFIRM_STEPS = 3
# Recovery-escape re-evaluation: consecutive frames a normal DWA rollout
# candidate must come back safe AND forward-moving (v > 0) before an active
# recovery/wall-follow episode is abandoned in favour of it. Without this,
# recovery_phase being non-None made choose_action() return early forever,
# never reconsidering whether a normal, already-safe forward path exists --
# a robot pinned in a corner with wall_escape_active's pure in-place turn
# could deadlock indefinitely even though dozens of safe forward rollout
# candidates existed the whole time. Kept > 1 (not released on a single
# lucky frame) for the same reason as DWA_PATH_RELEASE_CONFIRM_STEPS.
DWA_RECOVERY_ESCAPE_CONFIRM_STEPS = 3
# Progress watchdog (pose based): no DWA_PROGRESS_MIN_IMPROVEMENT reduction of
# the local-goal distance for DWA_PROGRESS_TIMEOUT seconds (crossing waits
# excluded) starts a wall-follow detour around the local minimum.  The detour
# ends Bug2-style once the robot is DWA_DETOUR_LEAVE_MARGIN closer than where
# it started and the straight goal line has headroom, or after
# DWA_DETOUR_MAX_SECONDS, after which the next detour takes the other side.
DWA_PROGRESS_TIMEOUT = 8.0
DWA_PROGRESS_MIN_IMPROVEMENT = 0.10
DWA_DETOUR_MAX_SECONDS = 45.0
DWA_DETOUR_LEAVE_MARGIN = 0.10
DWA_RECOVERY_YAW_RATE = 1.0
DWA_TURN_SWITCH_MARGIN = 0.75
DWA_WALL_FOLLOW_DISTANCE = 0.48
DWA_WALL_FOLLOW_SPEED = 0.10
DWA_WALL_FOLLOW_TURN_RATE = 0.60
DWA_WALL_FOLLOW_GAIN = 2.0
DWA_WALL_LOST_DISTANCE = 0.90
DWA_WALL_CLEAR_STEPS = 20
DWA_WALL_EMERGENCY_DISTANCE = 0.28
DWA_WALL_RELEASE_DISTANCE = 0.42
DWA_BACKOFF_TRIGGER_DISTANCE = 0.28
DWA_BACKOFF_CLEAR_DISTANCE = 0.45
DWA_BACKOFF_REAR_DISTANCE = 0.40
DWA_BACKOFF_SPEED = 0.12
DWA_BACKOFF_MAX_SECONDS = 1.5
DWA_SIDE_CAUTION_DISTANCE = 1.0
DWA_SIDE_CAUTION_SPEED = 0.08
DWA_SIDE_STOP_DISTANCE = 0.85
DWA_SIDE_YIELD_SECONDS = 5.0
DWA_SIDE_CLEAR_STEPS = 20
DWA_DYNAMIC_MIN_SPEED = 0.16
DWA_DYNAMIC_MAX_SPEED = 1.5
DWA_DYNAMIC_MATCH_DISTANCE = 0.12
DWA_DYNAMIC_CLUSTER_GAP = 0.07
DWA_DYNAMIC_MIN_CLUSTER_POINTS = 3
DWA_DYNAMIC_TRACK_HOLD_SECONDS = 0.15  # bridge a short one/few-frame cluster dropout
# While the robot's own measured yaw rate exceeds this, suppress promoting a
# NEW cluster to a tracked mover (a track already confirmed as moving is
# still held). A static corner's *visible surface point* slides as the
# robot's viewing angle changes, which a fast in-place turn (recovery/
# wall-follow) can reconstruct as a plausible world-frame velocity even
# though the obstacle itself never moved -- this bounds that false positive
# without weakening real moving-obstacle detection during normal driving.
DWA_EGO_ROTATION_SUPPRESS_RATE = 0.45  # rad/s
DWA_DYNAMIC_CLEARANCE_MARGIN = 0.15  # LiDAR 표면 뒤의 사람 몸체와 추정 오차
DWA_EMERGENCY_CLEARANCE_MARGIN = 0.02  # retain physical clearance during retreat
DWA_DYNAMIC_ACCELERATION_BOUND = 0.05  # 관측 뒤 방향 전환 가능성의 상한 (m/s²)
DWA_SIDE_APPROACH_DISTANCE = 0.80  # 측면 접근 장애물 감지 거리
DWA_CROSSING_YIELD_DISTANCE = 0.55  # 횡단 장애물 안전 대기 거리
DWA_CROSSING_BACKOFF_DISTANCE = 0.42  # 횡단/측면 접근 시 후진 회피 트리거 거리
DWA_CROSSING_DETECTION_DISTANCE = 1.80  # moving cluster screening range
DWA_CROSSING_STATIONARY_SPEED = 0.06  # below this, treat a track as stationary
DWA_CROSSING_LATERAL_SPEED = 0.04  # minimum lateral speed for crossing prediction
DWA_CROSSING_LANE_HALF_WIDTH = 0.25  # robot path half-width for crossing checks
DWA_CROSSING_CLEARING_X_RANGE = (-0.25, 0.05)  # occupied crossing zone behind/front
DWA_CROSSING_CLEARING_DISTANCE = 0.60  # maximum range for clearing through a crossing
DWA_CROSSING_CLEARING_LATERAL_DISTANCE = 0.35  # lateral extent of a clearing threat
DWA_CROSSING_PREDICTION_SECONDS = 1.5  # short, re-observed crossing prediction horizon
DWA_CROSSING_IMMINENT_X_MARGIN = 0.45  # predicted crossing x limit for backoff
DWA_CROSSING_YIELD_X_RANGE = (0.20, 0.80)  # predicted crossing x interval to yield for
DWA_CROSSING_GAP_LATERAL_DISTANCE = 0.30  # obstacle centre must clear the lane
DWA_CROSSING_GAP_MIN_SECONDS = 0.35  # conservative time gap if motion reverses
DWA_CROSSING_GAP_CONFIRM_STEPS = 3  # reject one-frame gaps/track jitter
DWA_CROSSING_COMMIT_SPEED = 0.22  # faster than the 0.16 m/s nominal path speed
DWA_CROSSING_COMMIT_MIN_DISTANCE = 0.65  # carry the robot fully past the crossing
DWA_CROSSING_COMMIT_MAX_DISTANCE = 1.00  # bound a bad crossing-range estimate
DWA_CROSSING_COMMIT_CANCEL_SECONDS = 1.20  # matches the near-term dynamic safety horizon

# Practice vision target. Replace only this section after the event target
# specification is published; image-space detections are not world positions.
VISION_RED_LOW_1 = (0, 100, 80)
VISION_RED_HIGH_1 = (10, 255, 255)
VISION_RED_LOW_2 = (170, 100, 80)
VISION_RED_HIGH_2 = (179, 255, 255)
VISION_MIN_AREA_PIXELS = 180
VISION_MORPH_KERNEL_SIZE = 3
VISION_MIN_CIRCULARITY = 0.70
VISION_MAX_ASPECT_RATIO = 1.35

# Practice-only route follower. The event controller must replace its waypoint
# source with Planning/A* while retaining wheel-speed and pose conventions.
PRACTICE_PATH_SPEED = 0.16
PRACTICE_PATH_TURN_RATE = 1.20
PRACTICE_PATH_TURN_THRESHOLD = 0.35
PRACTICE_PATH_HEADING_GAIN = 1.80
