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

# Odometry-only encoder -> metre conversion.
#
# WHEEL_RADIUS above is the *nominal* wheel geometry and is what the wheel-speed
# commands must keep using (v = WHEEL_RADIUS * omega), so DWA behaviour is
# unchanged.  The radius that converts *encoder* revolutions into travelled
# distance is the wheel's effective ROLLING radius, which is not identical:
#
#   Measured (Extended, EXTENDED_ODOMETRY_DIAG=1, encoder pose vs. Supervisor
#   truth pose, 208 steady-state straight steps at 0.24 m/s with no
#   acceleration and no rotation): truth distance / encoder distance =
#   1.003012, i.e. an effective rolling radius of 0.040120 m.  The deficit is
#   systematic, not noise: it is present in every steady straight segment and
#   accumulated 7.4 cm of the ~7.9 cm odometry error over the 16.6 m up to
#   Extended waypoint 6.
#
#   Cause: the wheel's boundingObject Cylinder (radius 0.04, subdivision 32)
#   collides as a tessellated 32-gon prism whose faces are tangent to the
#   nominal radius, so one wheel revolution advances the robot by the polygon
#   perimeter rather than 2*pi*0.04.  Effective radius =
#   n*sin(pi/n)/(pi*cos(pi/n)) * 0.04 = 0.0401259 m for n = 32, which agrees
#   with the measurement to 1.5e-5 m.
#
# EVENT DAY: re-measure this on the event robot (drive a known straight
# distance, divide by the summed encoder radians * 2) instead of reusing the
# simulation value; setting it equal to WHEEL_RADIUS restores the old
# behaviour exactly.
ODOMETRY_WHEEL_RADIUS = 0.040120

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

# ---------------------------------------------------------------------------
# ACTIVE (avoidance_v2.py): DWA-primary static avoidance with a pose-progress
# watchdog and one explicit recovery state machine.
#
# The safety model is deliberately NOT redefined here: robot radius, static
# and dynamic clearance margins, acceleration limits, rollout horizon and
# sample counts are the DWA_* values above, shared with avoidance.py.
# ---------------------------------------------------------------------------

# DWA scoring (NORMAL_DWA).  Progress is the decrease of the obstacle-aware
# cost-to-go along a rollout, normalised by DWA_MAX_LINEAR_SPEED*DWA_HORIZON.
# Cruise cap for NORMAL_DWA: the legacy DWA limit.  Measured trade-off in
# Webots: encoder-odometry error grows with speed (straight 4.5 m corridor:
# 0.6 cm at 0.16 m/s vs 1.3 cm at 0.24 m/s; practice course max 2.1 vs 4.2 cm),
# but the Extended benchmark's 71 m waypoint polyline alone needs 444 s at
# 0.16 m/s against its 420 s limit, so the lower cap cannot complete it.
STATIC_DWA_MAX_SPEED = DWA_MAX_LINEAR_SPEED
# While a tracked mover within DWA_CROSSING_DETECTION_DISTANCE approaches the
# robot, cruise at the legacy nominal path speed: the crossing supervisor's
# yield/commit distances were tuned for that approach speed (at 0.24 m/s the
# Extended dynamic-A actor reached the robot's lane before any yield).
DYNAMIC_CAUTION_SPEED = PRACTICE_PATH_SPEED
DYNAMIC_CAUTION_MIN_CLOSING_SPEED = 0.05  # m/s towards the robot
# Braking (inevitable collision) check: a DWA candidate executed for one step,
# then braked at the DWA acceleration limits and held stationary for this long,
# must keep the dynamic margin from every approaching mover of at least this
# speed (constant-velocity prediction).
DYNAMIC_ANTICIPATION_HOLD_SECONDS = 2.5
DYNAMIC_ANTICIPATION_MIN_SPEED = 0.10
STATIC_DWA_PROGRESS_WEIGHT = 4.0
STATIC_DWA_HEADING_WEIGHT = 0.4
STATIC_DWA_CLEARANCE_WEIGHT = 0.6
STATIC_DWA_SPEED_WEIGHT = 0.4
# Tiny preference for keeping the current turn sign; breaks exact left/right
# score ties caused by LiDAR noise without overriding a real score difference.
STATIC_DWA_TURN_HYSTERESIS_WEIGHT = 0.05
STATIC_DWA_CLEARANCE_SCALE = 0.8  # (clearance - radius) / scale, clipped to [0, 1]
# The local goal is a waypoint to pass through, not a pose to stop at: the
# path layer switches to the next waypoint within 0.14 m.  A rollout that
# passes within this radius of a (non-projected) waypoint before its tail is
# cut scores as having reached it, so DWA no longer brakes on every approach.
# Smaller than the 0.14 m switch distance so a passing rollout really switches.
STATIC_DWA_WAYPOINT_PASS_RADIUS = 0.10
# Execution buffer for NORMAL_DWA only (stricter than the shared veto, never
# looser).  From a clearance above static safety distance + buffer a candidate
# must stay above it; inside the buffer it may not lower the robot's clearance
# by more than the tolerance (driving along a wall is fine, towards it is not).
# Without it DWA hugs the 0.22 m boundary; LiDAR noise / tracking error then
# puts the robot a millimetre inside the margin, where the legacy "must move
# away" rule admits nothing next to a wall (forward and reverse both approach
# a wall point, in-place rotation is forbidden) -- Extended truth run 10,
# waypoint 8, 15 s frozen.
STATIC_DWA_EXECUTION_BUFFER = 0.01
STATIC_DWA_BUFFER_TOLERANCE = 0.002  # ~LiDAR range noise

# Local cost-to-go field (scoring aid only; never used as a safety check).
# Built every frame from the current scan's static points in a world-aligned
# grid around the robot; unknown space is treated as free.
NAV_FIELD_RESOLUTION = 0.10
NAV_FIELD_HALF_EXTENT = 2.5
NAV_FIELD_INFLATION = DWA_ROBOT_RADIUS + DWA_STATIC_CLEARANCE_MARGIN
NAV_FIELD_INFLATED_COST = 100.0  # traversal multiplier inside the inflation (last resort)
NAV_FIELD_ANGLE_BINS = 1440     # shadow-casting resolution seen from the goal
# Shadowed cells are only resolved inside this radius (rollouts and recovery
# primitives stay within ~0.6 m of the robot).
NAV_FIELD_LOCAL_RADIUS = 1.0
NAV_FIELD_DETOUR_EPSILON = 0.05  # geodesic - euclidean above this => obstacle-shaped route
# A goal inside the static safety distance is replaced (for the field only)
# by the nearest admissible point within this radius.
NAV_FIELD_GOAL_SEARCH_RADIUS = 0.60
NAV_FIELD_GOAL_EPSILON = 0.003  # projected goal clearance beyond NAV_FIELD_INFLATION
# Within this distance of the goal the field returns the exact Euclidean
# distance/direction (interpolation smooths the cone apex).
NAV_FIELD_NEAR_GOAL = 0.15

# Progress watchdog.  STUCK = no pose progress for the window while the
# waypoint is outside tolerance.  "Progress" is any of: goal-distance
# improvement, translation, or a consistent heading change, measured against
# the pose at the last progress event.  The long window additionally catches
# small in-place oscillations that keep re-arming the short criterion.
WATCHDOG_WINDOW_SECONDS = 1.5
WATCHDOG_MIN_GOAL_IMPROVEMENT = 0.03
WATCHDOG_MIN_DISPLACEMENT = 0.04
WATCHDOG_MIN_HEADING_CHANGE = 0.30
WATCHDOG_GOAL_TOLERANCE = 0.14
# When the waypoint lies inside the static safety distance the watchdog
# tracks the nearest admissible point instead, which must really be reached.
WATCHDOG_PROJECTED_GOAL_TOLERANCE = 0.03
WATCHDOG_LONG_WINDOW_SECONDS = 6.0
WATCHDOG_LONG_MIN_GOAL_IMPROVEMENT = 0.10
WATCHDOG_LONG_MIN_DISPLACEMENT = 0.25
WATCHDOG_GRACE_SECONDS = 1.0
WATCHDOG_WAYPOINT_JUMP = 0.30  # goal jump treated as a new waypoint when no id is given

# Dynamic supervisor (avoidance_v2 adapter around the legacy crossing logic).
# A crossing commit whose command is vetoed by STATIC geometry alone for this
# long is cancelled: static obstacles never clear, so waiting is a deadlock
# (seen when a static edge was briefly tracked as a mover).  A commit vetoed
# because of a moving obstacle keeps waiting exactly as in the legacy logic.
DYNAMIC_COMMIT_STATIC_BLOCK_SECONDS = 1.0
# Static-persistence check on the legacy tracker's output.  The tracker
# estimates velocity from cluster centroids; when a grazing wall end gains and
# loses a few beams the centroid jumps by several cm and the cluster is tracked
# as a mover indefinitely (Extended U-turn: 30 s crossing yield in front of a
# static wall).  A "moving" cluster is handed to the static layer when at least
# FRACTION of its points lie in world cells that have stayed occupied
# (dropouts <= DROPOUT) for >= SECONDS.  A real mover's leading cells are
# always new: for a surface of length L moving at v, only L - v*SECONDS of it
# can be that old, so a cluster of span <= 0.55 m moving at >= 0.055 m/s is
# never demoted (the tracker needs >= DWA_DYNAMIC_MIN_SPEED to promote at all).
DYNAMIC_STATIC_PERSISTENCE_SECONDS = 2.0
DYNAMIC_STATIC_PERSISTENCE_DROPOUT = 0.30
DYNAMIC_STATIC_PERSISTENCE_FRACTION = 0.8
DYNAMIC_STATIC_PERSISTENCE_RESOLUTION = 0.10

# LiDAR artifact filter (avoidance_v2 scan preprocessing).  The Webots 360
# degree Lidar renders through several virtual camera faces; at the face seams
# (+-45/+-135 deg) a beam next to a dead (inf) seam beam can report an edge
# point far too close (practice run: 0.16 m for a corner 0.27 m away).  A
# return is dropped only when it is closer than the nearest valid neighbour on
# BOTH sides by LIDAR_SPIKE_GAP and so close that a real object seen by just
# one beam would be thinner than LIDAR_SPIKE_MIN_OBJECT_WIDTH.
# TODO(event day): confirm the event LiDAR and that no obstacle is thinner.
LIDAR_SPIKE_GAP = 0.05
LIDAR_SPIKE_MIN_OBJECT_WIDTH = 0.02

# Recovery (RECOVERY_BACKOFF -> RECOVERY_COMMIT).  Completion is measured by
# odometry (distance / heading actually achieved), never by frame counts.
RECOVERY_BACKOFF_SPEED = 0.10
RECOVERY_BACKOFF_DISTANCE = 0.12
RECOVERY_BACKOFF_TIMEOUT = 2.5
RECOVERY_COMMIT_YAW_RATE = 0.90
RECOVERY_COMMIT_ANGLE = 0.80  # rad, minimum committed heading change
RECOVERY_COMMIT_FORWARD_SPEED = 0.10
RECOVERY_COMMIT_FORWARD_DISTANCE = 0.15  # minimum committed translation after the turn
RECOVERY_COMMIT_TIMEOUT = 5.0
RECOVERY_BLOCKED_TIMEOUT = 1.0  # a vetoed committed turn waits at most this long
# Repeated STUCK near the same place (same waypoint) escalates the committed
# backoff/turn/forward amounts instead of changing strategy.
RECOVERY_SITE_RADIUS = 0.35
RECOVERY_ESCALATION = 0.5
RECOVERY_MAX_ESCALATION_STEPS = 2
RECOVERY_DIRECTION_PROGRESS_TIE = 0.05
RECOVERY_DIRECTION_CLEARANCE_TIE = 0.05
