# TECH WEEK 해커톤 공통 에이전트 규칙

이 파일은 이 저장소에서 작업하는 AI 코딩 에이전트(Claude Code, Codex 등)가 공통으로 따라야 하는 규칙이다.

이 저장소의 목적은 부산대학교 TECH WEEK 해커톤의 **Autonomous Mobile Robot(AMR) - Search and Rescue** 미션을 Webots 시뮬레이션 환경에서 구현하는 것이다.

> [!IMPORTANT]
> - 이 문서는 **팀 내부 개발 규칙**이다.
> - 당일 운영진이 제공하는 Webots world, robot/device 구성, 센서 형식, target 정보, starter code가 이 문서와 충돌하면 **당일 제공 자료가 최우선**이다.
> - 확인되지 않은 Webots Device Name, 센서 사양, Target 특성, 평가 세부 가중치 등을 AI가 추측해서 사실처럼 코드에 고정하지 않는다.
> - 해커톤 당일 공개 정보가 필요한 부분은 인터페이스/TODO로 격리하고, 확인 후 연결한다.

---

## 1. 프로젝트 목표

최종 미션은 다음 흐름을 완성하는 것이다.

```text
START
  ↓
시작 Pose 저장
  ↓
Localization
  ↓
LiDAR 기반 Mapping
  ↓
Frontier 탐색
  ↓
다음 탐색 목표 선정
  ↓
A* Global Path 생성
  ↓
Path Following
  ↓
근거리 장애물 위험?
  ├─ YES → Obstacle Avoidance
  └─ NO  → Path Following 유지
  ↓
Camera Target Detection
  ↓
Target 발견?
  ├─ NO  → Exploration 계속
  └─ YES → Target으로 이동
              ↓
         Target 처리/도착
              ↓
       모든 Target 완료?
         ├─ NO → EXPLORE
         └─ YES
               ↓
          RETURN_HOME
               ↓
              DONE
```

프로젝트는 다음 성질을 만족하는 방향으로 개발한다.

- 사전 지도가 없는 환경에서 탐색한다.
- 현재 위치와 방향을 추정한다.
- LiDAR 등 센서 데이터를 이용해 지도를 생성한다.
- Target을 탐지하고 Target 위치로 이동한다.
- 정적 장애물과 움직이는 장애물에 충돌하지 않는다.
- 미션 완료 후 시작 지점으로 복귀한다.
- 실제 로봇이 아니라 Webots 시뮬레이션을 대상으로 한다.

---

## 2. 공식 정보와 우선순위

구현 판단의 우선순위는 다음과 같다.

### 1순위 — 해커톤 당일 운영진 제공 자료
예:

- Webots world (`.wbt`)
- Robot/PROTO
- 실제 Device Name
- Sensor 구성
- LiDAR 사양
- Odometry/Orientation 관련 센서 형식
- Target의 종류와 시각적 특징
- Starter Code
- 당일 추가 공지

### 2순위 — 사전 공식 안내 자료
현재 확인된 기준:

- Webots 시뮬레이션 사용
- Ubuntu 22.04 + Python 3.10 기준
- Windows/macOS 사용 가능
- C++ 사용 가능
- GPU 사용은 선택
- Differential Drive 로봇
- IMU 자체는 없으며 관련 기능을 구성하는 센서가 별도로 존재
- 지도/현재 위치/Target 위치는 사전 제공되지 않음
- 시작 Position & Orientation은 제공
- Mapping / Localization / Object Detection / Global Path Planning / Local Path Planning이 핵심 기술
- Global Planning 예시: Dijkstra, A*
- Local Planning 예시: Dynamic Window Approach 등
- Vision 문제는 복잡하게 다루지 않을 예정이며 Target 세부 정보는 당일 공개
- 전체 프로그램 틀은 제공되지 않으며 필요한 일부 기능 코드만 제공될 수 있음

### 3순위 — 이 저장소의 팀 규칙과 현재 구현
- `AGENTS.md`
- `CLAUDE.md`
- `README.md`
- 현재 `develop`의 코드와 테스트
- 팀이 합의한 인터페이스와 convention

### 충돌 처리
- 당일 제공 자료와 저장소 규칙이 충돌하면 당일 자료를 우선한다.
- 문서끼리 충돌하면 임의로 합치지 않는다.
- 충돌 위치, 영향 범위, 가능한 선택지를 보고하고 팀 결정을 기다린다.
- 확인되지 않은 내용을 "보통 Webots는 이렇다"는 이유로 고정 구현하지 않는다.

---

## 3. 기술 스택과 범위

기본 기술 스택:

```text
Simulator : Webots
Language  : Python 3.10
Numerical : NumPy
Vision    : OpenCV (필요한 경우)
```

원칙:

- 현재 저장소는 Python 기준으로 유지한다.
- 운영진이 C++ 사용을 허용하더라도 팀 합의 없이 Python/C++ 혼합 구조를 도입하지 않는다.
- Target Detection이 단순한 규칙 기반 CV로 해결 가능하면 불필요한 Deep Learning 모델을 추가하지 않는다.
- GPU는 필수가 아니므로 GPU 전용 구현을 기본 전제로 삼지 않는다.
- 새 라이브러리는 실제 필요성이 있을 때만 `requirements.txt`에 추가한다.
- 설치 비용이 큰 프레임워크를 해커톤 직전에 임의 도입하지 않는다.

---

## 4. 저장소 구조

현재 목표 구조:

```text
techweek_robot/
│
├── AGENTS.md
├── CLAUDE.md
├── README.md
├── requirements.txt
├── .gitignore
│
├── worlds/
│   └── .gitkeep
│
├── protos/
│   └── .gitkeep
│
├── controllers/
│   └── techweek_controller/
│       ├── techweek_controller.py
│       ├── config.py
│       ├── robot_io.py
│       ├── localization.py
│       ├── mapping.py
│       ├── astar.py
│       ├── frontier.py
│       ├── path_follower.py
│       ├── avoidance.py
│       ├── vision.py
│       └── state_machine.py
│
└── tests/
    ├── test_astar.py
    ├── test_frontier.py
    ├── test_mapping.py
    ├── test_path_follower.py
    ├── test_avoidance.py
    └── test_state_machine.py
```

### 구조 규칙

- `controllers/techweek_controller/techweek_controller.py`가 메인 Webots Controller Entry Point다.
- 별도 `main.py`를 새로 만들지 않는다.
- Webots API 직접 접근은 가능한 한 `robot_io.py`에 집중한다.
- 순수 알고리즘 모듈은 Webots에 직접 의존하지 않게 유지한다.
- 같은 책임을 가진 중복 파일을 만들지 않는다.
- 새 최상위 디렉터리나 새로운 계층을 추가하기 전에 기존 구조로 해결 가능한지 먼저 확인한다.
- 당일 받은 `.wbt`, `.proto` 파일은 실제 제공 구조를 확인한 뒤 `worlds/`, `protos/`에 배치한다.
- 운영진이 제공한 프로젝트 구조가 Webots 실행상 반드시 다른 배치를 요구하면, 기존 구조를 무리하게 고집하지 말고 변경 영향과 이유를 보고한 뒤 팀 합의로 수정한다.

---

## 5. 모듈 책임

### `techweek_controller.py`

전체 통합과 Webots main loop를 담당한다.

주요 책임:

```text
Robot I/O
→ Localization
→ Mapping
→ Frontier Selection
→ A*
→ Path Follower
→ Avoidance Selector
→ Motor Command
```

동시에:

```text
Camera
→ robot_io.py
→ Vision
→ State Machine
```

규칙:

- A*, Mapping, Vision 등 큰 알고리즘 구현을 이 파일에 직접 넣지 않는다.
- 이 파일은 orchestration/integration에 집중한다.
- 최종 Motor Command를 `robot_io.py`로 전달한다.

---

### `config.py`

공통 설정값을 관리한다.

예상 항목:

```text
TIME_STEP
MAX_SPEED
NORMAL_SPEED
TURN_SPEED
SAFE_DISTANCE
MAP_WIDTH
MAP_HEIGHT
MAP_RESOLUTION
ANGLE_THRESHOLD
GOAL_TOLERANCE
OBSTACLE_THRESHOLD
FRONTIER_MIN_DISTANCE
UNKNOWN
FREE
OCCUPIED
```

규칙:

- Webots/센서 실제 값이 확인되지 않은 항목은 임의로 "최종값"처럼 고정하지 않는다.
- 튜닝 값은 의미 있는 이름으로 관리한다.
- 동일한 숫자를 여러 모듈에 중복 하드코딩하지 않는다.

---

### `robot_io.py`

우리 코드와 Webots Device 사이의 경계다.

담당:

- Motor
- LiDAR
- Camera
- Odometry 관련 Sensor
- Orientation 관련 Sensor
- Device initialization
- Sensor enable
- Wheel speed command

예상 인터페이스:

```python
set_wheel_speed(left, right)
stop()

get_lidar()
get_camera_image()
get_odometry()
get_orientation()
```

규칙:

- 실제 Device Name을 확인하기 전 `left wheel motor` 같은 이름을 사실처럼 고정하지 않는다.
- Webots API 호출을 다른 알고리즘 파일에 흩뿌리지 않는다.
- 센서 raw data의 단위와 shape를 명시적으로 정리한다.
- `robot_io.py`는 데이터를 수집/정규화하고, 고수준 판단 알고리즘은 다른 모듈에 맡긴다.

---

### `localization.py`

현재 Robot Pose를 추정하고 관리한다.

Canonical Pose:

```text
(x, y, theta)
```

팀 convention:

- `x`, `y`: World Coordinate, meter
- `theta`: radian
- angle normalization 범위는 구현 시 하나로 통일하고 문서화한다.

예상 인터페이스:

```python
pose = localizer.update(...)
pose = localizer.get_pose()
```

주의:

- 사전 안내상 현재 위치는 직접 주어지지 않는다.
- 시작 Position & Orientation은 제공된다.
- Odometry, LiDAR, 기타 orientation 관련 센서를 실제로 어떤 방식으로 결합할지는 당일 센서 형식 확인 후 결정한다.
- Scan Matching을 구현할 경우 별도 근거와 테스트 없이 localization의 유일한 방법으로 고정하지 않는다.

---

### `mapping.py`

LiDAR와 현재 Pose를 이용해 Occupancy Grid Map을 생성/갱신한다.

팀 convention:

```text
UNKNOWN  = -1
FREE     = 0
OCCUPIED = 100
```

주요 기능:

```python
world_to_grid(x, y)
grid_to_world(row, col)

mark_free(...)
mark_obstacle(...)

update_from_lidar(...)
in_bounds(...)
```

기본 흐름:

```text
Pose (x, y, theta)
+
LiDAR (distance, angle)
→ LiDAR endpoint world coordinate
→ world_to_grid()
→ Ray가 지나간 영역 FREE
→ Endpoint OCCUPIED
```

규칙:

- 실제 LiDAR FOV, ray 수, min/max range가 확인되기 전 sensor-specific 상수를 추측하지 않는다.
- `inf`, `nan`, out-of-range 값을 명시적으로 처리한다.
- Grid 좌표계와 World 좌표계를 혼용하지 않는다.
- Mapping 알고리즘은 Webots Motor를 제어하지 않는다.

---

### `astar.py`

Occupancy Grid 기반 Global Path Planning을 담당한다.

Canonical Interface:

```python
path = astar(grid, start, goal)
```

입력:

```text
grid
start = (row, col)
goal  = (row, col)
```

출력:

```text
[(row, col), (row, col), ...]
```

규칙:

- Grid coordinate만 사용한다.
- Motor/Webots API에 접근하지 않는다.
- 경로가 없을 때 반환 형식을 명확히 정의한다.
- 장애물 cell을 통과하지 않는다.
- diagonal movement 허용 여부는 팀이 명시적으로 결정하기 전 임의로 바꾸지 않는다.
- heuristic을 변경하면 admissibility와 테스트 영향을 검토한다.

---

### `frontier.py`

아직 탐색하지 않은 영역으로 이동하기 위한 Frontier를 탐지하고 다음 탐색 Goal을 선정한다.

기본 Frontier 정의:

```text
FREE Cell
+
인접 UNKNOWN Cell 존재
```

주요 기능:

```python
find_frontiers(grid)
select_frontier(frontiers, current_position)
```

흐름:

```text
Occupancy Grid
→ Frontier 탐색
→ Frontier 후보 평가
→ 다음 Goal 선택
→ A*
```

역할 구분:

```text
Frontier = 어디로 갈 것인가
A*       = 그곳까지 어떤 Grid Path로 갈 것인가
```

규칙:

- Frontier selection scoring을 추가할 때 거리/크기/정보량 등 기준을 명시한다.
- Mapping의 UNKNOWN/FREE/OCCUPIED convention을 그대로 따른다.
- A*와 중복되는 path search logic을 새로 구현하지 않는다.

---

### `path_follower.py`

A*가 만든 Grid Path를 실제 Robot이 따라갈 수 있는 control command로 변환한다.

입력:

```text
현재 Pose: World (x, y, theta)
World waypoint path: [(x1, y1), (x2, y2), ...]
```

출력:

```text
(left_speed, right_speed)
```

기본 흐름:

```text
다음 Waypoint
→ dx, dy
→ target_angle = atan2(dy, dx)
→ angle_error
→ 회전 또는 전진
→ Waypoint 도달
→ 다음 Waypoint
```

규칙:

- A* 출력 `(row, col)`을 그대로 World Pose와 비교하지 않는다.
- 반드시 `grid_to_world()` 등의 명시적 변환을 거친다.
- Motor를 직접 호출하지 않는다.
- `compute_control()`은 wheel speed command만 반환한다.
- angle과 distance tolerance를 `config.py`로 관리한다.

---

### `avoidance.py`

근거리 장애물과 Dynamic Obstacle에 대한 Local Obstacle Avoidance를 담당한다.

초기 구현은 단순한 Rule-based 방식을 우선한다.

예:

```text
앞이 안전
→ Path Follower command 사용

앞이 위험
→ 좌/우 clearance 비교
→ 더 안전한 방향으로 회전/감속
```

출력:

```text
(left_speed, right_speed)
```

규칙:

- 초기 구현에서 DWA를 무리하게 강제하지 않는다.
- DWA 등 고급 Local Planner는 기본 end-to-end 동작이 안정된 뒤 검토한다.
- Motor를 직접 호출하지 않는다.
- 즉시 충돌 위험이 있을 때 Avoidance command가 Path Follower command보다 우선한다.
- Dynamic obstacle 대응에서 일시 센서 노이즈와 실제 장애물을 구분할 필요가 있으면 filtering/debounce 기준을 문서화한다.

---

### `vision.py`

Camera image 기반 Target Detection을 담당한다.

기본 인터페이스:

```python
target = detector.detect(image)
```

원칙:

- Target의 색상/형태/특징은 당일 공개 전까지 추측하지 않는다.
- 단순한 Vision 문제라는 사전 안내를 고려해, 불필요한 Deep Learning 모델을 먼저 도입하지 않는다.
- Camera pixel 좌표와 World `(x, y)` 좌표는 같은 것이 아니다.
- Target detection 결과 형식은 실제 미션 요구가 확인된 뒤 고정한다.

예상 가능한 단순 파이프라인:

```text
BGR/RGB
→ HSV 등 변환
→ Threshold
→ Mask
→ Contour
→ Target 후보
```

이는 예시일 뿐 당일 Target 사양보다 우선하지 않는다.

---

### `state_machine.py`

Mission-level state를 관리한다.

기본 상태:

```text
INIT
EXPLORE
GO_TO_TARGET
RETURN_HOME
DONE
```

기본 전이:

```text
INIT
→ EXPLORE
→ Target 발견
→ GO_TO_TARGET
→ Target 처리 완료
→ 아직 남은 Target 존재: EXPLORE
→ 모든 Target 완료: RETURN_HOME
→ DONE
```

규칙:

- 상태를 필요 이상으로 세분화하지 않는다.
- A* 한 번 호출 같은 짧은 계산을 굳이 State로 만들지 않는다.
- 상태 전이 조건은 `techweek_controller.py`에서 한눈에 추적 가능하게 유지한다.

---

## 6. 좌표와 단위 Convention

이 규칙은 팀 전체와 모든 AI가 반드시 지킨다.

### World Coordinate

```text
(x, y)
```

- 단위: meter
- Localization / Path Following에서 사용

### Orientation

```text
theta
```

- 단위: radian
- `sin`, `cos`, `atan2`와 일관되게 사용

### Grid Coordinate

```text
(row, col)
```

- 단위: cell
- Mapping / Frontier / A*에서 사용

### 금지

```text
(x, y)와 (row, col)을 이름 없이 혼용
degree와 radian 혼용
meter와 cell index 혼용
```

명시적 변환:

```python
row, col = world_to_grid(x, y)
x, y = grid_to_world(row, col)
```

---

## 7. 전체 데이터 흐름

```text
                              Webots
                                 │
          ┌──────────────────────┼──────────────────────┐
          │                      │                      │
   Odometry/Orientation        LiDAR                  Camera
          │                      │                      │
          └──────────────┬───────┴──────────────┬───────┘
                         ▼                      ▼
                    robot_io.py            robot_io.py
                         │                      │
              ┌──────────┼──────────┐           │
              │          │          │           ▼
              │          │          │       vision.py
              │          │          │           │
              │          │          │           ▼
              │          │          │     state_machine.py
              │          │          │
              │          │          └───────────────┐
              │          │                          │
              ▼          ▼                          ▼
       localization.py  LiDAR Data              avoidance.py
              │          │                          │
            Pose         │                    avoidance command
              │          │                          │
              └──────┐   │                          │
                     ▼   ▼                          │
                    mapping.py                      │
                        │                           │
                 Occupancy Grid                     │
                        │                           │
                        ▼                           │
                   frontier.py                      │
                        │                           │
                 Exploration Goal                   │
                        │                           │
                        ▼                           │
                     astar.py                       │
                        │                           │
                    Grid Path                       │
                        │                           │
                grid_to_world()                     │
                        │                           │
                        ▼                           │
               path_follower.py                     │
                        │                           │
                  normal command                    │
                        │                           │
                        └───────────┬───────────────┘
                                    ▼
                           control selection
                                    │
                                    ▼
                               robot_io.py
                                    │
                                    ▼
                                  Motor
                                    │
                                    ▼
                                  Webots
```

핵심:

- Webots의 Motor / LiDAR / Camera / Odometry·Orientation 관련 센서 접근은 가능한 한 `robot_io.py`를 경유한다.
- `localization.py`는 Odometry/Orientation 등 확인된 센서 정보를 이용해 현재 Robot Pose를 제공한다.
- `mapping.py`는 **Pose + LiDAR**를 이용해 Occupancy Grid를 갱신한다.
- `frontier.py`와 `astar.py`는 Grid Coordinate를 사용한다.
- A*의 Grid Path는 `grid_to_world()`를 통해 World Waypoint로 변환한 뒤 `path_follower.py`에 전달한다.
- Camera 데이터는 `robot_io.py`에서 읽은 뒤 `vision.py`로 전달한다.
- LiDAR 데이터는 Mapping뿐 아니라 `avoidance.py`의 근거리 충돌 위험 판단에도 사용될 수 있다.
- `path_follower.py`는 normal command를, `avoidance.py`는 위험 상황의 avoidance command를 각각 계산한다.
- 두 command는 병렬 후보이며, Path Follower의 출력이 Avoidance의 입력이 아니다.
- 최종 wheel speed를 정하는 control selection은 통합 계층에서 수행한다.
- Path Follower와 Avoidance는 Motor를 직접 건드리지 않는다.
- `robot_io.py`가 최종 Motor API를 호출한다.

---

## 8. Control Priority

정상 상황:

```text
A*
→ Path Follower
→ Wheel Command
```

즉시 충돌 위험:

```text
Avoidance
→ Wheel Command
```

개념 예시:

```python
if obstacle_is_dangerous:
    left, right = avoidance.compute_control(...)
else:
    left, right = path_follower.compute_control(...)

robot_io.set_wheel_speed(left, right)
```

금지:

- Path Follower와 Avoidance가 동시에 Motor API를 호출
- 두 모듈이 서로의 내부 상태를 임의 변경
- 장애물 회피가 끝났는데 기존 path 복귀 정책이 불명확한 상태로 방치

---

## 9. 사전 구현 목표

해커톤 전 목표:

### 최대한 완성 + Unit Test

```text
config.py
astar.py
frontier.py
state_machine.py
```

### 알고리즘 로직을 최대한 완성하되 환경 연동은 당일 확인

```text
mapping.py
path_follower.py
avoidance.py
```

### Interface / Skeleton 중심

```text
robot_io.py
localization.py
vision.py
techweek_controller.py
```

주의:

- "90% 완성"이라는 표현은 환경 의존 부분까지 테스트됐다는 의미가 아니다.
- 실제 sensor/device integration은 당일 환경에서 검증해야 한다.
- 사전에는 pure logic과 interface를 안정시키는 것을 목표로 한다.

---

## 10. 테스트 규칙

Webots 없이 검증 가능한 알고리즘은 Unit Test를 작성한다.

예상 테스트:

```text
tests/test_astar.py
tests/test_frontier.py
tests/test_mapping.py
tests/test_path_follower.py
tests/test_avoidance.py
tests/test_state_machine.py
```

### 테스트 파일 담당

```text
test_path_follower.py
test_state_machine.py
→ 임도연

test_mapping.py
→ 황예진

test_astar.py
test_frontier.py
→ 신지은

test_avoidance.py
→ 반현학
```

담당자는 자신의 기능 구현과 함께 해당 Unit Test까지 관리한다.

### `test_astar.py`

검증:

- 장애물을 피해 목적지 도달
- 시작점/목적지 처리
- 경로 없음
- map bounds
- 장애물 통과 금지

### `test_frontier.py`

검증:

- FREE + UNKNOWN 경계 탐지
- OCCUPIED를 Frontier로 선택하지 않음
- UNKNOWN이 없는 경우
- Frontier 후보 선택

### `test_mapping.py`

검증:

- World → Grid 변환
- Grid → World 변환
- Bounds
- FREE/OCCUPIED marking
- invalid range 처리

### `test_path_follower.py`

검증 예:

```text
Pose (0,0,0), Waypoint (1,0)
→ 전진 성향

Pose (0,0,0), Waypoint (0,1)
→ 좌회전 성향

Waypoint tolerance 안
→ 다음 waypoint
```

### `test_avoidance.py`

검증:

- 앞이 안전하면 회피 불필요
- 앞이 위험하고 왼쪽이 넓으면 왼쪽 회피
- 앞이 위험하고 오른쪽이 넓으면 오른쪽 회피
- 양쪽 모두 위험한 경우 명시된 fallback

### `test_state_machine.py`

검증:

- 초기 상태가 `INIT`인지 확인
- 초기화 완료 후 `INIT → EXPLORE` 전이
- Target 발견 시 `EXPLORE → GO_TO_TARGET` 전이
- Target 처리 후 남은 Target이 있으면 `GO_TO_TARGET → EXPLORE` 전이
- 모든 Target 처리가 끝나면 `GO_TO_TARGET → RETURN_HOME` 전이
- 시작 지점 복귀 완료 시 `RETURN_HOME → DONE` 전이
- 허용되지 않은 상태 전이가 임의로 발생하지 않는지 확인

### 테스트 행동 규칙

- 기존 테스트를 통과시키기 위해 테스트를 삭제/skip하지 않는다.
- assertion을 의미 없이 약화하지 않는다.
- 실행하지 않은 테스트를 "통과했다"고 말하지 않는다.
- 실패한 테스트와 미실행 테스트를 구분해서 보고한다.
- 버그 수정 시 가능하면 회귀 테스트를 추가한다.

---

## 11. Git Branch 전략

기본 흐름:

```text
feature/* 또는 fix/* 또는 docs/*
                ↓
             develop
                ↓
          통합 테스트
                ↓
              main
```

브랜치 의미:

```text
main
- 안정 버전
- 해커톤 제출/최종 검증 기준
- 초기 scaffold 이후 직접 개발 금지

develop
- 팀 통합 브랜치
- feature 결과를 합쳐 전체 동작을 검증
- 직접 기능 개발 금지

feature/*
- 실제 기능 개발

fix/*
- 버그 수정

docs/*
- 문서/규칙 변경
```

현재 4인 역할 기준으로 다음 작업 브랜치를 사용할 수 있다.

```text
feature/integration
feature/mapping
feature/planning
feature/avoidance-vision
```

단:

- 장기간 서로 고립시키지 않는다.
- 작은 기능이 안정되면 `develop`에 자주 통합한다.
- feature → feature 직접 merge를 기본 흐름으로 사용하지 않는다.
- 다른 팀원 변경이 필요하면 최신 `develop`을 가져와 동기화한다.

### Branch 생성 기본 순서

```bash
git switch develop
git pull origin develop
git switch -c feature/<scope>
```

작업 유형에 따라 `fix/`, `docs/` 사용 가능.

---

## 12. Git 작업 안전 규칙

AI는 저장소 작업을 시작할 때 먼저 현재 상태를 확인한다.

```bash
pwd
git rev-parse --show-toplevel
git status --short --branch
git log -5 --oneline
git remote -v
```

규칙:

- `main`, `develop`에서 기능 코드를 직접 수정하지 않는다.
- 초기 scaffold 세팅 이후 모든 변경은 작업 브랜치에서 수행한다.
- 사용자의 미커밋 변경을 임의로 되돌리지 않는다.
- 현재 branch가 불명확하면 변경 전에 확인한다.
- remote owner/repository를 추측하지 않는다.
- push 전에 `origin`을 확인한다.
- commit/push/PR/merge는 사용자가 명시적으로 요청한 범위에서만 수행한다.
- AI가 임의로 main/develop merge를 수행하지 않는다.
- branch 삭제 역시 사용자가 요청하지 않으면 수행하지 않는다.

금지:

```text
git reset --hard
git clean -fd
git push --force
rm -rf
```

사용자 작업을 파괴할 수 있는 명령을 실행하지 않는다.

---

## 13. Commit / PR 규칙

권장 commit prefix:

```text
feat: 기능 추가
fix: 버그 수정
test: 테스트 추가/수정
refactor: 동작 변경 없는 구조 개선
docs: 문서 변경
chore: 설정/구조/잡무
```

예:

```text
feat: implement A* path planning
feat: add frontier detection
test: add path follower turning cases
fix: correct world to grid conversion
docs: define coordinate conventions
chore: initialize Webots project structure
```

원칙:

- 한 commit은 한 가지 논리적 목적에 집중한다.
- unrelated formatting이나 대규모 refactor를 섞지 않는다.
- 관련 코드와 그 코드를 검증하는 테스트는 같은 목적의 commit에 포함할 수 있다.

PR 기본:

```text
작업 branch
→ develop
```

통합 검증 완료 후:

```text
develop
→ main
```

PR에 최소한 다음을 기록한다.

```text
변경 목적
변경 파일
주요 동작
테스트 결과
미실행/실패 항목
환경 의존 TODO
남은 위험
```

---

## 14. 4인 역할 분담 기준

현재 분담:

### Member 1 — Integration / Webots (임도연)

담당:

```text
techweek_controller.py
config.py
robot_io.py
state_machine.py
path_follower.py
test_path_follower.py
test_state_machine.py
```

핵심:

- Webots 실행
- Device 연결
- 전체 모듈 통합
- 최종 Motor Command
- State Transition
- State Machine Unit Test
- A* Path 기반 Waypoint Following

### Member 2 — Localization / Mapping (황예진)

담당:

```text
localization.py
mapping.py
test_mapping.py
```

핵심:

- Pose `(x, y, theta)`
- Coordinate Convention
- Occupancy Grid
- LiDAR Map Update
- World ↔ Grid 좌표 변환

### Member 3 — Planning / Exploration (신지은)

담당:

```text
astar.py
frontier.py
test_astar.py
test_frontier.py
```

핵심:

- Global Path Planning
- Frontier Detection / Selection
- Occupancy Grid 기반 탐색
- Pure Algorithm Test

### Member 4 — Avoidance / Vision (반현학)

담당:

```text
avoidance.py
vision.py
test_avoidance.py
```

핵심:

- Local Obstacle Avoidance
- Dynamic Obstacle 대응
- Target Detection Interface
- OpenCV 기반 단순 Vision

규칙:

- "담당"은 독점 소유를 의미하지 않는다.
- 최소 한 명의 다른 팀원이 각 영역의 기본 입출력을 이해해야 한다.
- 통합 문제는 담당자 경계를 이유로 미루지 않는다.
- 공통 Interface를 변경해야 할 경우 영향받는 담당자와 먼저 공유한다.
- `path_follower.py`의 주 담당은 임도연이며, Path Following과 Obstacle Avoidance가 맞물리는 주행 제어 문제는 반현학이 함께 지원한다.

---

## 15. AI 구현 행동 규칙

AI는 다음 원칙을 따른다.

### 기존 구조 먼저 탐색

- 이름만 보고 새 파일을 만들지 않는다.
- 관련 모듈과 테스트를 먼저 읽는다.
- 기존 함수 signature와 data convention을 확인한다.
- 같은 책임의 class/function을 중복 생성하지 않는다.

### 최소 변경

- 요청받은 범위만 수정한다.
- 관련 없는 파일을 정리하거나 리팩터링하지 않는다.
- 해커톤에 불필요한 범용 프레임워크를 만들지 않는다.
- "미래 확장성"만을 이유로 복잡한 abstraction을 추가하지 않는다.

### 환경 미확정 정보

다음은 실제 확인 전 추측 금지:

```text
Motor Device Name
LiDAR Device Name
Camera Device Name
LiDAR FOV
LiDAR ray count
LiDAR min/max range
Odometry sensor format
Orientation sensor format
Camera resolution
Target color/shape/model
World size
Dynamic obstacle behavior
실제 timestep 최적값
평가 세부 가중치
```

필요한 경우:

```python
# TODO: hackathon environment에서 실제 device name 확인 후 연결
```

처럼 명시적으로 격리한다.

### Interface 보호

- 이미 팀이 사용하는 public function signature를 조용히 변경하지 않는다.
- 변경이 필요하면 영향받는 파일을 먼저 나열한다.
- 여러 모듈의 계약을 동시에 바꾸는 변경은 구현 전에 계획을 제시한다.

### 단순성과 테스트 가능성 우선

- A*와 Frontier는 Webots 없이 테스트 가능해야 한다.
- Mapping core도 가능한 한 pure logic으로 분리한다.
- 단순한 Rule-based Avoidance로 충분한 단계에서 DWA를 강제하지 않는다.
- 단순한 Color/Shape Detection으로 충분한 단계에서 Deep Learning을 강제하지 않는다.

---

## 16. 구현 전 계획이 필요한 경우

다음 작업은 코드를 바꾸기 전에 짧은 계획을 작성한다.

- 두 개 이상의 핵심 모듈을 수정
- coordinate convention 변경
- A* path 형식 변경
- Mapping cell convention 변경
- Pose representation 변경
- State transition 변경
- `requirements.txt` dependency 추가
- repository structure 변경
- Webots project layout 변경
- path follower / avoidance control contract 변경

계획에는 최소한 다음을 포함한다.

```text
변경 목적
수정 파일
유지해야 하는 interface
영향받는 모듈
테스트
당일 환경 의존 부분
제외 범위
```

---

## 17. 해커톤 당일 작업 원칙

해커톤 당일은 "개별 알고리즘 최고 성능"보다 **end-to-end mission 통합 상태**를 지속적으로 확인한다.

초기 확인:

```text
1. 제공 코드 수정 없이 실행
2. Webots world 정상 실행
3. Robot Device 목록 확인
4. Motor 동작 확인
5. LiDAR raw data shape/단위 확인
6. Camera image 확인
7. Odometry/Orientation sensor 확인
8. Target 정보 확인
```

그 다음:

```text
robot_io 연결
→ localization 연결
→ mapping 연결
→ frontier/A* 연결
→ path follower 연결
→ avoidance 연결
→ vision 연결
→ state machine 통합
```

원칙:

- 기능별 코드를 마지막에 한 번에 합치지 않는다.
- `develop`에서 조기 통합 테스트를 반복한다.
- 새 고급 알고리즘 도입 전에 현재 end-to-end 경로가 유지되는지 확인한다.
- 당일 운영진 코드나 world를 크게 재구성하기 전에 왜 필요한지 검토한다.
- 정상 동작하던 경로를 최적화 과정에서 망가뜨리지 않는다.

---

## 18. 보안과 비밀값

현재 프로젝트는 일반적인 Webots 로컬 프로젝트지만 다음 규칙을 적용한다.

- `.env`
- token
- API key
- GitHub credential
- 개인 정보

를 commit하거나 로그에 남기지 않는다.

필요한 비밀값이 생기면 실제 값은 저장소에 넣지 않고 별도 방법을 사용한다.

---

## 19. AI 간 인수인계

Claude, Codex 등 서로 다른 AI가 같은 branch/worktree를 이어서 작업할 수 있도록 다음을 남긴다.

권장 형식:

```text
작업:
현재 branch:
완료:
변경 파일:
핵심 결정:
현재 interface:
실행한 테스트:
테스트 결과:
실패/미실행:
환경 의존 TODO:
남은 작업:
주의할 사항:
```

규칙:

- 이미 완료한 작업을 이유 없이 다시 만들지 않는다.
- 다른 AI의 사용자 변경을 되돌리지 않는다.
- 테스트하지 않은 내용을 테스트했다고 말하지 않는다.
- 추측한 Webots 사양을 인수인계에서 사실로 확정하지 않는다.

---

## 20. 작업 종료 보고

AI는 작업 종료 시 사실만 보고한다.

```text
작업:
branch:
변경 요약:
변경 파일:
유지한 interface:
실행한 테스트:
테스트 결과:
실패/미실행:
환경 의존 TODO:
남은 위험/후속 작업:
```

완료 보고 전에 확인:

- 요청 범위만 변경했는가.
- World `(x,y)`와 Grid `(row,col)` convention을 지켰는가.
- radian/meter/cell 단위를 혼용하지 않았는가.
- Webots Device Name을 추측하지 않았는가.
- Target 특성을 추측하지 않았는가.
- Path Follower/Avoidance가 Motor를 직접 제어하지 않는가.
- 관련 테스트를 실제로 실행했는가.
- 실행하지 않은 테스트를 성공했다고 말하지 않았는가.
- `main`/`develop`에 직접 개발 commit을 만들지 않았는가.
- 사용자 미커밋 변경을 보존했는가.

---

## 21. 핵심 원칙 요약

```text
1. 당일 운영진 제공 자료가 최우선이다.
2. 모르는 Webots/Target 정보를 추측하지 않는다.
3. World와 Grid 좌표계를 엄격하게 구분한다.
4. Webots I/O와 알고리즘을 분리한다.
5. 순수 알고리즘은 Webots 없이 테스트 가능하게 만든다.
6. Path Follower와 Avoidance는 command만 반환한다.
7. Motor API는 robot_io를 통해 제어한다.
8. 기능 branch → develop → 통합 테스트 → main 순서를 지킨다.
9. 해커톤에 불필요한 과도한 추상화와 dependency를 피한다.
10. end-to-end mission이 계속 동작하는 상태를 우선 유지한다.
```
