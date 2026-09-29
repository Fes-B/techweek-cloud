# 기존 DWA avoidance 발전 작업 인수인계 (claude/avoidance-evolve-original)

작업: 기존 `avoidance.py` 구조(DWA, recovery, wall-follow, backoff, front-corner escape,
dynamic tracking/crossing, release hysteresis)를 유지하면서 Webots에서 확인된 실패 원인을
로그 기반으로 분류·수정. 대규모 재설계 없음.

## 검증 환경

- Webots R2025a Linux, 컨테이너(소프트웨어 OpenGL, `xvfb-run`)에서 실제 실행.
- 모든 수치는 이 환경 기준이다. Windows/GPU 환경에서는 LiDAR 렌더링 차이(아래 참고)가
  있을 수 있으므로 로컬 재검증이 필요하다.

## 발견한 근본 원인 (분류)

| 분류 | 원인 | 수정 |
| --- | --- | --- |
| C | 전진 판정 임계 0.02 m/s > 한 스텝 가속 0.6×0.032=0.0192 m/s → 정지 상태를 corner trap으로 오분류 | `_FORWARD_SPEED_THRESHOLD` = 0.5×가속×dt |
| I | 360° LiDAR 각도 모델이 fov/(N-1) (실측 Webots는 fov/N) → 섹터 비대칭, 좌우 mirror 깨짐 | `_beam_angle_step` |
| E | 양쪽 측면이 열려 있으면 기본 좌회전 → front-left 장애물로 회전 | 전방 반쪽 비교 → 목표 방향 → 이전 방향 |
| E | recovery escape 직후 거리 trigger가 여전히 참 → recovery↔DWA 반복, 회전 방향 초기화 | re-entry latch (전방 arc가 비어질 때까지) |
| B | score가 goal 거리 위주, 1.2 s 안전창 너머의 막힘을 못 봄; 제자리 회전이 stop penalty 우회 | full-horizon headroom 항(점수 전용), idle penalty |
| E | 제자리 회전 nominal이 "안전"으로 판단되어 path follower로 조기 반환 | release 시 목표 방향 직선의 headroom 확인 |
| G | 정적 박스 옆을 지날 때 보이는 면의 중심이 이동 → phantom track | ray-consistency 운동 증거가 있어야 신규 track 승격 |
| D | 후진이 safety gate에 막혀 STOP만 보내도 backoff 진행으로 계산 → 무한 재진입 | 실제 통과한 명령만 진행으로 계산, 불가 시 DWA로 양보 |
| E | waypoint가 장애물 옆에 있을 때 backoff/recovery trigger와 충돌 | trigger를 목표가 요구하는 거리로 cap (충돌 margin 이하로는 안 내려감) |
| I | Webots 소프트웨어 렌더링 LiDAR가 뷰 경계(±45°, ±135°)에서 단일 ray를 누락/근거리 오검출 | footprint 내부 고립 ray 제거, tangent 판단 시 로봇 폭만큼의 연속 free ray 요구 |
| B/E | 대칭 장애물 앞에서 방향 결정 없음 | latched episode 동안 목표 기준 side 선호(점수 전용, 히스테리시스) |
| E | local minimum(U, 막힌 목표) | pose 기반 progress watchdog → 기존 wall-follow를 detour로 사용(Bug2식 종료) |
| 동적 | 횡단 대기 중 path follower로 제어권 반환, lane 끝에서 정지한 actor를 잊음 → 충돌 | 대기 중 release 금지, 경로 옆에 정지한 mover가 있으면 최대 10 s 대기 유지 |

## 안전 상수

로봇 반경, static/dynamic clearance margin, 충돌 threshold, world, waypoint는 변경하지 않았다.

## 코스 자체의 한계 (벤치마크 판정과 구분)

- Extended waypoint index 7 `(6.50,-6.50)`: 벽 끝과 0.10 m, 6→7 직선이 벽을 관통, 안전 경로는
  slalom obstacle 3을 돌아가는 3.1 m 우회뿐 (`tools/check_course_feasibility.py`). 원본 Extended는
  여기서 정지(접촉 0)한다.
- Extended 전체 직선거리 71 m / practice path follower 속도 0.16 m/s = 444 s > MAX_SECONDS 420 s.
- Extended corridor 2 actor C: lane이 복도 전체를 가로지르고 양 끝 dwell 위치가 경로에서
  0.26 m. dwell 3.5 s 안에 lane을 벗어나는 데 0.22–0.24 m/s로 2.9–3.9 s 필요 → dynamic margin을
  지키면서 통과를 보장할 수 없음. 현재 동작은 안전 대기.
- Practice `completed=false`: crossing commit이 정상 종료(`ox+0.25` 거리 소진, y≈0.80)하지만
  벤치마크는 y>0.90에서만 complete로 기록.

## 도구

- `tools/avoidance_trace.py`: `AVOID_TRACE=<file>`로 벤치마크/코스 컨트롤러 입력 기록, 오프라인 재생.
- `tools/check_course_feasibility.py`: world+waypoint의 기하학적 통과 가능성.
- `tools/build_avoidance_courses.py`, `tools/run_avoidance_courses.py`: 최소/스트레스 코스 생성·실행
  (`worlds/course_*.wbt`, 컨트롤러 `avoidance_course_controller`).

로컬 실행 예:

```bash
python tools/build_avoidance_courses.py --check
python tools/run_avoidance_courses.py min_ --jobs 2
python tools/run_avoidance_courses.py stress_ --jobs 2 --trace-dir traces
python tools/run_avoidance_practice.py
python tools/run_avoidance_extended.py
```
