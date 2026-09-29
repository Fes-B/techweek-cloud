# Unvalidated experiments (not applied)

`unvalidated_detour_and_crossing_commit.patch` applies on top of commit
`26c8633` and was **not** kept in HEAD because it was never run against the
full minimal/stress regression and its last part still produced a contact.

Contents and what was observed with the unlimited Extended diagnostic:

1. Detour wall-follow in narrow channels (centre-following standoff, narrow
   front check, corner-wrap priority, 90 s cap): the robot passes Extended
   waypoint 7 around slalom obstacle 3 without waypoint changes (7/39 -> 27/39).
2. Crossing gap acceptance checks the whole commit line against movers
   (`_commit_path_is_clear`): removed a contact in corridor 2; the robot then
   waits safely at actor C (permanent stall at waypoint 27).
3. Commit length from the mover's lane line (max 1.6 m): a contact remained in
   the mixed static/dynamic zone (segment diagnostic from waypoint 29).

Apply with `git apply experiments/unvalidated_detour_and_crossing_commit.patch`
and rerun `tools/run_avoidance_courses.py`, Practice and Extended before use.
