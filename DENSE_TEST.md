# Dense corridor evaluation

Open `worlds/dense_obstacles.wbt` in Webots. Its controller records waypoint
progress and prints one `[RESULT]` JSON object before exiting the batch test.

The map has four alternating divider walls, boundary walls, and three moving
obstacles crossing free corridors. The actor boxes do not pass through walls.
Maximum actor speed is 0.40 m/s; robot wheel speed is limited to 0.24 m/s.
The test follows 18 waypoints out and back. These waypoints are a practice
global path, not an implemented exploration or A* system.

Pass requires completing the route with no recorded body/wheel obstacle
contacts. The run fails on first contact, 45 seconds without reducing distance
to the current waypoint, or 360 seconds total. A short safety stop is allowed.

The controller uses the provided starting pose, wheel encoders and an independent
Compass heading measurement. Compass is a practice assumption; the organiser
has not confirmed which separate orientation sensors will be available.
Supervisor position is used only to score final position error and contacts;
it is not passed to the avoidance planner or waypoint selector. The temporary
encoder adapter is `practice_odometry.py`, separate from team Localization.

Confirmed event constraints from the organiser's answer: differential drive;
no IMU device; separate sensors for related measurements; target details and
essential functional code provided on the day. Exact sensor models, locations,
names and robot dimensions are unknown. The two drive wheels, passive casters,
encoders, Compass, 360-degree LiDAR and camera in this world are practice choices.
This world must not be described as an exact copy of the competition world.

Configure `DynamicWindowAvoidance(lidar_field_of_view=..., lidar_pose=(x,y,yaw))`
with the actual sensor geometry. LiDAR samples must be ordered from right to
left in robot coordinates. Wheel geometry and speed limits remain in config.py.
When teammate modules arrive, replace encoder pose with Localization output
and the waypoint list with the global planner's output.
