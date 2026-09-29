"""Encoder/compass practice adapter; replace with teammate Localization output."""
import math
from config import ODOMETRY_WHEEL_RADIUS, TIME_STEP


class EncoderPose:
    def __init__(self, robot, initial_pose):
        self.pose = tuple(initial_pose)
        self.left = robot.getDevice('left wheel sensor')
        self.right = robot.getDevice('right wheel sensor')
        self.left.enable(TIME_STEP)
        self.right.enable(TIME_STEP)
        self.compass = robot.getDevice('heading compass')
        self.compass.enable(TIME_STEP)
        self.initial_yaw = initial_pose[2]
        self.initial_bearing = None
        self.previous = None
        # Diagnostic-only mirrors of the last integration step (never used by
        # the planner); they make encoder vs. truth comparisons possible
        # without duplicating the encoder bookkeeping in a controller.
        self.last_ds = 0.0
        self.last_dl = 0.0
        self.last_dr = 0.0
        self.last_heading = None
        self.last_turn = 0.0

    def update(self):
        readings = (self.left.getValue(), self.right.getValue())
        if not all(math.isfinite(value) for value in readings):
            return self.pose
        north = self.compass.getValues()
        bearing = math.atan2(north[1], north[0])
        if self.initial_bearing is None:
            self.initial_bearing = bearing
        heading = self.initial_yaw + self.initial_bearing - bearing
        heading = math.atan2(math.sin(heading), math.cos(heading))
        self.last_ds = 0.0
        self.last_dl = 0.0
        self.last_dr = 0.0
        self.last_turn = 0.0
        if self.previous is not None:
            dl = (readings[0] - self.previous[0]) * ODOMETRY_WHEEL_RADIUS
            dr = (readings[1] - self.previous[1]) * ODOMETRY_WHEEL_RADIUS
            ds = (dl + dr) / 2
            x, y, yaw = self.pose
            turn = math.atan2(math.sin(heading - yaw), math.cos(heading - yaw))
            self.pose = (x + ds * math.cos(yaw + turn / 2),
                         y + ds * math.sin(yaw + turn / 2),
                         heading)
            self.last_dl = dl
            self.last_dr = dr
            self.last_ds = ds
            self.last_turn = turn
        self.last_heading = heading
        self.previous = readings
        return self.pose
