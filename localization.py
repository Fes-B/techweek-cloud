"""추후 Odometry/위치 추정을 구현할 자리입니다."""


class Pose2D:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta

    def update(self, x, y, theta):
        """TODO: 바퀴 encoder 기반 odometry로 교체합니다."""
        self.x, self.y, self.theta = x, y, theta
