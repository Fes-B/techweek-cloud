"""추후 Occupancy Grid Mapping을 구현할 자리입니다."""


class OccupancyGrid:
    def __init__(self):
        self.grid = None

    def update(self, pose, lidar_ranges):
        raise NotImplementedError("다음 단계에서 inverse sensor model을 구현합니다.")
