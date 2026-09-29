"""최종 미션 상태의 이름만 먼저 정의합니다."""

from enum import Enum, auto


class MissionState(Enum):
    EXPLORE = auto()
    GO_TARGET = auto()
    RETURN_HOME = auto()
    DONE = auto()


class MissionStateMachine:
    def __init__(self):
        self.state = MissionState.EXPLORE

    def update(self):
        """TODO: 각 단계가 구현될 때 전이 조건을 추가합니다."""
        return self.state
