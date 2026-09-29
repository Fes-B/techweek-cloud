"""Mapping은 아직 미구현임을 명시적으로 확인하는 임시 테스트."""

import sys
import unittest
from pathlib import Path

PROJECT_DIR = Path(__file__).parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from mapping import OccupancyGrid


class TestMappingStub(unittest.TestCase):
    def test_mapping_is_intentionally_not_implemented_yet(self):
        grid = OccupancyGrid()
        with self.assertRaises(NotImplementedError):
            grid.update(None, None)


if __name__ == "__main__":
    unittest.main()
