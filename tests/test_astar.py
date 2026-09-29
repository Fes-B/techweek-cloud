"""A*는 아직 미구현임을 명시적으로 확인하는 임시 테스트."""

import sys
import unittest
from pathlib import Path

PROJECT_DIR = Path(__file__).parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from astar import find_path


class TestAStarStub(unittest.TestCase):
    def test_astar_is_intentionally_not_implemented_yet(self):
        with self.assertRaises(NotImplementedError):
            find_path(None, (0, 0), (1, 1))


if __name__ == "__main__":
    unittest.main()
