"""Webots가 프로젝트 루트의 main.py를 실행하도록 연결하는 어댑터."""

import sys
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[2]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

from main import main


if __name__ == "__main__":
    main()
