"""Run the extended endurance benchmark world headlessly.

    python tools/run_avoidance_extended.py              # official (420 s limit)
    python tools/run_avoidance_extended.py --unlimited  # DIAGNOSTIC: no time limit

The unlimited run keeps contacts and the stall criterion; it writes its own
log/result files so it can never be mistaken for the official result.
"""

import os
from pathlib import Path
import subprocess
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[1]
webots = (
    Path(os.environ.get("WEBOTS_HOME", r"C:\Program Files\Webots"))
    / "msys64/mingw64/bin/webots.exe"
)
if not webots.exists():
    webots = Path("webots")

environment = os.environ.copy()
environment["PATH"] = (
    str(Path(sys.executable).parent)
    + os.pathsep
    + environment.get("PATH", "")
)
unlimited = "--unlimited" in sys.argv[1:]
if unlimited:
    environment["EXTENDED_TIME_LIMIT"] = "unlimited"
suffix = "-unlimited" if unlimited else ""
log = Path(tempfile.gettempdir(), f"techweek-avoidance-extended{suffix}.log")
report = Path(tempfile.gettempdir(), f"techweek-avoidance-extended{suffix}-result.json")
report.unlink(missing_ok=True)

with log.open("w", encoding="utf-8") as stream:
    process = subprocess.Popen(
        [
            str(webots),
            "--batch",
            "--mode=fast",
            "--no-rendering",
            "--stdout",
            "--stderr",
            "--port=12512",
            str(ROOT / "worlds/avoidance_extended.wbt"),
        ],
        cwd=ROOT,
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    for line in process.stdout:
        stream.write(line)
        stream.flush()
    exit_code = process.wait()

print(f"Webots exit={exit_code}; log={log}")
if report.exists():
    print(report.read_text(encoding="utf-8"))
sys.exit(exit_code)
