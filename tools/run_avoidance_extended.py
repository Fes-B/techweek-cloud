"""Run the extended endurance benchmark world headlessly."""

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
log = Path(tempfile.gettempdir(), "techweek-avoidance-extended.log")
report = Path(tempfile.gettempdir(), "techweek-avoidance-extended-result.json")
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
