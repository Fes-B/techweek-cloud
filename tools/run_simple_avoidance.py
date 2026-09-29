"""Run all minimal Webots cases against ProgressWatchdogDWA."""

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[1]
CASES = ("front_left", "front_right", "front", "corridor")
webots = Path(os.environ.get("WEBOTS_HOME", r"C:\Program Files\Webots")) / "msys64/mingw64/bin/webots.exe"
if not webots.exists():
    webots = Path("webots")

results = []
for index, case_name in enumerate(CASES):
    environment = os.environ.copy()
    environment["SIMPLE_DWA_CASE"] = case_name
    environment["PATH"] = str(Path(sys.executable).parent) + os.pathsep + environment.get("PATH", "")
    report = Path(tempfile.gettempdir(), f"techweek-simple-dwa-{case_name}.json")
    report.unlink(missing_ok=True)
    log = Path(tempfile.gettempdir(), f"techweek-simple-dwa-{case_name}.log")
    with log.open("w", encoding="utf-8") as stream:
        process = subprocess.run(
            [
                str(webots), "--batch", "--mode=fast", "--no-rendering",
                "--stdout", "--stderr", f"--port={12600 + index}",
                str(ROOT / "worlds/avoidance_minimal.wbt"),
            ],
            cwd=ROOT,
            env=environment,
            stdout=stream,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
        )
    if report.exists():
        result = json.loads(report.read_text(encoding="utf-8"))
    else:
        result = {"case": case_name, "passed": False, "error": "missing report"}
    result["webots_exit"] = process.returncode
    result["log"] = str(log)
    results.append(result)
    print(json.dumps(result, sort_keys=True), flush=True)

passed = all(result.get("passed") for result in results)
print(json.dumps({"passed": passed, "results": results}, indent=2), flush=True)
sys.exit(0 if passed else 1)
