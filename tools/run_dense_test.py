"""Run with `py -3.10 tools/run_dense_test.py`; results go to the OS temp folder."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
webots = Path(os.environ.get('WEBOTS_HOME', r'C:\Program Files\Webots')) / 'msys64/mingw64/bin/webots.exe'
if not webots.exists():
    webots = Path('webots')
env = os.environ.copy()
env['PATH'] = str(Path(sys.executable).parent) + os.pathsep + env.get('PATH', '')
log = Path(tempfile.gettempdir(), 'techweek-dense-benchmark.log')
report = Path(tempfile.gettempdir(), 'techweek-dense-result.json')
report.unlink(missing_ok=True)
with log.open('w', encoding='utf-8') as stream:
    process = subprocess.Popen([
        str(webots), '--batch', '--mode=fast', '--no-rendering', '--stdout',
        '--stderr', '--port=12510', str(ROOT / 'worlds/dense_obstacles.wbt')],
        cwd=ROOT, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, encoding='utf-8', errors='replace')
    for line in process.stdout:
        stream.write(line)
        stream.flush()
    exit_code = process.wait()
print(f'Webots exit={exit_code}; log={log}')
if report.exists():
    print(report.read_text(encoding='utf-8'))
sys.exit(exit_code)
