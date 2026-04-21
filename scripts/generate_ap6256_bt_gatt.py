from pathlib import Path
import subprocess
import sys

try:
    Import("env")
except NameError:
    env = None


if env is not None:
    project_dir = Path(env["PROJECT_DIR"])
else:
    project_dir = Path(__file__).resolve().parents[1]

gatt_src = project_dir / "src" / "ap6256_bt_peripheral.gatt"
generated_dir = project_dir / "include" / "generated"
generated_dir.mkdir(parents=True, exist_ok=True)
gatt_header = generated_dir / "ap6256_bt_peripheral_gatt.h"
compiler = project_dir / "third_party" / "btstack" / "tool" / "compile_gatt.py"

result = subprocess.run(
    [sys.executable, str(compiler), str(gatt_src), str(gatt_header)],
    cwd=str(project_dir),
    capture_output=True,
    text=True,
)

if result.returncode != 0:
    sys.stderr.write(result.stdout)
    sys.stderr.write(result.stderr)
    raise SystemExit(result.returncode)
