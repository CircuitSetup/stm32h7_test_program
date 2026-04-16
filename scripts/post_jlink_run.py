"""Resume the STM32 after PlatformIO's J-Link upload step.

PlatformIO's generated J-Link upload sequence resets and halts the core after
programming. That is useful for debugging, but confusing for RTT Viewer: the
viewer connects successfully and then sees no live `board_test>` prompt because
the firmware is not running. This post-action leaves the target running unless
KARIOS_SKIP_JLINK_RUN=1 is set in the environment.
"""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

Import("env")  # type: ignore[name-defined]


def _candidate_jlink_paths() -> list[Path]:
    paths: list[Path] = []
    env_jlink = os.environ.get("JLINK_EXE")
    if env_jlink:
        paths.append(Path(env_jlink))

    project_packages = Path(env.subst("$PROJECT_PACKAGES_DIR"))  # type: ignore[name-defined]
    paths.append(project_packages / "tool-jlink" / "JLink.exe")
    paths.append(Path(r"C:\Program Files\SEGGER\JLink_V924a\JLink.exe"))
    paths.append(Path("JLink.exe"))
    return paths


def _find_jlink() -> Path | None:
    for path in _candidate_jlink_paths():
        if path.exists() or path.name.lower() == "jlink.exe":
            return path
    return None


def _post_upload_run(source, target, env):  # noqa: ANN001
    if os.environ.get("KARIOS_SKIP_JLINK_RUN") == "1":
        print("KARIOS_SKIP_JLINK_RUN=1, leaving target state unchanged after upload.")
        return

    jlink = _find_jlink()
    if jlink is None:
        print("JLink.exe not found; target may remain halted after upload.")
        return

    out_dir = Path(env.subst("$PROJECT_DIR")) / ".pio" / "hil_logs"
    out_dir.mkdir(parents=True, exist_ok=True)
    script = out_dir / "post_upload_run.jlink"
    script.write_text(
        "\n".join(
            [
                "device STM32H743VI",
                "si SWD",
                "speed 1000",
                "connect",
                "r",
                "g",
                "exit",
            ]
        )
        + "\n",
        encoding="ascii",
    )

    try:
        proc = subprocess.run(
            [str(jlink), "-CommanderScript", str(script)],
            cwd=env.subst("$PROJECT_DIR"),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=30,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        print(f"Post-upload J-Link run failed: {exc}")
        return

    log = out_dir / "post_upload_run.txt"
    log.write_text(proc.stdout, encoding="utf-8", errors="ignore")
    if proc.returncode == 0:
        print(f"Post-upload J-Link run/resume complete ({log}).")
    else:
        print(f"Post-upload J-Link run/resume failed with {proc.returncode} ({log}).")


env.AddPostAction("upload", _post_upload_run)  # type: ignore[name-defined]
