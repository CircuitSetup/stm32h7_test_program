from pathlib import Path
import shutil

Import("env")


def copy_if_present(src: Path, dst: Path) -> None:
    if not src.exists():
        return
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(src, dst)


project_dir = Path(env["PROJECT_DIR"])
libdeps_dir = Path(env["PROJECT_LIBDEPS_DIR"]) / env["PIOENV"]
library_dir = libdeps_dir / "STM32duino LwIP" / "src" / "arch"

copy_if_present(project_dir / "include" / "arch" / "cc.h", library_dir / "cc.h")
copy_if_present(project_dir / "include" / "arch" / "sys_arch.h", library_dir / "sys_arch.h")
