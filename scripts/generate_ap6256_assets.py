from __future__ import annotations

import hashlib
from pathlib import Path

try:
    Import("env")
except NameError:
    env = None


ASSETS = [
    {
        "macro": "WIFI_FIRMWARE",
        "symbol": "wifi_firmware",
        "relative_path": "assets/ap6256/brcmfmac43456-sdio.bin",
    },
    {
        "macro": "WIFI_CLM_BLOB",
        "symbol": "wifi_clm_blob",
        "relative_path": "assets/ap6256/brcmfmac43456-sdio.clm_blob",
    },
    {
        "macro": "WIFI_NVRAM",
        "symbol": "wifi_nvram",
        "relative_path": "assets/ap6256/brcmfmac43456-sdio.txt",
    },
    {
        "macro": "BT_PATCHRAM",
        "symbol": "bt_patchram",
        "relative_path": "assets/ap6256/BCM4345C0_003.001.025.0162.0000_Generic_UART_37_4MHz_wlbga_ref_iLNA_iTR_eLG.hcd",
    },
    {
        "macro": "WIFI_NVRAM_REFERENCE",
        "symbol": "wifi_nvram_reference",
        "relative_path": "assets/ap6256/reference/nvram_ap6256.txt",
    },
]


def sha256_hex(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_if_changed(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    current = path.read_text(encoding="utf-8") if path.exists() else None
    if current != content:
        path.write_text(content, encoding="utf-8", newline="\n")


if env is not None:
    project_dir = Path(env["PROJECT_DIR"])
else:
    project_dir = Path(__file__).resolve().parents[1]
manifest_header = project_dir / "include" / "generated" / "ap6256_assets_manifest.h"
assembly_source = project_dir / "src" / "generated" / "ap6256_assets_data.S"

header_lines = [
    "#ifndef AP6256_ASSETS_MANIFEST_H",
    "#define AP6256_ASSETS_MANIFEST_H",
    "",
    "#include <stdint.h>",
    "",
    "#define AP6256_ASSET_MANIFEST_VERSION 1U",
    "",
]

asm_lines = [
    ".syntax unified",
    "",
]

for asset in ASSETS:
    path = project_dir / asset["relative_path"]
    if not path.exists():
        raise FileNotFoundError(f"Missing AP6256 asset: {path}")

    file_name = path.name
    size = path.stat().st_size
    sha = sha256_hex(path)
    symbol = asset["symbol"]
    macro = asset["macro"]
    absolute_asset = path.resolve().as_posix()

    header_lines.extend(
        [
            f'#define AP6256_ASSET_{macro}_FILENAME "{file_name}"',
            f"#define AP6256_ASSET_{macro}_SIZE {size}U",
            f'#define AP6256_ASSET_{macro}_SHA256 "{sha}"',
            f"extern const uint8_t ap6256_asset_{symbol}_start[];",
            f"extern const uint8_t ap6256_asset_{symbol}_end[];",
            "",
        ]
    )

    asm_lines.extend(
        [
            f'.section .rodata.ap6256_{symbol},"a",%progbits',
            ".balign 4",
            f".global ap6256_asset_{symbol}_start",
            f".global ap6256_asset_{symbol}_end",
            f"ap6256_asset_{symbol}_start:",
            f'.incbin "{absolute_asset}"',
            f"ap6256_asset_{symbol}_end:",
            ".byte 0",
            "",
        ]
    )

header_lines.append("#endif /* AP6256_ASSETS_MANIFEST_H */")

write_if_changed(manifest_header, "\n".join(header_lines) + "\n")
write_if_changed(assembly_source, "\n".join(asm_lines) + "\n")
