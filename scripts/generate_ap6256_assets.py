from __future__ import annotations

import hashlib
import os
from pathlib import Path

try:
    Import("env")
except NameError:
    env = None


RPI_7_84_17_1_BASE_URL = (
    "https://raw.githubusercontent.com/RPi-Distro/firmware-nonfree/"
    "bullseye/debian/config/brcm80211/brcm"
)
RPI_7_84_17_1_DIR = "assets/ap6256/profiles/rpi_7_84_17_1"
MANJARO_AP6256_BASE_URL = (
    "https://gitlab.manjaro.org/manjaro-arm/packages/community/"
    "ap6256-firmware/-/raw/master"
)
MANJARO_AP6256_DIR = "assets/ap6256/profiles/manjaro_ap6256_2020_02"
HYBRID_AP6256_DIR = "assets/ap6256/profiles/hybrid_ap6256_safe5g"

COMMON_ASSETS = [
    {
        "macro": "BT_PATCHRAM",
        "symbol": "bt_patchram",
        "relative_path": "assets/ap6256/BCM4345C5.hcd",
        "source_url": f"{MANJARO_AP6256_BASE_URL}/BCM4345C5.hcd",
        "version_hint": "2020.02-manjaro-ap6256",
    },
    {
        "macro": "WIFI_NVRAM_REFERENCE",
        "symbol": "wifi_nvram_reference",
        "relative_path": "assets/ap6256/reference/nvram_ap6256.txt",
        "source_url": "repo:assets/ap6256/reference/nvram_ap6256.txt",
        "version_hint": "ap6256-module",
    },
]

WIFI_ASSET_PROFILES = {
    "current_repo": {
        "description": "current repository BCM43456 firmware, CLM, and AP6256 module NVRAM",
        "source_url": "repo:assets/ap6256",
        "firmware_version_hint": "current-repo",
        "default_generic_nvram": 0,
        "wifi": [
            {
                "macro": "WIFI_FIRMWARE",
                "symbol": "wifi_firmware",
                "relative_path": "assets/ap6256/brcmfmac43456-sdio.bin",
                "source_url": "repo:assets/ap6256/brcmfmac43456-sdio.bin",
                "version_hint": "current-repo",
            },
            {
                "macro": "WIFI_CLM_BLOB",
                "symbol": "wifi_clm_blob",
                "relative_path": "assets/ap6256/brcmfmac43456-sdio.clm_blob",
                "source_url": "repo:assets/ap6256/brcmfmac43456-sdio.clm_blob",
                "version_hint": "current-repo",
            },
            {
                "macro": "WIFI_NVRAM",
                "symbol": "wifi_nvram",
                "relative_path": "assets/ap6256/brcmfmac43456-sdio.txt",
                "source_url": "repo:assets/ap6256/brcmfmac43456-sdio.txt",
                "version_hint": "current-repo-generic",
            },
        ],
    },
    "rpi_7_84_17_1_ap6256_nvram": {
        "description": "Raspberry Pi BCM43456 7.84.17.1 firmware/CLM with AP6256 module NVRAM",
        "source_url": RPI_7_84_17_1_BASE_URL,
        "firmware_version_hint": "7.84.17.1",
        "default_generic_nvram": 0,
        "wifi": [
            {
                "macro": "WIFI_FIRMWARE",
                "symbol": "wifi_firmware",
                "relative_path": f"{RPI_7_84_17_1_DIR}/brcmfmac43456-sdio.bin",
                "source_url": f"{RPI_7_84_17_1_BASE_URL}/brcmfmac43456-sdio.bin",
                "version_hint": "7.84.17.1",
            },
            {
                "macro": "WIFI_CLM_BLOB",
                "symbol": "wifi_clm_blob",
                "relative_path": f"{RPI_7_84_17_1_DIR}/brcmfmac43456-sdio.clm_blob",
                "source_url": f"{RPI_7_84_17_1_BASE_URL}/brcmfmac43456-sdio.clm_blob",
                "version_hint": "7.84.17.1",
            },
            {
                "macro": "WIFI_NVRAM",
                "symbol": "wifi_nvram",
                "relative_path": f"{RPI_7_84_17_1_DIR}/brcmfmac43456-sdio.txt",
                "source_url": f"{RPI_7_84_17_1_BASE_URL}/brcmfmac43456-sdio.txt",
                "version_hint": "7.84.17.1-rpi",
            },
        ],
    },
    "rpi_7_84_17_1_coherent": {
        "description": "Raspberry Pi BCM43456 7.84.17.1 coherent firmware, CLM, and NVRAM set",
        "source_url": RPI_7_84_17_1_BASE_URL,
        "firmware_version_hint": "7.84.17.1",
        "default_generic_nvram": 1,
        "wifi": [
            {
                "macro": "WIFI_FIRMWARE",
                "symbol": "wifi_firmware",
                "relative_path": f"{RPI_7_84_17_1_DIR}/brcmfmac43456-sdio.bin",
                "source_url": f"{RPI_7_84_17_1_BASE_URL}/brcmfmac43456-sdio.bin",
                "version_hint": "7.84.17.1",
            },
            {
                "macro": "WIFI_CLM_BLOB",
                "symbol": "wifi_clm_blob",
                "relative_path": f"{RPI_7_84_17_1_DIR}/brcmfmac43456-sdio.clm_blob",
                "source_url": f"{RPI_7_84_17_1_BASE_URL}/brcmfmac43456-sdio.clm_blob",
                "version_hint": "7.84.17.1",
            },
            {
                "macro": "WIFI_NVRAM",
                "symbol": "wifi_nvram",
                "relative_path": f"{RPI_7_84_17_1_DIR}/brcmfmac43456-sdio.txt",
                "source_url": f"{RPI_7_84_17_1_BASE_URL}/brcmfmac43456-sdio.txt",
                "version_hint": "7.84.17.1-rpi",
            },
        ],
    },
    "rpi_7_84_17_1_coherent_us": {
        "description": "Raspberry Pi BCM43456 7.84.17.1 coherent firmware/CLM/NVRAM with explicit US country",
        "source_url": f"{RPI_7_84_17_1_BASE_URL} + repo US NVRAM overlay",
        "firmware_version_hint": "7.84.17.1",
        "default_generic_nvram": 1,
        "wifi": [
            {
                "macro": "WIFI_FIRMWARE",
                "symbol": "wifi_firmware",
                "relative_path": f"{RPI_7_84_17_1_DIR}/brcmfmac43456-sdio.bin",
                "source_url": f"{RPI_7_84_17_1_BASE_URL}/brcmfmac43456-sdio.bin",
                "version_hint": "7.84.17.1",
            },
            {
                "macro": "WIFI_CLM_BLOB",
                "symbol": "wifi_clm_blob",
                "relative_path": f"{RPI_7_84_17_1_DIR}/brcmfmac43456-sdio.clm_blob",
                "source_url": f"{RPI_7_84_17_1_BASE_URL}/brcmfmac43456-sdio.clm_blob",
                "version_hint": "7.84.17.1",
            },
            {
                "macro": "WIFI_NVRAM",
                "symbol": "wifi_nvram",
                "relative_path": f"{RPI_7_84_17_1_DIR}/brcmfmac43456-sdio-us.txt",
                "source_url": "repo:assets/ap6256/profiles/rpi_7_84_17_1/brcmfmac43456-sdio-us.txt",
                "version_hint": "7.84.17.1-rpi-us",
            },
        ],
    },
    "rpi_7_84_17_1_safe5g_ap6256_nvram": {
        "description": "Raspberry Pi BCM43456 7.84.17.1 firmware/CLM with AP6256 conservative 5GHz NVRAM",
        "source_url": f"{RPI_7_84_17_1_BASE_URL} + repo AP6256 safe5g NVRAM",
        "firmware_version_hint": "7.84.17.1",
        "default_generic_nvram": 1,
        "wifi": [
            {
                "macro": "WIFI_FIRMWARE",
                "symbol": "wifi_firmware",
                "relative_path": f"{RPI_7_84_17_1_DIR}/brcmfmac43456-sdio.bin",
                "source_url": f"{RPI_7_84_17_1_BASE_URL}/brcmfmac43456-sdio.bin",
                "version_hint": "7.84.17.1",
            },
            {
                "macro": "WIFI_CLM_BLOB",
                "symbol": "wifi_clm_blob",
                "relative_path": f"{RPI_7_84_17_1_DIR}/brcmfmac43456-sdio.clm_blob",
                "source_url": f"{RPI_7_84_17_1_BASE_URL}/brcmfmac43456-sdio.clm_blob",
                "version_hint": "7.84.17.1",
            },
            {
                "macro": "WIFI_NVRAM",
                "symbol": "wifi_nvram",
                "relative_path": f"{HYBRID_AP6256_DIR}/nvram_ap6256_safe5g.txt",
                "source_url": "repo:assets/ap6256/profiles/hybrid_ap6256_safe5g/nvram_ap6256_safe5g.txt",
                "version_hint": "ap6256-safe5g",
            },
        ],
    },
    "current_repo_safe5g_ap6256_nvram": {
        "description": "current repository firmware/CLM with AP6256 conservative 5GHz NVRAM",
        "source_url": "repo:assets/ap6256 + repo AP6256 safe5g NVRAM",
        "firmware_version_hint": "current-repo",
        "default_generic_nvram": 1,
        "wifi": [
            {
                "macro": "WIFI_FIRMWARE",
                "symbol": "wifi_firmware",
                "relative_path": "assets/ap6256/brcmfmac43456-sdio.bin",
                "source_url": "repo:assets/ap6256/brcmfmac43456-sdio.bin",
                "version_hint": "current-repo",
            },
            {
                "macro": "WIFI_CLM_BLOB",
                "symbol": "wifi_clm_blob",
                "relative_path": "assets/ap6256/brcmfmac43456-sdio.clm_blob",
                "source_url": "repo:assets/ap6256/brcmfmac43456-sdio.clm_blob",
                "version_hint": "current-repo",
            },
            {
                "macro": "WIFI_NVRAM",
                "symbol": "wifi_nvram",
                "relative_path": f"{HYBRID_AP6256_DIR}/nvram_ap6256_safe5g.txt",
                "source_url": "repo:assets/ap6256/profiles/hybrid_ap6256_safe5g/nvram_ap6256_safe5g.txt",
                "version_hint": "ap6256-safe5g",
            },
        ],
    },
    "current_bin_rpi_clm": {
        "description": "current repository firmware with Raspberry Pi 7.84.17.1 CLM and AP6256 module NVRAM",
        "source_url": "repo:assets/ap6256 + raspberrypi/firmware-nonfree CLM",
        "firmware_version_hint": "current-repo",
        "default_generic_nvram": 0,
        "wifi": [
            {
                "macro": "WIFI_FIRMWARE",
                "symbol": "wifi_firmware",
                "relative_path": "assets/ap6256/brcmfmac43456-sdio.bin",
                "source_url": "repo:assets/ap6256/brcmfmac43456-sdio.bin",
                "version_hint": "current-repo",
            },
            {
                "macro": "WIFI_CLM_BLOB",
                "symbol": "wifi_clm_blob",
                "relative_path": f"{RPI_7_84_17_1_DIR}/brcmfmac43456-sdio.clm_blob",
                "source_url": f"{RPI_7_84_17_1_BASE_URL}/brcmfmac43456-sdio.clm_blob",
                "version_hint": "7.84.17.1",
            },
            {
                "macro": "WIFI_NVRAM",
                "symbol": "wifi_nvram",
                "relative_path": "assets/ap6256/brcmfmac43456-sdio.txt",
                "source_url": "repo:assets/ap6256/brcmfmac43456-sdio.txt",
                "version_hint": "current-repo-generic",
            },
        ],
    },
    "current_bin_rpi_nvram": {
        "description": "current repository firmware/CLM with Raspberry Pi 7.84.17.1 NVRAM",
        "source_url": "repo:assets/ap6256 firmware/CLM + raspberrypi/firmware-nonfree NVRAM",
        "firmware_version_hint": "current-repo",
        "default_generic_nvram": 1,
        "wifi": [
            {
                "macro": "WIFI_FIRMWARE",
                "symbol": "wifi_firmware",
                "relative_path": "assets/ap6256/brcmfmac43456-sdio.bin",
                "source_url": "repo:assets/ap6256/brcmfmac43456-sdio.bin",
                "version_hint": "current-repo",
            },
            {
                "macro": "WIFI_CLM_BLOB",
                "symbol": "wifi_clm_blob",
                "relative_path": "assets/ap6256/brcmfmac43456-sdio.clm_blob",
                "source_url": "repo:assets/ap6256/brcmfmac43456-sdio.clm_blob",
                "version_hint": "current-repo",
            },
            {
                "macro": "WIFI_NVRAM",
                "symbol": "wifi_nvram",
                "relative_path": f"{RPI_7_84_17_1_DIR}/brcmfmac43456-sdio.txt",
                "source_url": f"{RPI_7_84_17_1_BASE_URL}/brcmfmac43456-sdio.txt",
                "version_hint": "7.84.17.1-rpi",
            },
        ],
    },
    "manjaro_ap6256_2020_02_coherent": {
        "description": "Manjaro AP6256 firmware package, coherent firmware/CLM/AP6256 NVRAM",
        "source_url": MANJARO_AP6256_BASE_URL,
        "firmware_version_hint": "7.45.96.61-ap6256",
        "default_generic_nvram": 1,
        "wifi": [
            {
                "macro": "WIFI_FIRMWARE",
                "symbol": "wifi_firmware",
                "relative_path": f"{MANJARO_AP6256_DIR}/brcmfmac43456-sdio.bin",
                "source_url": f"{MANJARO_AP6256_BASE_URL}/fw_bcm43456c5_ag.bin",
                "version_hint": "7.45.96.61-ap6256",
            },
            {
                "macro": "WIFI_CLM_BLOB",
                "symbol": "wifi_clm_blob",
                "relative_path": f"{MANJARO_AP6256_DIR}/brcmfmac43456-sdio.clm_blob",
                "source_url": f"{MANJARO_AP6256_BASE_URL}/brcmfmac43456-sdio.clm_blob",
                "version_hint": "7.45.96.61-ap6256",
            },
            {
                "macro": "WIFI_NVRAM",
                "symbol": "wifi_nvram",
                "relative_path": f"{MANJARO_AP6256_DIR}/brcmfmac43456-sdio.AP6256.txt",
                "source_url": f"{MANJARO_AP6256_BASE_URL}/brcmfmac43456-sdio.AP6256.txt",
                "version_hint": "ap6256-manjaro",
            },
        ],
    },
}


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


def c_string(value: str) -> str:
    return value.replace("\\", "\\\\").replace('"', '\\"')


def get_project_dir() -> Path:
    if env is not None:
        return Path(env["PROJECT_DIR"])
    return Path(__file__).resolve().parents[1]


def selected_profile_name() -> str:
    profile = os.environ.get("AP6256_WIFI_ASSET_PROFILE")
    if not profile and env is not None:
        profile = env.GetProjectOption("custom_ap6256_wifi_asset_profile", default=None)
    return profile or "current_repo"


def get_profile(profile_name: str) -> dict:
    try:
        return WIFI_ASSET_PROFILES[profile_name]
    except KeyError as exc:
        valid = ", ".join(WIFI_ASSET_PROFILES.keys())
        raise ValueError(f"Unknown AP6256 Wi-Fi asset profile '{profile_name}'. Valid profiles: {valid}") from exc


def profile_assets(profile: dict) -> list[dict]:
    return list(profile["wifi"]) + list(COMMON_ASSETS)


def generate() -> None:
    project_dir = get_project_dir()
    profile_name = selected_profile_name()
    profile = get_profile(profile_name)
    assets = profile_assets(profile)

    manifest_header = project_dir / "include" / "generated" / "ap6256_assets_manifest.h"
    assembly_source = project_dir / "src" / "generated" / "ap6256_assets_data.S"

    header_lines = [
        "#ifndef AP6256_ASSETS_MANIFEST_H",
        "#define AP6256_ASSETS_MANIFEST_H",
        "",
        "#include <stdint.h>",
        "",
        "#define AP6256_ASSET_MANIFEST_VERSION 2U",
        f'#define AP6256_ASSET_WIFI_PROFILE_NAME "{c_string(profile_name)}"',
        f'#define AP6256_ASSET_WIFI_PROFILE_DESCRIPTION "{c_string(profile["description"])}"',
        f'#define AP6256_ASSET_WIFI_PROFILE_SOURCE_URL "{c_string(profile["source_url"])}"',
        f'#define AP6256_ASSET_WIFI_PROFILE_FIRMWARE_VERSION_HINT "{c_string(profile["firmware_version_hint"])}"',
        f"#define AP6256_ASSET_WIFI_PROFILE_DEFAULT_GENERIC_NVRAM {int(profile['default_generic_nvram'])}U",
        "",
    ]

    asm_lines = [
        ".syntax unified",
        "",
    ]

    for asset in assets:
        path = project_dir / asset["relative_path"]
        if not path.exists():
            profile_hint = (
                " Run 'python scripts\\validate_ap6256_asset_profiles.py --fetch' "
                "to fetch optional Raspberry Pi candidate assets."
            )
            raise FileNotFoundError(f"Missing AP6256 asset for profile '{profile_name}': {path}.{profile_hint}")

        file_name = path.name
        size = path.stat().st_size
        sha = sha256_hex(path)
        symbol = asset["symbol"]
        macro = asset["macro"]
        absolute_asset = path.resolve().as_posix()

        header_lines.extend(
            [
                f'#define AP6256_ASSET_{macro}_FILENAME "{c_string(file_name)}"',
                f"#define AP6256_ASSET_{macro}_SIZE {size}U",
                f'#define AP6256_ASSET_{macro}_SHA256 "{sha}"',
                f'#define AP6256_ASSET_{macro}_SOURCE_URL "{c_string(asset["source_url"])}"',
                f'#define AP6256_ASSET_{macro}_VERSION_HINT "{c_string(asset["version_hint"])}"',
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


if env is not None or __name__ == "__main__":
    generate()
