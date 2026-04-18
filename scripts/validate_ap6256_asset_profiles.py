from __future__ import annotations

import argparse
import hashlib
import re
import sys
import urllib.request
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
RPI_BASE_URL = (
    "https://raw.githubusercontent.com/RPi-Distro/firmware-nonfree/"
    "bullseye/debian/config/brcm80211/brcm"
)
RPI_PROFILE_DIR = REPO_ROOT / "assets" / "ap6256" / "profiles" / "rpi_7_84_17_1"
MANJARO_PROFILE_DIR = REPO_ROOT / "assets" / "ap6256" / "profiles" / "manjaro_ap6256_2020_02"

PROFILES = {
    "current_repo": [
        "assets/ap6256/brcmfmac43456-sdio.bin",
        "assets/ap6256/brcmfmac43456-sdio.clm_blob",
        "assets/ap6256/brcmfmac43456-sdio.txt",
        "assets/ap6256/reference/nvram_ap6256.txt",
    ],
    "rpi_7_84_17_1_ap6256_nvram": [
        "assets/ap6256/profiles/rpi_7_84_17_1/brcmfmac43456-sdio.bin",
        "assets/ap6256/profiles/rpi_7_84_17_1/brcmfmac43456-sdio.clm_blob",
        "assets/ap6256/profiles/rpi_7_84_17_1/brcmfmac43456-sdio.txt",
        "assets/ap6256/reference/nvram_ap6256.txt",
    ],
    "rpi_7_84_17_1_coherent": [
        "assets/ap6256/profiles/rpi_7_84_17_1/brcmfmac43456-sdio.bin",
        "assets/ap6256/profiles/rpi_7_84_17_1/brcmfmac43456-sdio.clm_blob",
        "assets/ap6256/profiles/rpi_7_84_17_1/brcmfmac43456-sdio.txt",
    ],
    "current_bin_rpi_clm": [
        "assets/ap6256/brcmfmac43456-sdio.bin",
        "assets/ap6256/profiles/rpi_7_84_17_1/brcmfmac43456-sdio.clm_blob",
        "assets/ap6256/brcmfmac43456-sdio.txt",
        "assets/ap6256/reference/nvram_ap6256.txt",
    ],
    "current_bin_rpi_nvram": [
        "assets/ap6256/brcmfmac43456-sdio.bin",
        "assets/ap6256/brcmfmac43456-sdio.clm_blob",
        "assets/ap6256/profiles/rpi_7_84_17_1/brcmfmac43456-sdio.txt",
    ],
    "manjaro_ap6256_2020_02_coherent": [
        "assets/ap6256/profiles/manjaro_ap6256_2020_02/brcmfmac43456-sdio.bin",
        "assets/ap6256/profiles/manjaro_ap6256_2020_02/brcmfmac43456-sdio.clm_blob",
        "assets/ap6256/profiles/manjaro_ap6256_2020_02/brcmfmac43456-sdio.AP6256.txt",
    ],
}

RPI_FILES = (
    "brcmfmac43456-sdio.bin",
    "brcmfmac43456-sdio.clm_blob",
    "brcmfmac43456-sdio.txt",
)


def sha256_hex(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def fetch_rpi_assets() -> None:
    RPI_PROFILE_DIR.mkdir(parents=True, exist_ok=True)
    for filename in RPI_FILES:
        target = RPI_PROFILE_DIR / filename
        url = f"{RPI_BASE_URL}/{filename}"
        if target.exists() and target.stat().st_size > 0:
            print(f"fetch: keep {target.relative_to(REPO_ROOT)}")
            continue
        print(f"fetch: {url}")
        with urllib.request.urlopen(url, timeout=60) as response:
            data = response.read()
        if not data:
            raise RuntimeError(f"downloaded empty asset: {url}")
        target.write_bytes(data)


def load_manifest() -> dict[str, str]:
    path = REPO_ROOT / "include" / "generated" / "ap6256_assets_manifest.h"
    if not path.exists():
        return {}
    values: dict[str, str] = {}
    define_re = re.compile(r"^#define\s+(\S+)\s+(.+)$")
    for line in path.read_text(encoding="utf-8").splitlines():
        match = define_re.match(line)
        if match is None:
            continue
        key, value = match.groups()
        values[key] = value.strip().strip('"')
    return values


def validate_profile(profile: str, manifest: dict[str, str]) -> int:
    missing = 0
    print(f"profile: {profile}")
    for rel in PROFILES[profile]:
        path = REPO_ROOT / rel
        if not path.exists():
            print(f"  MISSING {rel}")
            missing += 1
            continue
        print(f"  {rel} size={path.stat().st_size} sha256={sha256_hex(path)}")

    manifest_profile = manifest.get("AP6256_ASSET_WIFI_PROFILE_NAME")
    if manifest_profile == profile:
        print("  manifest: selected profile matches")
        generated_checks = (
            ("WIFI_FIRMWARE", "assets/ap6256/brcmfmac43456-sdio.bin"),
            ("WIFI_CLM_BLOB", "assets/ap6256/brcmfmac43456-sdio.clm_blob"),
            ("WIFI_NVRAM", "assets/ap6256/brcmfmac43456-sdio.txt"),
            ("WIFI_NVRAM_REFERENCE", "assets/ap6256/reference/nvram_ap6256.txt"),
        )
        for macro, fallback in generated_checks:
            filename = manifest.get(f"AP6256_ASSET_{macro}_FILENAME")
            size = manifest.get(f"AP6256_ASSET_{macro}_SIZE", "").rstrip("U")
            sha = manifest.get(f"AP6256_ASSET_{macro}_SHA256")
            if not filename or not size or not sha:
                print(f"  manifest: missing {macro} metadata")
                missing += 1
                continue
            candidates = [REPO_ROOT / rel for rel in PROFILES[profile] if Path(rel).name == filename]
            if not candidates:
                candidates = [REPO_ROOT / fallback]
            path = candidates[0]
            if path.exists() and (str(path.stat().st_size) != size or sha256_hex(path) != sha):
                print(f"  manifest: {macro} mismatch")
                missing += 1
    elif manifest_profile:
        print(f"  manifest: currently generated for {manifest_profile}")
    else:
        print("  manifest: not generated yet")

    return missing


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate AP6256 Wi-Fi asset profiles.")
    parser.add_argument("--fetch", action="store_true", help="download optional Raspberry Pi candidate assets")
    parser.add_argument("--profile", action="append", choices=sorted(PROFILES), help="profile to validate")
    parser.add_argument("--all", action="store_true", help="validate every known profile")
    args = parser.parse_args()

    if args.fetch:
        fetch_rpi_assets()

    profiles = args.profile or []
    if args.all or not profiles:
        profiles = list(PROFILES)

    manifest = load_manifest()
    failures = 0
    for profile in profiles:
        failures += validate_profile(profile, manifest)

    if failures:
        print(f"validation: FAIL missing_or_mismatch={failures}")
        return 1
    print("validation: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
