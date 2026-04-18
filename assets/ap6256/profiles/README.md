# AP6256 Wi-Fi Asset Profiles

The normal firmware build uses `current_repo`, which keeps the repository
`brcmfmac43456-sdio.bin`, CLM blob, and AP6256 module NVRAM default.

Candidate BCM43456 profiles are selected at build time:

```powershell
$env:AP6256_WIFI_ASSET_PROFILE="rpi_7_84_17_1_ap6256_nvram"
pio run -e h743vitx -j1
```

Known profile order for HIL testing:

1. `current_repo`
2. `rpi_7_84_17_1_ap6256_nvram`
3. `rpi_7_84_17_1_coherent`
4. `current_bin_rpi_clm`

Isolation profile used when the reset behavior differs between AP6256 and
coherent Raspberry Pi NVRAM:

- `current_bin_rpi_nvram`
- `manjaro_ap6256_2020_02_coherent`

Fetch and validate optional Raspberry Pi candidate assets with:

```powershell
python scripts\validate_ap6256_asset_profiles.py --fetch --all
```

`wifi_info` prints the selected profile, source, version hint, asset hashes, and
whether the profile defaults to AP6256 module NVRAM or profile/generic NVRAM.
