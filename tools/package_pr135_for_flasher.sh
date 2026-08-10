#!/usr/bin/env bash
# Package a locally-built firmware into the web-flasher as tag "pr-135".
# Mirrors .github/workflows/release.yml. USB (ESP Web Tools) artifacts are
# always produced. The BLE-OTA detools patch is added only if detools is
# importable (it needs a C compiler to install on this machine).
set -euo pipefail

TAG="pr-135"
ENV="waveshare-esp32s3-touch-amoled-164"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="$REPO/.pio/build/$ENV"
OUT="$REPO/tools/web-flasher/firmware/$TAG"
BASE="smart-grind-by-weight-$TAG"

if [ -x "$REPO/tools/venv/Scripts/python.exe" ]; then
  VPY="$REPO/tools/venv/Scripts/python.exe"
else
  VPY="$REPO/tools/venv/bin/python"
fi

for f in firmware.bin bootloader.bin partitions.bin; do
  [ -f "$BUILD/$f" ] || { echo "ERROR: $BUILD/$f missing - build did not complete"; exit 1; }
done

mkdir -p "$OUT"
cp "$BUILD/firmware.bin"   "$OUT/$BASE.bin"
cp "$BUILD/bootloader.bin" "$OUT/$BASE-bootloader.bin"
cp "$BUILD/partitions.bin" "$OUT/$BASE-partitions.bin"
head -c 8192 /dev/zero > "$OUT/blank_8KB.bin"

# Manifest for ESP Web Tools (USB) - always produced
cat > "$OUT/$BASE.manifest.json" <<EOF
{
  "name": "Smart Grind By Weight",
  "version": "$TAG",
  "home_assistant_domain": "grinder",
  "new_install_skip_erase": true,
  "builds": [
    {
      "chipFamily": "ESP32-S3",
      "parts": [
        { "path": "$BASE-bootloader.bin", "offset": 0 },
        { "path": "$BASE-partitions.bin", "offset": 32768 },
        { "path": "blank_8KB.bin", "offset": 57344 },
        { "path": "$BASE.bin", "offset": 3276800 }
      ]
    }
  ]
}
EOF

# BLE OTA patch - optional, requires detools
OTA_LINE=""
if "$VPY" -c "import detools" 2>/dev/null; then
  : > "$OUT/empty.bin"
  "$VPY" -c "import detools;detools.create_patch(open('$OUT/empty.bin','rb'),open('$OUT/$BASE.bin','rb'),open('$OUT/$BASE-web-ota.bin','wb'),compression='heatshrink')"
  rm -f "$OUT/empty.bin"
  OTA_LINE="    \"ota\": \"firmware/$TAG/$BASE-web-ota.bin\","
  echo "[OK] detools available - BLE OTA patch created"
else
  echo "[SKIP] detools not installed - USB flashing only for PR #135 (BLE OTA omitted)"
fi

# index.json - pr-135 first (default selection), stock rc.6 kept for rollback
cat > "$REPO/tools/web-flasher/firmware/index.json" <<EOF
[
  {
    "tag": "$TAG",
    "version": "$(sed -n 's/.*BUILD_FIRMWARE_VERSION "\([^"]*\)".*/\1/p' "$REPO/src/config/build_info.h")",
    "display": "PR #135 - time mode without weight sensor",
    "prerelease": false,
$OTA_LINE
    "manifest": "firmware/$TAG/$BASE.manifest.json"
  },
  {
    "tag": "v1.3.0-rc.6",
    "version": "1.3.0-rc.6",
    "display": "v1.3.0-rc.6 (stock - rollback)",
    "prerelease": false,
    "ota": "firmware/smart-grind-by-weight-v1.3.0-rc.6-web-ota.bin",
    "manifest": "firmware/smart-grind-by-weight-v1.3.0-rc.6.manifest.json"
  }
]
EOF

echo "OK: packaged $TAG"
ls -la "$OUT"
