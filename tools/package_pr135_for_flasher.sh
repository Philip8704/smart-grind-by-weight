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

# BLE OTA patch - needs detools, which does not install on this machine's Windows venv
# because it builds C extensions. It lives in a WSL venv instead, so try that as well.
# Without the patch a phone cannot flash at all: ESP Web Tools needs USB, and BLE OTA is
# the only path a phone has. Dropping silently to USB-only looks like a working package
# right up until someone tries to use it, so the fallback is worth the extra branch.
OTA_LINE=""
OTA_BUILT=0

make_patch() {  # $1 = command prefix that runs a python with detools
  : > "$OUT/empty.bin"
  if "$@" -c "import detools;detools.create_patch(open('$OUT/empty.bin','rb'),open('$OUT/$BASE.bin','rb'),open('$OUT/$BASE-web-ota.bin','wb'),compression='heatshrink')"; then
    rm -f "$OUT/empty.bin"
    return 0
  fi
  rm -f "$OUT/empty.bin"
  return 1
}

if "$VPY" -c "import detools" 2>/dev/null && make_patch "$VPY"; then
  OTA_BUILT=1
  echo "[OK] BLE OTA patch created (native venv)"
elif command -v wsl >/dev/null 2>&1 && \
     wsl -e bash -lc "~/dtenv/bin/python -c 'import detools'" >/dev/null 2>&1; then
  WSL_OUT="$(wsl -e wslpath -a "$(cygpath -w "$OUT" 2>/dev/null || echo "$OUT")" 2>/dev/null)"
  if [ -n "$WSL_OUT" ] && wsl -e bash -lc "cd '$WSL_OUT' && : > empty.bin && ~/dtenv/bin/python -c \"import detools;detools.create_patch(open('empty.bin','rb'),open('$BASE.bin','rb'),open('$BASE-web-ota.bin','wb'),compression='heatshrink')\" && rm -f empty.bin"; then
    OTA_BUILT=1
    echo "[OK] BLE OTA patch created (WSL detools)"
  fi
fi

if [ "$OTA_BUILT" = "1" ]; then
  OTA_LINE="    \"ota\": \"firmware/$TAG/$BASE-web-ota.bin\","
else
  # Remove rather than leave behind: a patch from an earlier build is named exactly like
  # a current one, and index.json omitting it is the only thing standing between that
  # file and someone flashing last week's firmware believing it is this one.
  rm -f "$OUT/$BASE-web-ota.bin"
  echo "[WARN] detools unavailable in venv and WSL - USB flashing only, stale OTA patch removed"
  echo "       Phones cannot flash over USB; install detools to restore phone flashing."
fi

# The build number is what distinguishes one local package from the next - the version
# string only moves on a release - so it goes in the label the flasher shows. Picking it
# out of the generated header rather than tracking it by hand keeps the page honest
# about which build is actually being served.
BUILD_NO="$(sed -n 's/.*define BUILD_NUMBER \([0-9]*\).*/\1/p' "$REPO/include/git_info.h" | head -1)"
[ -n "$BUILD_NO" ] || BUILD_NO="?"

# index.json - pr-135 first (default selection), stock rc.6 kept for rollback
cat > "$REPO/tools/web-flasher/firmware/index.json" <<EOF
[
  {
    "tag": "$TAG",
    "version": "$(sed -n 's/.*BUILD_FIRMWARE_VERSION "\([^"]*\)".*/\1/p' "$REPO/src/config/build_info.h")",
    "display": "PR #135 fork - build $BUILD_NO",
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
