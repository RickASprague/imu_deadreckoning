#!/usr/bin/env bash
#
# screenshot.sh — grab the connected Android device's screen into
# resources/screenshot.png (repo-relative, regardless of where you run it).
#
# One framebuffer read via `adb exec-out screencap` — no on-device encoder, so
# it does not perturb a running app the way `screenrecord` does.
#
#   scripts/screenshot.sh              # -> resources/screenshot.png
#   scripts/screenshot.sh foo.png      # -> resources/foo.png
#   scripts/screenshot.sh -s SERIAL    # target a specific device
#
set -euo pipefail

# --- locate the repo + adb, independent of cwd ------------------------------
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
OUT_DIR="$REPO_DIR/resources"

# adb: prefer PATH, fall back to the SDK location used on this box.
ADB="$(command -v adb || true)"
[ -z "$ADB" ] && [ -x "$HOME/Android/Sdk/platform-tools/adb" ] \
    && ADB="$HOME/Android/Sdk/platform-tools/adb"
[ -z "$ADB" ] && { echo "screenshot: adb not found (PATH or ~/Android/Sdk)" >&2; exit 1; }

# --- args: optional -s SERIAL, optional output basename ---------------------
SERIAL=()
NAME="screenshot.png"
while [ $# -gt 0 ]; do
    case "$1" in
        -s) SERIAL=(-s "${2:?-s needs a serial}"); shift 2 ;;
        -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
        *)  NAME="$1"; shift ;;
    esac
done

mkdir -p "$OUT_DIR"
OUT="$OUT_DIR/$NAME"

# exec-out gives a raw binary stream; plain `shell screencap -p` mangles the PNG
# via pty newline translation on some hosts.
"$ADB" "${SERIAL[@]}" exec-out screencap -p > "$OUT"

# screencap emits nothing on a bad grab (e.g. secure screen) -> catch empty file
[ -s "$OUT" ] || { echo "screenshot: empty capture (device locked / secure screen?)" >&2; rm -f "$OUT"; exit 1; }

echo "screenshot: $(wc -c < "$OUT") bytes  ->  ${OUT#"$REPO_DIR"/}"
