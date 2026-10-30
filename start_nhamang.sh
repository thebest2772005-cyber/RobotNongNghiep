#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VIDEO_ARGS=()

HAS_CAM_FLAG=false
for arg in "$@"; do
  if [ "$arg" = "--cam" ] || [ "$arg" = "--camera" ]; then
    HAS_CAM_FLAG=true
    break
  fi
done

if [ "$HAS_CAM_FLAG" = false ] && [ -f "$PROJECT_DIR/nhamang_20261025_030235.mp4" ]; then
  VIDEO_ARGS=(--video "$PROJECT_DIR/nhamang_20261025_030235.mp4")
fi

exec bash "$PROJECT_DIR/start.sh" \
  --mode ai \
  "${VIDEO_ARGS[@]}" \
  --model "$PROJECT_DIR/models/nhamang_batden_seg.pt" \
  --profile "$PROJECT_DIR/profiles/nhamang_batden_profile.json" \
  --device auto \
  --imgsz 416 \
  --conf 0.25 \
  "$@"

