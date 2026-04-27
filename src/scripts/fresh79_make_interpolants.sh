#!/bin/bash
set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

OUT_DIR=/mnt/data/checkpoints/usv_rl/fresh79_interpolants
mkdir -p "$OUT_DIR"

LEFT=/mnt/data/checkpoints/usv_rl/fresh74_best.pt
RIGHT_A=/mnt/data/checkpoints/usv_rl/fresh78_best.pt
RIGHT_B=/mnt/data/checkpoints/usv_rl/fresh76_headon_focus.pt

if [[ ! -f "$LEFT" || ! -f "$RIGHT_A" || ! -f "$RIGHT_B" ]]; then
  echo "ERROR: one or more source checkpoints are missing"
  exit 1
fi

make_pair() {
  local left="$1"
  local right="$2"
  local prefix="$3"
  local template="$4"
  for alpha in 0.20 0.35 0.50 0.65 0.80; do
    local beta
    beta=$(python3 - <<PY
alpha = float('$alpha')
print(f"{1.0-alpha:.2f}")
PY
)
    local out="$OUT_DIR/${prefix}_a${alpha//./}.pt"
    python3 src/scripts/fresh79_blend_checkpoints.py \
      --checkpoint "$left:$beta" \
      --checkpoint "$right:$alpha" \
      --template "$template" \
      --label "$prefix alpha=$alpha" \
      --output "$out" >/tmp/$(basename "$out" .pt)_blend.json
  done
}

make_pair "$LEFT" "$RIGHT_A" fresh74_fresh78 "$RIGHT_A"
make_pair "$LEFT" "$RIGHT_B" fresh74_fresh76focus "$RIGHT_B"

# One simple 3-way EWA candidate to test whether a small head-on focus term helps.
python3 src/scripts/fresh79_blend_checkpoints.py \
  --checkpoint "$LEFT:0.45" \
  --checkpoint "$RIGHT_A:0.40" \
  --checkpoint "$RIGHT_B:0.15" \
  --template "$RIGHT_A" \
  --label "fresh74_045_fresh78_040_fresh76focus_015" \
  --output "$OUT_DIR/fresh74_fresh78_fresh76focus_ewa.pt" >/tmp/fresh79_ewa_blend.json

ls -1 "$OUT_DIR"
