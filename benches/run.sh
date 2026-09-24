#!/usr/bin/env bash
# Benchmark harness: times `dotv --dump` on every bench graph, GD_TIMING per phase.
# Usage: ./benches/run.sh [release-binary]
set -u
BIN=${1:-target/release/dotv}
DIR="$(cd "$(dirname "$0")" && pwd)"
GRAPHS="$DIR/graphs"
OUT="$DIR/results"
mkdir -p "$OUT"
LABEL=$(date +%s)
SUMMARY="$OUT/summary-$LABEL.txt"
for g in "$GRAPHS"/*.dot; do
  name=$(basename "$g" .dot)
  # wall time, 3 runs, report best
  best=999
  for i in 1 2 3; do
    s=$( { /usr/bin/time -f "%e" "$BIN" --dump "$g" > /dev/null; } 2>&1 )
    awk -v a="$best" -v b="$s" 'BEGIN{print (b<a)?b:a}' > /tmp/best
    best=$(cat /tmp/best)
  done
  echo "== $name: ${best}s"
  # one timing-instrumented run
  GD_TIMING=1 "$BIN" --dump "$g" > /dev/null 2> "$OUT/timing-$name.txt"
  grep '^\[timing\]' "$OUT/timing-$name.txt" | sed 's/^/   /'
  echo "== $name: ${best}s" >> "$SUMMARY"
done
echo "summary -> $SUMMARY"
