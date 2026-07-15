#!/usr/bin/env bash
# Rebuild every committed design fixture (*.qrd) from its CLI build script (scripts/*.cli) through
# the real qtrocket-cli. Each script ends with `checkdesign`, so a rebuild that produces overlaps or
# air gaps fails here rather than landing in the tree. xl75_multi_pokethrough.qrd is deliberately
# NOT script-built: it is the frozen self-intersecting offender fixture (see its XML comment).
#
# Usage: ./regenerate.sh [path-to-qtrocket-cli]     (default: ../../../build/cli/qtrocket-cli)
#
# After regenerating, refresh the placement baseline:
#   QTROCKET_REGEN_PLACEMENT_BASELINE=1 <build>/tests/integration_tests \
#      --gtest_filter='PlacementInvariance.RegenerateBaseline'
set -euo pipefail
cd "$(dirname "$0")"
CLI="${1:-../../../build/cli/qtrocket-cli}"

if [[ ! -x "$CLI" ]]; then
   echo "qtrocket-cli not found at '$CLI' (build it, or pass its path)" >&2
   exit 1
fi

log="$(mktemp)"
trap 'rm -f "$log" log.txt' EXIT

for script in scripts/*.cli; do
   stem="$(basename "$script" .cli)"
   {
      echo "loaddb ../motors/estes_small.qmd"
      echo "loadmotors ../../../data/Aerotech.rse"
      cat "$script"
      echo "savedesign ${stem}.qrd"
      echo "quit"
   } | "$CLI" >"$log" 2>&1 || true   # CLI exits 1 on any ERR; the grep below reports the detail

   if grep -q '^ERR' "$log"; then
      echo "FAILED: ${stem}" >&2
      grep '^ERR' "$log" >&2
      exit 1
   fi
   echo "rebuilt ${stem}.qrd"
done
