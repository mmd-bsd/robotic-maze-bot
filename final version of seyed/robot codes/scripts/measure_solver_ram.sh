#!/usr/bin/env bash
# measure_solver_ram.sh -- answer "where does the 8 KB go?"
#
# The SEYED firmware runs on 8 KB of RAM, and that is the hard constraint on
# every change: adding a struct field is a RAM-budget decision.  This script
# builds the real firmware once and reports
#
#   1. the total RAM the linker actually placed, vs the 8 KB budget
#   2. a per-object breakdown, largest first -- so the cost of a change is
#      attributable to a file rather than to the image as a whole
#
# HISTORY.  Until 2026-09-23 this script measured "solver dormant vs solver
# activated": it patched a COPY of main.c to include maze_hal.h and call
# maze_hal_init(), and compared the two links.  That comparison no longer
# exists.  The brain is the firmware's decision maker unconditionally -- the
# legacy explorer was deleted outright, with no #ifndef guard and no fallback
# build -- so there is no dormant build left to compare against, and the seam
# it patched (maze_hal.h) is gone.  It builds once now, and the answer to "does
# it fit?" is a single number rather than a delta.
#
# Core/Src/main.c is NEVER modified -- nothing is patched any more.
#
# Usage (from robot codes/):
#   bash scripts/measure_solver_ram.sh
#
# The build switches (USE_MAZE_TELEMETRY / USE_MAZE_HEALTH / HEALTH_ONLY) live in
# main.c's BUILD SWITCHES block and are read from there by the build below.  They
# are echoed at the top of this report on purpose: they move the RAM figure by
# ~80 B and the flash figure by ~2 KB, so a size with no mode attached cannot be
# compared against an older measurement.
#
# Requires the same Keil ARMCC 5 toolchain as build_firmware.sh.

set -u

KEIL="${KEIL:-/c/Keil_v5/ARM/ARMCC/bin}"
ARMLINK="$KEIL/armlink.exe"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOLVER="$(cd "$HERE/.." && pwd)"
OUT="$SOLVER/build/firmware"
MAP="$OUT/seyed.map"

RAM_TOTAL=8192

[ -x "$ARMLINK" ] || { echo "ERROR: not found: $ARMLINK (set KEIL=...)" >&2; exit 2; }

# --- 1. build once ----------------------------------------------------------
echo "### Building the firmware ###"
if ! bash "$HERE/build_firmware.sh" >"$OUT/measure.log" 2>&1; then
    tail -20 "$OUT/measure.log"
    echo "build failed -- full log: $OUT/measure.log" >&2
    exit 1
fi
grep -E '^ (FLASH|RAM) :' "$OUT/measure.log" | sed 's/^/    /'
# Which switches produced those numbers -- see the usage note at the top.
grep -m1 'Switches, from main.c' "$OUT/measure.log" | sed 's/^/    /'

[ -f "$MAP" ] || { echo "ERROR: no link map at $MAP" >&2; exit 1; }

# --- 2. totals and breakdown from the map ----------------------------------
# The map's size table has columns
#     Code (inc. data)  RO Data  RW Data  ZI Data  Debug  Object Name
# i.e. "Code (inc. data)" is TWO numeric fields, so ZI Data is $5 and the
# object name is $7.  Only the block between the "Object Name" header and the
# "Object Totals" line is the per-object table; the sections after it are the
# library tables, which would double-count.
#
# Rows are printed as "ZI RW name" and sorted by ZI descending.
BODY=$(tr -d '\r' <"$MAP" | awk '
    /Object Name/ { inobj = 1; next }
    inobj && /Object Totals/ { inobj = 0 }
    inobj && NF >= 7 && $7 ~ /\.o$/ { printf "%d %d %s\n", $5, $4, $7 }
' | sort -k1,1 -n -r)

TOTAL_RAM=$(tr -d '\r' <"$MAP" \
            | awk '/Total RW  Size/ { for (i=1;i<=NF;i++) if ($i ~ /^[0-9]+$/) { print $i; exit } }')
TOTAL_ROM=$(tr -d '\r' <"$MAP" \
            | awk '/Total ROM Size/ { for (i=1;i<=NF;i++) if ($i ~ /^[0-9]+$/) { print $i; exit } }')

if [ -z "$TOTAL_RAM" ]; then
    echo "ERROR: could not read 'Total RW  Size' from $MAP" >&2
    exit 1
fi

echo
echo "=============================================="
printf " RAM  %5d / %5d bytes  (%2d%% used, %d free)\n" \
       "$TOTAL_RAM" "$RAM_TOTAL" $(( TOTAL_RAM * 100 / RAM_TOTAL )) $(( RAM_TOTAL - TOTAL_RAM ))
[ -n "$TOTAL_ROM" ] && printf " FLASH %4d / %5d bytes  (%2d%% used)\n" \
       "$TOTAL_ROM" 65536 $(( TOTAL_ROM * 100 / 65536 ))
echo "=============================================="
echo
echo " Largest RAM consumers (ZI = zero-init/.bss, RW = initialised):"
echo
printf "   %8s %8s   %s\n" "ZI" "RW" "object"
printf "   %8s %8s   %s\n" "--------" "--------" "------------------------------"
echo "$BODY" | head -12 | while read -r zi rw name; do
    printf "   %8s %8s   %s\n" "$zi" "$rw" "$name"
done
echo
echo " Reference points, so a change can be attributed:"
echo "     startup.o    stack (1024) + heap (512) reservation -- not variables"
echo "     brain.o      the decision core's MazeGraph + MazeRobot + the two plans"
echo "     main.o       the firmware's own buffers, incl. the replay strings"
echo "     maze_*.o     the solver library (flash only; its RAM is inside brain.o)"
echo
echo " For comparison, the legacy left-hand-rule firmware measured 7632 bytes"
echo " (560 free) while the solver sat dormant.  The brain-driven build above is"
echo " both smaller and doing the whole job -- see CHANGELOG.md, 2026-09-23."

echo
if [ "$TOTAL_RAM" -gt "$RAM_TOTAL" ]; then
    echo " *** OVER BUDGET by $(( TOTAL_RAM - RAM_TOTAL )) bytes ***"
    exit 1
fi
echo " FITS."
