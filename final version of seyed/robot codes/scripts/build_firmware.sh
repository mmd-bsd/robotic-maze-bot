#!/usr/bin/env bash
# build_firmware.sh -- Build the SEYED STM32 firmware from the command line.
#
# Compiles and LINKS the real firmware project (final version of seyed/firmware)
# for STM32G031G8Ux using the same ARM Compiler 5 that Keil MDK uses, and prints
# the Keil "Program Size" line plus a flashable .hex.
#
# Why: the RAM budget is the hard constraint on this project (8 KB total).
# This lets you check "does it still fit?" in one command, without opening Keil.
#
# Usage (from robot codes/):
#   bash scripts/build_firmware.sh                 # the mission firmware
#   USE_TELEMETRY=1 bash scripts/build_firmware.sh # + the M1 bench telemetry dump
#
# Override the Keil location if needed:
#   KEIL=/c/Keil_v5/ARM/ARMCC/bin bash scripts/build_firmware.sh
#
# Output: build/firmware/ (gitignored) -- .o objects, seyed.axf, seyed.hex,
# link map, and the scatter file used.
#
# NOTE ON PATHS: every path in this repo contains spaces ("Robotic fle 2022",
# "final version of seyed"). Sources and flags are therefore held in bash
# ARRAYS and expanded as "${arr[@]}" -- never joined into a string and
# re-split, which is what breaks word-splitting.
set -u

# ---- toolchain ----
KEIL="${KEIL:-/c/Keil_v5/ARM/ARMCC/bin}"
ARMCC="$KEIL/armcc.exe"
ARMASM="$KEIL/armasm.exe"
ARMLINK="$KEIL/armlink.exe"
FROMELF="$KEIL/fromelf.exe"
for t in "$ARMCC" "$ARMASM" "$ARMLINK" "$FROMELF"; do
    if [ ! -x "$t" ]; then
        echo "ERROR: not found: $t" >&2
        echo "Set KEIL=/path/to/ARM/ARMCC/bin and retry." >&2
        exit 2
    fi
done

# ---- layout ----
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # .../robot codes/scripts
SOLVER="$(cd "$HERE/.." && pwd)"                       # .../robot codes
SEYED="$(cd "$SOLVER/.." && pwd)"                      # .../final version of seyed
FW="$SEYED/firmware"
OUT="$SOLVER/build/firmware"

CPU="Cortex-M0+"

MODE="brain-driven (decision core + solver library)"
if [ "${USE_TELEMETRY:-0}" = "1" ]; then
    MODE="$MODE + USE_MAZE_TELEMETRY ON (M1 bench build: 125 Hz sensor/encoder dump)"
fi
echo "=============================================="
echo " SEYED firmware build -- $MODE"
echo " Project : $FW"
echo " Output  : $OUT"
echo "=============================================="

# ---- flags (must match Source.uvprojx) ----
CFLAGS=(--c99 -c "--cpu=$CPU" -O3 -DUSE_HAL_DRIVER -DSTM32G031xx)
CFLAGS+=("-I$FW/Core/Inc")
CFLAGS+=("-I$FW/Drivers/STM32G0xx_HAL_Driver/Inc")
CFLAGS+=("-I$FW/Drivers/STM32G0xx_HAL_Driver/Inc/Legacy")
CFLAGS+=("-I$FW/Drivers/CMSIS/Device/ST/STM32G0xx/Include")
CFLAGS+=("-I$FW/Drivers/CMSIS/Include")
CFLAGS+=("-I$SOLVER/inc")
# There is no USE_MAZE_SOLVER switch any more: the brain IS the decision maker,
# the legacy explorer was deleted outright, and main.c no longer tests the flag.
# M1 bench telemetry: sensor masks + encoder counts over the Bluetooth link.
# Adds ~72 B of RAM (tlm_ms + a 64 B pending line).  It also arms the 5 s
# bring-up pause, so USE_TELEMETRY=1 is the SUPERVISED build -- the plain build
# drives on without stopping.
[ "${USE_TELEMETRY:-0}" = "1" ] && CFLAGS+=(-DUSE_MAZE_TELEMETRY)

# ---- source list: exactly what Source.uvprojx compiles ----
# Deliberately NOT a glob of HAL_Driver/Src, which also holds *_template.c
# files that belong to no target.
SRCS=()
for b in main stm32g0xx_it stm32g0xx_hal_msp Hardware LSM6DS3TR Serial system_stm32g0xx; do
    SRCS+=("$FW/Core/Src/$b.c")
done
for b in stm32g0xx_hal_i2c stm32g0xx_hal_i2c_ex stm32g0xx_hal_rcc \
         stm32g0xx_hal_rcc_ex stm32g0xx_ll_rcc stm32g0xx_hal_flash \
         stm32g0xx_hal_flash_ex stm32g0xx_hal_gpio stm32g0xx_hal_dma \
         stm32g0xx_hal_dma_ex stm32g0xx_ll_dma stm32g0xx_hal_pwr \
         stm32g0xx_hal_pwr_ex stm32g0xx_hal_cortex stm32g0xx_hal \
         stm32g0xx_hal_exti stm32g0xx_hal_tim stm32g0xx_hal_tim_ex; do
    SRCS+=("$FW/Drivers/STM32G0xx_HAL_Driver/Src/$b.c")
done
# The solver library is compiled in place -- single source of truth shared with
# run_maze.py / build_all.ps1.  All of these now cost RAM as well: brain.c owns
# the MazeGraph + MazeRobot statically, so nothing is dead-stripped by the
# linker.  main.c reaches brain.c through inc/brain.h.
for b in maze_graph maze_robot maze_explore maze_proof maze_fastrun maze_solver brain; do
    SRCS+=("$SOLVER/src/$b.c")
done

mkdir -p "$OUT"
rm -f "$OUT"/*.o "$OUT"/*.axf "$OUT"/*.map "$OUT"/*.hex

# ---- compile ----
echo
echo "--- compile ---"
fail=0
for f in "${SRCS[@]}"; do
    b=$(basename "$f" .c)
    msg=$("$ARMCC" "${CFLAGS[@]}" "$f" -o "$OUT/$b.o" 2>&1)
    if [ $? -ne 0 ]; then
        echo "  [FAIL] $b.c"; echo "$msg" | sed 's/^/         /' | head -20
        fail=$((fail+1))
    elif [ -n "$msg" ]; then
        echo "  [WARN] $b.c"; echo "$msg" | sed 's/^/         /' | head -10
    else
        printf "  [ ok ] %s.c\n" "$b"
    fi
done

# ---- assemble startup ----
msg=$("$ARMASM" "--cpu=$CPU" "$FW/MDK-ARM/startup_stm32g031xx.s" -o "$OUT/startup.o" 2>&1)
if [ $? -ne 0 ]; then
    echo "  [FAIL] startup_stm32g031xx.s"; echo "$msg" | sed 's/^/         /' | head -20
    fail=$((fail+1))
else
    echo "  [ ok ] startup_stm32g031xx.s"
fi

if [ $fail -ne 0 ]; then
    echo
    echo "BUILD FAILED: $fail file(s) with errors -- not linking."
    exit 1
fi

# ---- scatter (same memory map Keil auto-generates for this part) ----
cat > "$OUT/scatter.sct" <<'EOF'
LR_IROM1 0x08000000 0x00010000  {          ; 64 KB flash
  ER_IROM1 0x08000000 0x00010000  {
   *.o (RESET, +First)
   *(InRoot$$Sections)
   .ANY (+RO)
  }
  RW_IRAM1 0x20000000 0x00002000  {        ; 8 KB RAM
   .ANY (+RW +ZI)
  }
}
EOF

# ---- link ----
echo
echo "--- link ---"
OBJS=()
for f in "$OUT"/*.o; do OBJS+=("$f"); done

LINKOUT=$("$ARMLINK" "--cpu=$CPU" --strict --remove --scatter "$OUT/scatter.sct" \
    --map --info=sizes --info=totals "${OBJS[@]}" -o "$OUT/seyed.axf" 2>&1)
echo "$LINKOUT" | tr -d '\r' | grep -Ei "error|warning" | head -10
echo "$LINKOUT" | tr -d '\r' | grep -E "Total (RO|RW|ROM) " | sed 's/^ */  /'

# AC5 armlink's --map takes NO argument and creates NO file -- it prints the
# memory map to stdout, and --map=<file> is rejected with L3916U.  So persist
# stdout here, otherwise the "Map:" path printed at the end is a lie (it was,
# until this line existed).  Written before the .axf check so the path in the
# failure message below is real too.
printf '%s\n' "$LINKOUT" > "$OUT/seyed.map"

if [ ! -f "$OUT/seyed.axf" ]; then
    echo
    echo "LINK FAILED -- see $OUT/seyed.map"
    exit 1
fi

# ---- flashable image ----
"$FROMELF" --i32combined "$OUT/seyed.axf" -o "$OUT/seyed.hex" >/dev/null 2>&1

# ---- program size (Keil's own format) ----
# fromelf -z prints columns: Code(inc.data) RO Data RW Data ZI Data Debug Name.
# It is used ONLY for the per-section breakdown in the Program Size line.
# NOTE: the row is tagged "(uncompressed)" only when the image carries
# compressed data.  This build does not, so the tag is absent and an
# "(uncompressed)" grep matches nothing -- which is what silently zeroed the RAM
# figure below until 2026-09-23.  Match the image row by name instead.
SIZES=$("$FROMELF" --text -z "$OUT/seyed.axf" 2>/dev/null \
        | tr -d '\r' | grep -F "seyed.axf" | head -1)
CODE=$(echo "$SIZES" | awk '{print $1}')
RO=$(echo   "$SIZES" | awk '{print $3}')
RW=$(echo   "$SIZES" | awk '{print $4}')
ZI=$(echo   "$SIZES" | awk '{print $5}')

echo
echo "=============================================="
echo " Program Size: Code=$CODE RO-data=$RO RW-data=$RW ZI-data=$ZI"
echo "=============================================="

# The LINKER's totals are authoritative for the fit check, not fromelf's --
# they are what Keil reports, and they cannot be knocked out by a tool-output
# format change.  Lines read:
#   "Total RO  Size (Code + RO Data)                40668 (  39.71kB)"
#   "Total RW  Size (RW Data + ZI Data)              6832 (   6.67kB)"
#   "Total ROM Size (Code + RO Data + RW Data)      40904 (  39.95kB)"
# No '=', so pull the FIRST bare integer on each line (the kB figure is not bare).
_linker_total() {
    echo "$LINKOUT" | tr -d '\r' \
        | awk -v pat="$1" '$0 ~ pat { for (i=1;i<=NF;i++) if ($i ~ /^[0-9]+$/) { print $i; exit } }'
}
TOTAL_ROM=$(_linker_total "Total ROM Size")
TOTAL_RAM=$(_linker_total "Total RW  Size")
# Fall back to fromelf's parts only if the linker line is missing entirely.
[ -z "$TOTAL_ROM" ] && TOTAL_ROM=$(( CODE + RO + RW ))
[ -z "$TOTAL_RAM" ] && TOTAL_RAM=$(( RW + ZI ))

if [ -z "$TOTAL_RAM" ] || [ -z "$TOTAL_ROM" ]; then
    echo " *** could not read the linker totals -- fit check NOT performed ***" >&2
    exit 1
fi

printf " FLASH : %6d / %6d bytes  (%2d%% used, %d free)\n" \
       "$TOTAL_ROM" 65536 $(( TOTAL_ROM * 100 / 65536 )) $(( 65536 - TOTAL_ROM ))
printf " RAM   : %6d / %6d bytes  (%2d%% used, %d free)\n" \
       "$TOTAL_RAM" 8192 $(( TOTAL_RAM * 100 / 8192 )) $(( 8192 - TOTAL_RAM ))

if [ "$TOTAL_RAM" -gt 8192 ]; then
    echo
    echo " *** RAM OVERFLOW by $(( TOTAL_RAM - 8192 )) bytes -- will not fit. ***"
fi
echo
echo " Hex: $OUT/seyed.hex"
echo " Map: $OUT/seyed.map"
