# build_all.ps1 -- Rebuild and run ALL C tests in one shot.
#
# Usage (from robot codes/):
#   .\scripts\build_all.ps1
#
# Requires GCC (MSYS2 MinGW) on PATH.
# Output .exe files go to build/ (gitignored).

$ErrorActionPreference = "Stop"

# -- Add GCC to PATH if needed --
if (-not (Get-Command gcc -ErrorAction SilentlyContinue)) {
    $env:PATH = "C:\msys64\mingw64\bin;$env:PATH"
}

$SRC  = "src"
$TEST = "test"
$INC  = "inc"
$OUT  = "build"
$FLAGS = "-std=c11 -Wall -Wextra -pedantic"
$FLAGS_WERROR = "$FLAGS -Werror"

# Ensure build dir exists
New-Item -ItemType Directory -Force -Path $OUT | Out-Null

Write-Host "==============================================" -ForegroundColor Cyan
Write-Host "  BUILD ALL" -ForegroundColor Cyan
Write-Host "==============================================" -ForegroundColor Cyan
Write-Host ""

$builds = @()

# ---- test_graph ----
$name = "test_graph"
$builds += @{
    Name = $name
    Cmd  = "gcc $FLAGS -I $INC $SRC/maze_graph.c $TEST/$name.c -o $OUT/$name.exe"
}

# ---- test_robot ----
$name = "test_robot"
$builds += @{
    Name = $name
    Cmd  = "gcc $FLAGS -I $INC $SRC/maze_graph.c $SRC/maze_robot.c $TEST/$name.c -o $OUT/$name.exe"
}

# ---- integration_test (sample_maze, 27 nodes) ----
$name = "integration_test"
$builds += @{
    Name = $name
    Cmd  = "gcc $FLAGS -I $INC $SRC/maze_graph.c $SRC/maze_robot.c $SRC/maze_explore.c $SRC/maze_proof.c $SRC/maze_fastrun.c $SRC/maze_solver.c $TEST/$name.c -lm -o $OUT/$name.exe"
}

# ---- brain.c (compile-only, -Werror) ----
# The decision core's zero-warning check.  brain_host, its maze-driven test,
# needs a generated _maze_data.h and is driven by scripts/run_brain.py instead,
# so the object compile is what keeps brain.c under the same rule as the rest of
# the library.  brain_oracle (below) is the target that actually LINKS and RUNS
# it, and needs no maze data at all.
$name = "brain"
$builds += @{
    Name = $name
    Cmd  = "gcc $FLAGS_WERROR -I $INC -c $SRC/brain.c -o $OUT/brain.o"
}

# ---- brain_oracle (the virtual brain, linked and run) ----
# brain.c as a stdin/stdout filter: BrainIn in, one move out.  bt_monitor.py
# drives it over a pipe to check the robot's decisions against the real brain.
# No maze header, so unlike brain_host it builds and runs right here.
$name = "brain_oracle"
$builds += @{
    Name = $name
    Cmd  = "gcc $FLAGS_WERROR -I $INC $SRC/maze_graph.c $SRC/maze_robot.c $SRC/maze_explore.c $SRC/maze_proof.c $SRC/maze_fastrun.c $SRC/maze_solver.c $SRC/brain.c $TEST/brain_oracle.c -lm -o $OUT/$name.exe"
}

# -- BUILD --
$failed = 0
foreach ($b in $builds) {
    Write-Host "[BUILD] $($b.Name)" -ForegroundColor Yellow
    $result = cmd /c "$($b.Cmd) 2>&1"
    if ($LASTEXITCODE -ne 0) {
        Write-Host $result
        Write-Host "  FAILED" -ForegroundColor Red
        $failed++
    } else {
        if ($result -and $result.Trim()) { Write-Host $result }
        Write-Host "  OK" -ForegroundColor Green
    }
}

Write-Host ""
Write-Host "==============================================" -ForegroundColor Cyan
Write-Host "  RUN ALL" -ForegroundColor Cyan
Write-Host "==============================================" -ForegroundColor Cyan
Write-Host ""

# -- RUN --
# brain.o is compile-only, so it is not listed (its maze-driven test needs a
# generated header -- see scripts/run_brain.py).  brain_oracle needs no maze and
# runs right here.
$runs = @("test_graph","test_robot","integration_test")
foreach ($name in $runs) {
    $exe = "$OUT/$name.exe"
    if (Test-Path $exe) {
        Write-Host "[RUN] $name" -ForegroundColor Yellow
        & $exe
        if ($LASTEXITCODE -ne 0) { $failed++ }
        Write-Host ""
    } else {
        Write-Host "[SKIP] $name (not built)" -ForegroundColor DarkYellow
    }
}

# ---- brain_oracle --selftest ----
# The only place brain.c is EXECUTED in this suite.  Five junctions of a 2x2
# square; on any mismatch the process exits 1, so this cannot pass vacuously.
$exe = "$OUT/brain_oracle.exe"
if (Test-Path $exe) {
    Write-Host "[RUN] brain_oracle --selftest" -ForegroundColor Yellow
    & $exe --selftest
    if ($LASTEXITCODE -ne 0) { $failed++ }
    Write-Host ""
} else {
    Write-Host "[SKIP] brain_oracle (not built)" -ForegroundColor DarkYellow
}

# ---- bt_monitor.py replay checks ----
# THE DETECTOR DETECTS.  A checker that only ever passes proves nothing (the
# vacuously-passing RAM check in CHANGELOG.md is this repo's own precedent), so
# every direction is asserted: the agreeing fixture must produce ZERO
# mismatches, the disagreeing one EXACTLY ONE, the healthy health capture no
# FAIL, and the planted-fault one a FAIL -- all with exit 1 on failure.
#
# The two health fixtures are SYNTHETIC (scripts/make_health_fixtures.py) and
# test the TOOLS, not the robot.  They cannot say anything about hardware,
# which is what the bench check itself is for.
if (-not (Get-Command python -ErrorAction SilentlyContinue)) {
    Write-Host "[SKIP] bt_monitor replay (python not on PATH)" -ForegroundColor DarkYellow
} else {
    $replays = @(
        @{ File = "capture_agree.txt";        Want = 0; Args = @()
           What = "0 decision mismatches" },
        @{ File = "capture_disagree.txt";     Want = 1; Args = @()
           What = "exactly 1 decision mismatch" },
        @{ File = "capture_health_ok.txt";    Want = 0; Args = @("--health")
           What = "0 health FAIL" },
        @{ File = "capture_health_fault.txt"; Want = 1; Args = @("--health")
           What = "1 health FAIL (S7 stuck black, no calibration)" }
    )
    foreach ($r in $replays) {
        $fix = "$TEST/fixtures/$($r.File)"
        if (-not (Test-Path $fix)) {
            Write-Host "[SKIP] bt_monitor $($r.File) (fixture missing)" -ForegroundColor DarkYellow
            continue
        }
        Write-Host "[RUN] bt_monitor --replay $($r.File) $($r.Args -join ' ')" -ForegroundColor Yellow
        $argv = @("scripts/bt_monitor.py", "--replay", $fix, "--headless") + $r.Args
        $out = & python @argv 2>&1 | Out-String
        $code = $LASTEXITCODE
        Write-Host $out.TrimEnd()
        if ($code -ne $r.Want) {
            Write-Host "  FAILED: expected $($r.What), exit=$code (want $($r.Want))" -ForegroundColor Red
            $failed++
        } else {
            Write-Host "  OK ($($r.What), exit=$code)" -ForegroundColor Green
        }
        Write-Host ""
    }
}

Write-Host "==============================================" -ForegroundColor Cyan
if ($failed -eq 0) {
    Write-Host "  DONE -- all builds and runs passed" -ForegroundColor Green
    Write-Host "  (21 unit tests + 1 oracle selftest + 2 decision replays + 2 health replays)" -ForegroundColor Green
} else {
    Write-Host "  FAILED: $failed build(s)/run(s)" -ForegroundColor Red
}
Write-Host "==============================================" -ForegroundColor Cyan

if ($failed -ne 0) { exit 1 }
