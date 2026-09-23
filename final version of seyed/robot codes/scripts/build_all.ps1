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
# The decision core's warning check.  It cannot be LINKED here -- brain_host,
# its only test, needs a generated _maze_data.h and is driven by
# scripts/run_brain.py instead -- but the object compile needs no maze data and
# is what keeps brain.c under the same zero-warning rule as the rest of the
# library.  Compile-only, so it is not run below.
$name = "brain"
$builds += @{
    Name = $name
    Cmd  = "gcc $FLAGS_WERROR -I $INC -c $SRC/brain.c -o $OUT/brain.o"
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
# brain is compile-only, so it is not listed (its test needs a generated maze
# header -- see scripts/run_brain.py).
$runs = @("test_graph","test_robot","integration_test")
foreach ($name in $runs) {
    $exe = "$OUT/$name.exe"
    if (Test-Path $exe) {
        Write-Host "[RUN] $name" -ForegroundColor Yellow
        & $exe
        Write-Host ""
    } else {
        Write-Host "[SKIP] $name (not built)" -ForegroundColor DarkYellow
    }
}

Write-Host "==============================================" -ForegroundColor Cyan
Write-Host "  DONE" -ForegroundColor Cyan
Write-Host "==============================================" -ForegroundColor Cyan
