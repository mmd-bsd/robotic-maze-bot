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

# ---- test_hal_compile ----
$name = "test_hal_compile"
$builds += @{
    Name = $name
    Cmd  = "gcc $FLAGS_WERROR -I $INC $SRC/maze_graph.c $SRC/maze_robot.c $SRC/maze_explore.c $SRC/maze_proof.c $SRC/maze_fastrun.c $SRC/maze_solver.c $TEST/$name.c -lm -o $OUT/$name.exe"
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

# -- RUN (skip test_hal_compile which needs stubbed firmware globals) --
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
