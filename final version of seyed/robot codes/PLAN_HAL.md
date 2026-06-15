# HAL Bridge Plan — Step 9 ✅ IMPLEMENTED

## Goal

Create `maze_hal.h` that bridges the new C solver (`maze_solver.h`) to the STM32
firmware (`New Start/code/Core/Src/main.c`), replacing the old V12-style GBF
integration behind a `USE_MAZE_SOLVER` compile-time flag.

## Design

### API surface (what the firmware calls)

Two functions + a global (pattern from the existing `maze_hal.h`):

```c
void maze_hal_init(void);          // init solver globals, called once at boot
MazeCommand maze_hal_tick(void);   // read sensors, update solver, return next command
```

`maze_hal_tick()` does ALL the work in one shot:
1. Reads firmware globals (`s[]`, `left_poss`, `right_poss`, `end_zone_timer`, `nav`)
2. Gets current position from `node[path_c][0..1]`
3. Calls `maze_solver_update_position()` to tell the solver where we are and what we see
4. Calls `maze_solver_step()` to advance the FSM
5. Returns the `MazeCommand` (F/L/R/B) — the firmware converts it to `cross` via
   the already-existing `maze_cmd_to_cross()` in `maze_types.h`

### Sensor mapping (head-aware)

The firmware drives with `head`: 0 = driving forward (sensors 3-6 center, 2=left, 7=right),
1 = driving reversed (sensors 12-15 center, 11=left, 16=right).

```
can_go_forward:  s[3..6] (head=0) or s[12..15] (head=1)
can_go_left:     left_poss == 1   (already latched by firmware intersection logic)
can_go_right:    right_poss == 1
can_go_back:     always true
target_reached:  end_zone_timer > 2  (debounced black-area detection)
```

### Position coupling

The solver manages its OWN graph (static `s_maze_graph` inside the HAL). Position
reads from the firmware's `node[path_c][0..1]` (encoder-based dead reckoning
maintained by the firmware's existing per-segment logic).

### Firmware integration pattern

In `main.c`, the `#ifdef USE_MAZE_GBF` blocks become `#ifdef USE_MAZE_SOLVER`
with a different call shape:

```c
#ifdef USE_MAZE_SOLVER
#include "maze_hal.h"
#endif

// At init:
#ifdef USE_MAZE_SOLVER
    maze_hal_init();
#endif

// At intersection decision (head=0 block, ~line 1221-1267):
#ifdef USE_MAZE_SOLVER
    {
        MazeCommand _cmd = maze_hal_tick();
        if (_cmd == MAZE_CMD_NONE) {
            cross = 0; MotorB(0, 0);
        } else {
            cross = maze_cmd_to_cross(_cmd);
            path_append(cross == 0 ? 'S' : cross == 1 ? 'L' :
                        cross == 2 ? 'R' : 'B');
            if (cross == 0) { left_poss = 0; right_poss = 0; }
        }
    }
#endif
```

Same for the head=1 block (~line 1328-1370).

### What the HAL does NOT do

- Does NOT drive motors directly (the firmware's existing `cross` dispatch handles that)
- Does NOT track position — it reads the firmware's `node[path_c]`
- Does NOT handle Bluetooth/UART — the firmware's `BLT_SendData`/`path_append` stay
- Does NOT replace the firmware's `loop_start` FSM — it runs within `loop_start == 1`

### File to create

`robot codes/inc/maze_hal.h` — single header (100-150 lines),
same `static inline` style as the existing `maze_hal.h`.

### Tests

Add a compile-only check: GCC `-fsyntax-only` with stubs for firmware globals.
No integration test yet (needs real hardware or a full firmware simulator).

### Deliverables

1. `maze_hal.h` — the bridge header
2. `test/test_hal_compile.c` — compile-only verification (stubs for firmware globals)
3. Updated `STATUS.md` — mark Step 9 done
4. Updated `CHANGELOG.md` — entry for today
5. Updated `CLAUDE.md` — add maze_hal.h to the C reference map
