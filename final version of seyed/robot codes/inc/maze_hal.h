/**
 * @file maze_hal.h
 * @brief Hardware Abstraction Layer — bridges the C maze solver to STM32 firmware.
 *
 * ## Integration pattern (in main.c)
 *
 * Replace the existing `USE_MAZE_GBF` blocks with `USE_MAZE_SOLVER`:
 *
 *     #ifdef USE_MAZE_SOLVER
 *     #include "maze_hal.h"
 *     #endif
 *
 *     // At boot (after sensor init, before main loop):
 *     #ifdef USE_MAZE_SOLVER
 *         maze_hal_init();
 *     #endif
 *
 *     // At each intersection decision point (inside `if (right_poss || left_poss)`):
 *     #ifdef USE_MAZE_SOLVER
 *         MazeCommand _cmd = maze_hal_tick();
 *         if (_cmd == MAZE_CMD_NONE) {
 *             cross = 0; MotorB(0, 0);
 *         } else {
 *             cross = maze_cmd_to_cross(_cmd);
 *             path_append(cross == 0 ? 'S' : cross == 1 ? 'L' :
 *                         cross == 2 ? 'R' : 'B');
 *             if (cross == 0) { left_poss = 0; right_poss = 0; }
 *         }
 *     #endif
 *
 * The HAL does NOT drive motors directly — it returns a MazeCommand; the
 * firmware's existing `cross` → Forward/turn_left/turn_right dispatch handles
 * execution.  Position tracking (`node[path_c]`, `path_append`) and UART debug
 * output remain the firmware's responsibility.
 *
 * ## Design notes
 *
 * - **Header-only** (static inline) — matches the existing `maze_hal.h` pattern.
 * - **Solver state is owned here** (`static MazeGraph + MazeRobot`).  Only one
 *   translation unit should include this header (main.c on target).
 * - **Sensor reading is head-aware** — the firmware drives both forward (`head=0`,
 *   sensors 3-6) and reversed (`head=1`, sensors 12-15); the HAL reads whichever
 *   bank is active.
 * - **Position is READ-ONLY** — the HAL reads `node[path_c][0..1]` (the firmware's
 *   encoder-based dead reckoning) and feeds it into the solver's graph.
 * - **Target detection** uses `end_zone_timer > 2` — the same debounced counter
 *   the firmware already maintains.
 */

#ifndef MAZE_HAL_H
#define MAZE_HAL_H

#include "maze_solver.h"
#include "maze_robot.h"
#include "maze_graph.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/*============================================================================
 * FIRMWARE GLOBALS (extern — defined in main.c, stubbed in test harness)
 *============================================================================*/

extern int      head;            /* 0 = front-forward, 1 = reversed (main.c:486)  */
extern int      nav;             /* 0=N, 1=W, 2=S, 3=E (main.c:88)               */
extern int      path_c;          /* current node index (main.c:810)               */
extern int      left_poss;       /* latched: left branch exists at junction (:102) */
extern int      right_poss;      /* latched: right branch exists at junction (:103)*/
extern uint8_t  cross;           /* 0=straight 1=left 2=right 4=U-turn (:485)    */
extern int      roatating;       /* 1 while a motion primitive is in flight (:484)*/
extern _Bool    s[18];           /* IR sensor array (main.c:95)                   */
extern int      end_zone_timer;  /* target-zone debounce counter (main.c:80)      */
extern int      node[50][6];     /* node[path_c][0]=x, [1]=y (main.c:112)         */
extern char     BLT_TX_Buffer[];

/* Forward / turn primitives — declared so the optional direct-motor-control
 * helpers below can reference them.  Defined in main.c. */
extern void Forward(void);
extern void Forward_r(void);
extern void turn_left(void);
extern void turn_left_r(void);
extern void turn_right(void);
extern void turn_right_r(void);
extern void MotorB(int l, int r);
extern int  BLT_SendData(int len);

/*============================================================================
 * SOLVER STATE (static — owned by this header, single-TU by design)
 *============================================================================*/

static MazeGraph  s_maze_graph;
static MazeRobot  s_maze_robot;
static bool       s_maze_hal_ready = false;

/*============================================================================
 * SENSOR READING (head-aware)
 *
 * Meaningful only at an intersection decision point, where the firmware has
 * already latched `left_poss`/`right_poss` and the center-sensor snapshot
 * reflects "does the straight path continue past this junction".
 *============================================================================*/

/** Check the center sensor bank for a continuing straight path. */
static inline bool _hal_check_forward(void) {
    if (head == 0) {
        return (s[3] || s[4] || s[5] || s[6]);
    }
    return (s[12] || s[13] || s[14] || s[15]);
}

/** Left branch exists (already latched by firmware intersection logic). */
static inline bool _hal_check_left(void)  { return (left_poss == 1); }

/** Right branch exists (already latched). */
static inline bool _hal_check_right(void) { return (right_poss == 1); }

/** Back (U-turn) is always physically possible. */
static inline bool _hal_check_back(void)  { return true; }

/** Debounced black-area (target zone) detection. */
static inline bool _hal_check_all_black(void) {
    return (end_zone_timer > 2);
}

/** Populate a MazeSensors struct from the current firmware state. */
static inline void _hal_read_sensors(MazeSensors* sensors) {
    sensors->can_go_forward = _hal_check_forward();
    sensors->can_go_left    = _hal_check_left();
    sensors->can_go_right   = _hal_check_right();
    sensors->can_go_back    = _hal_check_back();
    sensors->target_reached = _hal_check_all_black();
}

/*============================================================================
 * POSITION — read-only window onto the firmware's per-segment encoder tracker.
 *============================================================================*/

/** Current X coordinate (cm) from the firmware's dead reckoning. */
static inline int16_t _hal_get_x(void) { return (int16_t)node[path_c][0]; }

/** Current Y coordinate (cm). */
static inline int16_t _hal_get_y(void) { return (int16_t)node[path_c][1]; }

/** Current position as a MazeCoord. */
static inline MazeCoord _hal_get_position(void) {
    MazeCoord pos = { _hal_get_x(), _hal_get_y() };
    return pos;
}

/*============================================================================
 * COMMAND → CROSS CONVERSION
 *
 * Converts the solver's MazeCommand (F/L/R/B) into the firmware's heading-
 * relative `cross` value (0/1/2/4).  This is a thin wrapper over the inline
 * `maze_cmd_to_cross()` already defined in maze_types.h.
 *
 * The firmware's existing motion dispatch (Forward, turn_left, etc.) is keyed
 * on `cross`, so the integration is drop-in: store `cross`, and the firmware
 * handles the rest.
 *============================================================================*/

static inline uint8_t _hal_cmd_to_cross(MazeCommand cmd) {
    return maze_cmd_to_cross(cmd);
}

/*============================================================================
 * INITIALIZATION
 *
 * Call once at boot, after sensor calibration and IMU initialisation.
 * Resets `path_c` / `nav` / `head` and initialises the solver at (0,0).
 *============================================================================*/

static inline void maze_hal_init(void) {
    /* Reset firmware position tracking to origin. */
    path_c     = 0;
    node[0][0] = 0;
    node[0][1] = 0;
    nav        = 0;
    head       = 0;
    left_poss  = 0;
    right_poss = 0;
    cross      = 0;
    roatating  = 0;

    /* Initialise solver graph + robot at start (0,0). */
    MazeCoord start = {0, 0};
    maze_solver_init(&s_maze_graph, &s_maze_robot, start);
    s_maze_hal_ready = true;

    BLT_SendData(sprintf(BLT_TX_Buffer, "Maze Solver HAL Ready\r\n"));
}

/*============================================================================
 * MAIN TICK — call once per intersection decision point.
 *
 *   1. Read sensors and current position from firmware globals.
 *   2. Feed position + sensor data into the solver (graph update).
 *   3. Advance the solver state machine one step.
 *   4. Return the MazeCommand to execute.
 *
 * Returns MAZE_CMD_NONE when the mission is DONE.
 *============================================================================*/

static inline MazeCommand maze_hal_tick(void) {
    if (!s_maze_hal_ready)         return MAZE_CMD_NONE;
    if (s_maze_robot.phase == MAZE_PHASE_DONE) return MAZE_CMD_NONE;

    /* 1. Read current state from firmware. */
    MazeSensors sensors;
    _hal_read_sensors(&sensors);
    MazeCoord pos = _hal_get_position();

    /* 2. Update the solver's world model. */
    maze_solver_update_position(&s_maze_robot, pos, &sensors);

    /* 3. Advance the mission FSM. */
    MazeCommand cmd = maze_solver_step(&s_maze_robot, MAZE_V_MAX_FP, MAZE_ACCEL_FP);

    return cmd;
}

/*============================================================================
 * DIAGNOSTICS (optional — compile-time gated by MAZE_DEBUG_ENABLED)
 *============================================================================*/

#if MAZE_DEBUG_ENABLED

static inline void maze_hal_print_state(void) {
    MazeCoord c = _hal_get_position();
    const char* phase_str;
    switch (s_maze_robot.phase) {
        case MAZE_PHASE_EXPLORE:     phase_str = "EXPLORE";     break;
        case MAZE_PHASE_RETURN_HOME: phase_str = "RETURN_HOME"; break;
        case MAZE_PHASE_FAST_RUN:    phase_str = "FAST_RUN";    break;
        case MAZE_PHASE_DONE:        phase_str = "DONE";        break;
        default:                     phase_str = "?";           break;
    }
    BLT_SendData(sprintf(BLT_TX_Buffer,
        "Maze: phase=%s pos=(%d,%d) nav=%d head=%d nodes=%u edges=%u steps=%u\r\n",
        phase_str, (int)c.x, (int)c.y, nav, head,
        (unsigned)s_maze_graph.node_count,
        (unsigned)s_maze_graph.edge_count,
        (unsigned)s_maze_robot.steps_taken));
}

static inline void maze_hal_print_command(MazeCommand cmd) {
    const char* s = (cmd == MAZE_CMD_FORWARD) ? "F" :
                    (cmd == MAZE_CMD_LEFT)    ? "L" :
                    (cmd == MAZE_CMD_RIGHT)   ? "R" :
                    (cmd == MAZE_CMD_BACK)    ? "B" :
                    (cmd == MAZE_CMD_NONE)    ? "STOP" : "?";
    BLT_SendData(sprintf(BLT_TX_Buffer, "Maze cmd: %s\r\n", s));
}

#else

#define maze_hal_print_state()     do {} while (0)
#define maze_hal_print_command(c)  do {} while (0)

#endif /* MAZE_DEBUG_ENABLED */

/*============================================================================
 * DIRECT MOTOR CONTROL (alternative integration path)
 *
 * The PRIMARY integration uses `cross`-based dispatch (call `maze_hal_tick()`,
 * convert via `maze_cmd_to_cross()`, store in `cross`, and let the firmware's
 * existing motion logic run it).
 *
 * This helper exists for any alternative call site that wants to drive motors
 * directly without going through `cross`.  BACK follows the firmware's U-turn
 * idiom (flip `head`, +180° on `nav`, drive reversed).
 *============================================================================*/

static inline bool maze_hal_execute_direct(MazeCommand cmd) {
    switch (cmd) {
        case MAZE_CMD_FORWARD:
            if (head == 0) Forward(); else Forward_r();
            return true;
        case MAZE_CMD_LEFT:
            if (head == 0) turn_left(); else turn_left_r();
            return true;
        case MAZE_CMD_RIGHT:
            if (head == 0) turn_right(); else turn_right_r();
            return true;
        case MAZE_CMD_BACK:
            head ^= 1;
            nav = (nav + 2) & 0x3;
            if (head == 0) Forward(); else Forward_r();
            return true;
        case MAZE_CMD_NONE:
            MotorB(0, 0);
            return true;
        default:
            return false;
    }
}

#endif /* MAZE_HAL_H */
