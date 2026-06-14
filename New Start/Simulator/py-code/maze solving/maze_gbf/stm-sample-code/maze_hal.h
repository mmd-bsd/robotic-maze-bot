/**
 * @file maze_hal.h
 * @brief Hardware Abstraction Layer for Maze Navigation
 *
 * Bridges the maze navigation algorithm in `maze_gbf/` with the robot's
 * existing STM32 firmware (sensors, motors, encoders, localization).
 *
 * Design notes:
 *   - The header declares every firmware symbol it touches as `extern` so it
 *     can be included at any point in `main.c` (or a test .c with matching
 *     stub definitions under -DMAZE_HAL_TEST).
 *   - Position is NOT tracked by the HAL. `node[path_c][0..1]` is maintained
 *     by the firmware's per-segment encoder logic (main.c:859/868/874/880,
 *     reset at main.c:1197-1198). The HAL only reads it.
 *   - Target-zone detection uses the firmware's already-debounced counter
 *     `end_zone_timer > 2` (main.c:1170-1175) rather than the local-scope
 *     `OnEndZoon` which isn't reachable from outside the main loop.
 */

#ifndef MAZE_HAL_H
#define MAZE_HAL_H

#include "../inc/maze_types.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/*============================================================================
 * Firmware globals + motion primitives (defined in main.c, or stubbed by
 * the host test harness when MAZE_HAL_TEST is set).
 *============================================================================*/

extern int      head;           /* 0 = front-forward, 1 = reversed (main.c:485)  */
extern int      nav;            /* 0=N, 1=W, 2=S, 3=E (main.c:80)                */
extern int      path_c;         /* current node index (main.c:809)               */
extern int      left_poss;      /* latched: left path exists at junction (:94)   */
extern int      right_poss;     /* latched: right path exists at junction (:95)  */
extern uint8_t  cross;          /* 0=straight 1=left 2=right 4=U-turn (:484)     */
extern int      roatating;      /* 1 while a motion primitive is in flight (:483)*/
extern _Bool    s[18];          /* sensor array, 1..8 front, 10..17 rear (:87)   */
extern int      end_zone_timer; /* target-zone debounce counter (main.c:72)      */
extern int      node[50][6];    /* node[path_c][0]=x, [1]=y (main.c:104)         */
extern char     BLT_TX_Buffer[];

extern void Forward(void);
extern void Forward_r(void);
extern void turn_left(void);
extern void turn_left_r(void);
extern void turn_right(void);
extern void turn_right_r(void);
extern void MotorB(int l, int r);
extern int  BLT_SendData(int len);

/*============================================================================
 * SENSOR READING
 *
 * Meaningful only at an intersection decision point, where the firmware has
 * already latched `left_poss`/`right_poss` and the center-sensor snapshot
 * reflects "does the straight path continue past this junction".
 *============================================================================*/

static inline bool check_forward_sensor(void) {
    if (head == 0) {
        return (s[3] || s[4] || s[5] || s[6]);
    }
    return (s[12] || s[13] || s[14] || s[15]);
}

static inline bool check_left_sensor(void)  { return (left_poss == 1); }
static inline bool check_right_sensor(void) { return (right_poss == 1); }
static inline bool check_back_sensor(void)  { return true; } /* U-turn always legal */

static inline bool check_all_black(void) {
    /* Same 2-tick debounce the firmware uses to derive `OnEndZoon`. */
    return (end_zone_timer > 2);
}

static inline void read_sensors(MazeSensors* sensors) {
    sensors->can_go_forward = check_forward_sensor();
    sensors->can_go_left    = check_left_sensor();
    sensors->can_go_right   = check_right_sensor();
    sensors->can_go_back    = check_back_sensor();
    sensors->target_reached = check_all_black();
}

/*============================================================================
 * LOCALIZATION — read-only window onto the firmware's per-segment tracker.
 *============================================================================*/

static inline int16_t get_current_x(void) { return (int16_t)node[path_c][0]; }
static inline int16_t get_current_y(void) { return (int16_t)node[path_c][1]; }

static inline MazeCoord get_current_position(void) {
    MazeCoord pos = { get_current_x(), get_current_y() };
    return pos;
}

/*============================================================================
 * DIRECTION TRANSLATION
 *
 * The maze algorithm emits absolute cardinal directions (verified
 * maze_robot.c:304-324):
 *     FORWARD=+Y(N)  LEFT=-X(W)  BACK=-Y(S)  RIGHT=+X(E)
 * The firmware drives via `cross` in the robot's heading-relative frame:
 *     0=straight  1=left  2=right  4=U-turn
 * This helper does the conversion using the current `nav`.
 *============================================================================*/

static inline int maze_direction_as_nav(MazeDirection dir) {
    switch (dir) {
        case MAZE_DIR_FORWARD: return 0;  /* north */
        case MAZE_DIR_LEFT:    return 1;  /* west  */
        case MAZE_DIR_BACK:    return 2;  /* south */
        case MAZE_DIR_RIGHT:   return 3;  /* east  */
        default:               return -1;
    }
}

/**
 * @brief Translate an absolute MazeDirection to the firmware `cross` value.
 * @return 0/1/2/4 for a valid direction; 0xFF for STOP or unknown input.
 */
static inline uint8_t maze_direction_to_cross(MazeDirection dir, int cur_nav) {
    int dir_nav = maze_direction_as_nav(dir);
    if (dir_nav < 0) return 0xFF;
    int delta = (dir_nav - cur_nav) & 0x3;
    static const uint8_t lut[4] = { 0, 1, 4, 2 };
    return lut[delta];
}

/*============================================================================
 * DIRECT MOTOR CONTROL (optional path)
 *
 * The primary integration in `main.c` uses the cross-based wiring: call
 * `maze_gbf_get_next_move()`, convert with `maze_direction_to_cross()`, store
 * into `cross`, and let the firmware's existing motion dispatch run it.
 *
 * This helper exists for any alternative call site that wants to drive motors
 * directly without going through `cross`. BACK follows the firmware's U-turn
 * idiom at main.c:1267 (flip `head`, +180° on `nav`, drive reversed).
 *============================================================================*/

static inline bool execute_movement(MazeDirection direction) {
    switch (direction) {
        case MAZE_DIR_FORWARD:
            if (head == 0) Forward(); else Forward_r();
            return true;
        case MAZE_DIR_LEFT:
            if (head == 0) turn_left(); else turn_left_r();
            return true;
        case MAZE_DIR_RIGHT:
            if (head == 0) turn_right(); else turn_right_r();
            return true;
        case MAZE_DIR_BACK:
            head ^= 1;
            nav = (nav + 2) & 0x3;
            if (head == 0) Forward(); else Forward_r();
            return true;
        case MAZE_DIR_STOP:
            MotorB(0, 0);
            return true;
        default:
            return false;
    }
}

/*============================================================================
 * DEBUG OUTPUT
 *============================================================================*/

static inline void maze_debug_printf(const char* message) {
    BLT_SendData(sprintf(BLT_TX_Buffer, "%s\r\n", message));
}

static inline void maze_print_direction(MazeDirection direction) {
    const char* dir_str;
    switch (direction) {
        case MAZE_DIR_STOP:    dir_str = "STOP";    break;
        case MAZE_DIR_FORWARD: dir_str = "FORWARD"; break;
        case MAZE_DIR_LEFT:    dir_str = "LEFT";    break;
        case MAZE_DIR_RIGHT:   dir_str = "RIGHT";   break;
        case MAZE_DIR_BACK:    dir_str = "BACK";    break;
        default:               dir_str = "UNKNOWN"; break;
    }
    BLT_SendData(sprintf(BLT_TX_Buffer, "Dir: %s\r\n", dir_str));
}

/*============================================================================
 * INITIALIZATION
 *============================================================================*/

static inline void maze_hal_init(void) {
    path_c     = 0;
    node[0][0] = 0;
    node[0][1] = 0;
    nav        = 0;
    head       = 0;
    left_poss  = 0;
    right_poss = 0;
    cross      = 0;
    roatating  = 0;
    BLT_SendData(sprintf(BLT_TX_Buffer, "Maze HAL Init Complete\r\n"));
}

#endif /* MAZE_HAL_H */
