/**
 * @file test_hal_compile.c
 * @brief Compile + link verification for maze_hal.h with stubbed firmware globals.
 *
 * Does NOT run a full mission — that requires real sensor data or a true-maze
 * emulator.  This test simply verifies:
 *   1. maze_hal.h compiles without warnings or errors.
 *   2. maze_hal_init() and maze_hal_tick() can be called.
 *   3. All linker symbols resolve against the real solver modules.
 *
 * Compile (from robot codes/):
 *   gcc -std=c11 -Wall -Wextra -pedantic -Werror \
 *       -I inc src/maze_graph.c src/maze_robot.c \
 *       src/maze_explore.c src/maze_proof.c \
 *       src/maze_fastrun.c src/maze_solver.c \
 *       test/test_hal_compile.c -lm -o build/test_hal_compile.exe
 */

#include "maze_hal.h"
#include <stdio.h>

/*============================================================================
 * STUB DEFINITIONS — simulate the firmware's global variables.
 *============================================================================*/

int      head            = 0;
int      nav             = 0;
int      path_c          = 0;
int      left_poss       = 0;
int      right_poss      = 0;
uint8_t  cross           = 0;
int      roatating       = 0;
_Bool    s[18]           = {0};
int      end_zone_timer  = 0;
int      node[50][6]     = {{0}};
char     BLT_TX_Buffer[256];

/*============================================================================
 * STUB MOTION PRIMITIVES — no-op implementations for the host build.
 *============================================================================*/

void Forward(void)       {}
void Forward_r(void)     {}
void turn_left(void)     {}
void turn_left_r(void)   {}
void turn_right(void)    {}
void turn_right_r(void)  {}
void MotorB(int l, int r) { (void)l; (void)r; }
int  BLT_SendData(int len) { (void)len; return 0; }

/*============================================================================
 * SIMPLE TRUE-MAZE SIMULATOR
 *
 * A minimal 3-node L-shaped maze to verify the HAL + solver integration runs
 * through its phases without crashing.  This is NOT an exhaustive test — the
 * integration_test.c covers the full 27-node maze.  Here we only verify that
 * the HAL composes correctly with the solver FSM.
 *
 * Maze layout (coordinates in cm):
 *   node 0: (0, 0)  — start
 *   node 1: (0, 20) — corner
 *   node 2: (20, 20) — target
 *
 * Edges:  0-1 (vertical, 20 cm),  1-2 (horizontal, 20 cm)
 *============================================================================*/

static int tests_run = 0, tests_failed = 0;
#define T(n)  do { tests_run++; printf("  %-52s ", n); } while(0)
#define P()   do { printf("PASS\n"); } while(0)
#define F(m)  do { printf("FAIL: %s\n", m); tests_failed++; } while(0)

/** Simulate the robot arriving at a node: update firmware position globals
 *  and sensor readings to match what the robot would see. */
static void simulate_arrive_at(int16_t x, int16_t y,
                               bool fwd, bool left, bool rgt) {
    node[path_c][0] = x;
    node[path_c][1] = y;
    left_poss  = left  ? 1 : 0;
    right_poss = rgt   ? 1 : 0;
    /* Simulate center sensors: one sensor ON = path continues straight */
    if (fwd) {
        if (head == 0) { s[4] = 1; s[5] = 1; }
        else           { s[13] = 1; s[14] = 1; }
    } else {
        if (head == 0) { s[4] = 0; s[5] = 0; s[3] = 0; s[6] = 0; }
        else           { s[13] = 0; s[14] = 0; s[12] = 0; s[15] = 0; }
    }
}

/** Simulate reaching the target: all sensors on black. */
static void simulate_target_reached(void) {
    end_zone_timer = 3;  /* >2 = target detected */
}

int main(void) {
    printf("=== HAL Compile + Smoke Test ===\n\n");

    /* ---- Test 1: Init ---- */
    T("maze_hal_init() compiles and runs");
    maze_hal_init();
    if (node[0][0] != 0 || node[0][1] != 0) { F("position not reset"); goto done; }
    if (head != 0 || nav != 0)               { F("heading not reset"); goto done; }
    if (s_maze_robot.phase != MAZE_PHASE_EXPLORE) { F("should start in EXPLORE"); goto done; }
    P();

    /* ---- Test 2: First tick at start (0,0) ---- */
    T("First tick at start sees branches");
    /* At (0,0): straight path continues (→node 1), no left/right branches */
    simulate_arrive_at(0, 0, true, false, false);
    MazeCommand cmd = maze_hal_tick();
    /* The solver should issue a forward command (P1: explore branch). */
    if (cmd == MAZE_CMD_NONE) { F("unexpected NONE on first tick"); goto done; }
    printf("(cmd=%c) ", (char)cmd);
    P();

    /* ---- Test 3: Execute forward to node 1 (0,20) ---- */
    T("Move to (0,20) — corner node");
    path_c = 1;
    simulate_arrive_at(0, 20, false, true, false);  /* dead end ahead, left branch */
    cmd = maze_hal_tick();
    /* Should turn left (to node 2). */
    if (cmd == MAZE_CMD_NONE) { F("unexpected NONE at corner"); goto done; }
    printf("(cmd=%c) ", (char)cmd);
    P();

    /* ---- Test 4: Move to (20,20) and detect target ---- */
    T("Reach target at (20,20)");
    path_c = 2;
    simulate_arrive_at(20, 20, false, false, false);  /* dead end */
    simulate_target_reached();
    cmd = maze_hal_tick();
    printf("(cmd=%c, target_found=%d) ", (char)cmd, (int)s_maze_robot.target_found);
    P();

    /* ---- Test 5: Command-to-cross conversion ---- */
    T("maze_cmd_to_cross() mappings");
    if (maze_cmd_to_cross(MAZE_CMD_FORWARD) != 0) { F("F→0"); goto done; }
    if (maze_cmd_to_cross(MAZE_CMD_LEFT)    != 1) { F("L→1"); goto done; }
    if (maze_cmd_to_cross(MAZE_CMD_RIGHT)   != 2) { F("R→2"); goto done; }
    if (maze_cmd_to_cross(MAZE_CMD_BACK)    != 4) { F("B→4"); goto done; }
    P();

    /* ---- Test 6: Direct motor execution (stubs) ---- */
    T("maze_hal_execute_direct() does not crash");
    bool ok = true;
    ok = ok && maze_hal_execute_direct(MAZE_CMD_FORWARD);
    ok = ok && maze_hal_execute_direct(MAZE_CMD_LEFT);
    ok = ok && maze_hal_execute_direct(MAZE_CMD_RIGHT);
    ok = ok && maze_hal_execute_direct(MAZE_CMD_NONE);
    if (!ok) { F("direct execution failed"); goto done; }
    P();

    /* ---- Test 7: Head-aware sensor check (head=0) ---- */
    T("Sensor forward check (head=0)");
    head = 0;
    s[3] = 0; s[4] = 1; s[5] = 0; s[6] = 0;
    if (!_hal_check_forward()) { F("forward should be true"); goto done; }
    s[3] = 0; s[4] = 0; s[5] = 0; s[6] = 0;
    if (_hal_check_forward())  { F("forward should be false"); goto done; }
    P();

    /* ---- Test 8: Head-aware sensor check (head=1) ---- */
    T("Sensor forward check (head=1)");
    head = 1;
    s[12] = 1; s[13] = 0; s[14] = 0; s[15] = 0;
    if (!_hal_check_forward()) { F("forward should be true (rear bank)"); goto done; }
    s[12] = 0; s[13] = 0; s[14] = 0; s[15] = 0;
    if (_hal_check_forward())  { F("forward should be false (rear bank)"); goto done; }
    P();

    /* ---- Test 9: Target detection ---- */
    T("Target zone detection (end_zone_timer > 2)");
    end_zone_timer = 0;
    if (_hal_check_all_black())  { F("should not detect at 0"); goto done; }
    end_zone_timer = 3;
    if (!_hal_check_all_black()) { F("should detect at 3"); goto done; }
    P();

    /* ---- Test 10: Position reading ---- */
    T("Position read from firmware node array");
    path_c = 5;
    node[5][0] = 42;
    node[5][1] = -17;
    MazeCoord c = _hal_get_position();
    if (c.x != 42 || c.y != -17) { F("position mismatch"); goto done; }
    P();

done:
    printf("\n%d tests, %d failed\n", tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}
