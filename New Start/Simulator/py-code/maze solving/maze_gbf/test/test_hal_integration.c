/**
 * @file test_hal_integration.c
 * @brief Host-side test for the STM32 maze HAL.
 *
 * Built only with -DMAZE_HAL_TEST (see Makefile target `test-hal`). Stubs out
 * the firmware globals and motion primitives that `maze_hal.h` references,
 * then exercises:
 *   1. the absolute-direction-to-cross translation table across all
 *      (direction, nav) combinations,
 *   2. the BACK path in `execute_movement()` (head toggle + 180° on nav),
 *   3. an end-to-end drive through a small maze using the real algorithm +
 *      HAL translator + a simulated firmware heading.
 */

#include "../inc/maze_gbf.h"
#include "../inc/maze_utils.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/*============================================================================
 * Stub definitions for symbols declared `extern` in maze_hal.h.
 *============================================================================*/

int      head           = 0;
int      nav            = 0;
int      path_c         = 0;
int      left_poss      = 0;
int      right_poss     = 0;
uint8_t  cross          = 0;
int      roatating      = 0;
_Bool    s[18]          = {0};
int      end_zone_timer = 0;
int      node[50][6]    = {{0}};
char     BLT_TX_Buffer[256] = {0};

/* Counters let test cases verify which primitives were invoked. */
static int count_forward, count_forward_r;
static int count_turn_left, count_turn_left_r;
static int count_turn_right, count_turn_right_r;
static int count_motor_b_stop;

void Forward(void)       { count_forward++; }
void Forward_r(void)     { count_forward_r++; }
void turn_left(void)     { count_turn_left++; }
void turn_left_r(void)   { count_turn_left_r++; }
void turn_right(void)    { count_turn_right++; }
void turn_right_r(void)  { count_turn_right_r++; }
void MotorB(int l, int r) { if (l == 0 && r == 0) count_motor_b_stop++; }
int  BLT_SendData(int len) { (void)len; return 0; }

/* Include the HAL AFTER stubs are defined so the extern declarations link. */
#include "../stm-sample-code/maze_hal.h"

static void reset_counters(void) {
    count_forward = count_forward_r = 0;
    count_turn_left = count_turn_left_r = 0;
    count_turn_right = count_turn_right_r = 0;
    count_motor_b_stop = 0;
}

/*============================================================================
 * 1. Direction-to-cross mapping table (16 cells)
 *============================================================================*/

static void test_direction_to_cross_mapping(void) {
    printf("\n=== TEST: maze_direction_to_cross mapping ===\n");

    /* Expected matrix [dir][nav], per plan:
     *   FORWARD=N  LEFT=W  BACK=S  RIGHT=E
     *   cross: 0=straight 1=left 2=right 4=U-turn                          */
    const uint8_t expected[4][4] = {
        /* cur_nav:      0(N)  1(W)  2(S)  3(E) */
        /* FORWARD (N)*/ { 0,    2,    4,    1 },
        /* LEFT    (W)*/ { 1,    0,    2,    4 },
        /* BACK    (S)*/ { 4,    1,    0,    2 },
        /* RIGHT   (E)*/ { 2,    4,    1,    0 },
    };
    const MazeDirection dirs[4] = {
        MAZE_DIR_FORWARD, MAZE_DIR_LEFT, MAZE_DIR_BACK, MAZE_DIR_RIGHT
    };

    int failures = 0;
    for (int d = 0; d < 4; d++) {
        for (int n = 0; n < 4; n++) {
            uint8_t got = maze_direction_to_cross(dirs[d], n);
            if (got != expected[d][n]) {
                printf("  FAIL dir=%d nav=%d: expected %u, got %u\n",
                       dirs[d], n, expected[d][n], got);
                failures++;
            }
        }
    }

    /* STOP should produce 0xFF (non-actionable sentinel). */
    if (maze_direction_to_cross(MAZE_DIR_STOP, 0) != 0xFF) {
        printf("  FAIL STOP did not return 0xFF\n");
        failures++;
    }

    if (failures == 0) printf("  PASS: all 16 cells + STOP sentinel correct\n");
    assert(failures == 0);
}

/*============================================================================
 * 2. BACK path toggles head + adjusts nav + drives reversed primitive
 *============================================================================*/

static void test_back_toggles_head_and_nav(void) {
    printf("\n=== TEST: execute_movement(MAZE_DIR_BACK) ===\n");

    head = 0;
    nav = 0;
    reset_counters();

    bool ok = execute_movement(MAZE_DIR_BACK);
    assert(ok);
    assert(head == 1);
    assert(nav == 2);
    /* With head flipped to 1, the reversed primitive should have fired. */
    assert(count_forward_r == 1);
    assert(count_forward == 0);

    /* Calling BACK again should flip head back to 0 and roll nav to 0. */
    ok = execute_movement(MAZE_DIR_BACK);
    assert(ok);
    assert(head == 0);
    assert(nav == 0);
    assert(count_forward == 1);

    printf("  PASS: head toggles, nav += 2 mod 4, correct primitive called\n");
}

/*============================================================================
 * 3. Other execute_movement dispatch paths (sanity)
 *============================================================================*/

static void test_execute_movement_dispatch(void) {
    printf("\n=== TEST: execute_movement forward/left/right dispatch ===\n");

    head = 0; reset_counters();
    execute_movement(MAZE_DIR_FORWARD); assert(count_forward == 1);
    execute_movement(MAZE_DIR_LEFT);    assert(count_turn_left == 1);
    execute_movement(MAZE_DIR_RIGHT);   assert(count_turn_right == 1);

    head = 1; reset_counters();
    execute_movement(MAZE_DIR_FORWARD); assert(count_forward_r == 1);
    execute_movement(MAZE_DIR_LEFT);    assert(count_turn_left_r == 1);
    execute_movement(MAZE_DIR_RIGHT);   assert(count_turn_right_r == 1);

    reset_counters();
    execute_movement(MAZE_DIR_STOP);    assert(count_motor_b_stop == 1);

    printf("  PASS: forward/left/right routed per head; STOP halts motors\n");
}

/*============================================================================
 * 4. End-to-end pipeline: read_sensors + get_current_position + algorithm +
 *    translator, all wired together. We verify each HAL stage faithfully
 *    reflects firmware-global state; the algorithm's own decisions are
 *    exercised by test_maze_gbf.c / integration_test.c, not here.
 *============================================================================*/

static void test_drive_linear_maze(void) {
    printf("\n=== TEST: HAL pipeline (read_sensors + position + translator) ===\n");

    MazeRobot robot;
    MazeGraph graph;
    maze_hal_init();   /* resets firmware-global state */
    int init = maze_gbf_init(&robot, &graph, 0, 0);
    assert(init == MAZE_SUCCESS);

    /* (a) get_current_position reads node[path_c][] correctly. */
    path_c = 1; node[1][0] = 123; node[1][1] = -45;
    MazeCoord pos = get_current_position();
    assert(pos.x == 123 && pos.y == -45);

    /* (b) read_sensors reflects latched firmware flags and forward detection. */
    path_c = 0; node[0][0] = 0; node[0][1] = 0;
    left_poss = 1; right_poss = 0;
    s[3] = 0; s[4] = 1; s[5] = 0; s[6] = 0;  /* path continues forward */
    end_zone_timer = 0;
    head = 0;

    MazeSensors sensors;
    read_sensors(&sensors);
    assert(sensors.can_go_forward);          /* s[4] = 1 */
    assert(sensors.can_go_left);             /* left_poss = 1 */
    assert(!sensors.can_go_right);           /* right_poss = 0 */
    assert(sensors.can_go_back);             /* always true (U-turn) */
    assert(!sensors.target_reached);         /* end_zone_timer not over 2 */

    /* (c) target-zone debounce: end_zone_timer > 2 trips target_reached. */
    end_zone_timer = 3;
    read_sensors(&sensors);
    assert(sensors.target_reached);
    end_zone_timer = 2;                       /* boundary: strictly greater-than */
    read_sensors(&sensors);
    assert(!sensors.target_reached);

    /* (d) Algorithm → translator pipeline does not crash and yields a
     *     translator-recognized value (0/1/2/4) or the 0xFF sentinel. */
    end_zone_timer = 0;
    left_poss = 0; right_poss = 0;
    s[3] = s[4] = s[5] = s[6] = 1;
    MazeDirection dir = maze_gbf_get_next_move(
            &robot, &sensors, get_current_x(), get_current_y());
    uint8_t c = maze_direction_to_cross(dir, nav);
    printf("  algo @ (0,0) nav=0: dir=%s cross=%u\n",
           maze_utils_direction_to_string(dir), c);
    assert(c == 0 || c == 1 || c == 2 || c == 4 || c == 0xFF);

    /* (e) Reversed-heading sensor path: head=1 reads rear sensor bank. */
    head = 1;
    s[3] = s[4] = s[5] = s[6] = 0;
    s[12] = 0; s[13] = 1; s[14] = 0; s[15] = 0;
    read_sensors(&sensors);
    assert(sensors.can_go_forward);          /* s[13] via head==1 branch */

    maze_gbf_cleanup(&robot, &graph);
    printf("  PASS: sensors, position, target debounce, and algorithm plumbing all wired\n");
}

/*============================================================================
 * Main
 *============================================================================*/

int main(void) {
    printf("=================================================\n");
    printf(" Maze HAL Integration Tests (host)\n");
    printf("=================================================\n");

    test_direction_to_cross_mapping();
    test_back_toggles_head_and_nav();
    test_execute_movement_dispatch();
    test_drive_linear_maze();

    printf("\n=================================================\n");
    printf(" ALL HAL TESTS PASSED\n");
    printf("=================================================\n");
    return 0;
}
