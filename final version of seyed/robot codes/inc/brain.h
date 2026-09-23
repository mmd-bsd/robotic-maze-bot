/**
 * @file brain.h
 * @brief SEYED decision core — a pure decision function over junction reports.
 *
 * THE CONTRACT
 * ------------
 * The caller owns everything physical: line following, driving to the next
 * junction, junction detection, target detection, encoder distance.  The brain
 * owns only the reasoning: what the maze looks like, where to go next, and
 * when the search is provably finished.
 *
 * The caller drives to a junction, reads its exits, and calls brain_step()
 * once.  The brain answers with one move.  The caller executes it and drives
 * to the next junction.  Repeat until brain_step() returns BRAIN_DONE, at
 * which point brain_home_path() and brain_fast_path() hold the two command
 * strings the mission needs.
 *
 *     robot                                     brain
 *       |                                         |
 *       |  drives to the next junction            |
 *       |---- BrainIn{exits, target, dist_cm} --->|
 *       |<------- 'F' | 'L' | 'R' | 'B' ----------|
 *       |  executes the move                      |
 *       |         ... repeat ...                  |
 *       |<------- BRAIN_DONE ---------------------|
 *       |  brain_home_path() -> "FLRFF..."        |
 *       |  brain_fast_path() -> "FFRFLF..."       |
 *
 * The brain never reads a sensor, a motor, an encoder, or an absolute compass
 * heading.  "Left" and "right" are always relative to the robot's own facing.
 * The brain tracks its heading internally, so the caller never has to know
 * which way is north.
 *
 * WHERE THE ROBOT MUST REPORT
 * ---------------------------
 * At every junction it stops at.  It does NOT have to stop at a straight
 * passthrough, and it should not: a node with no side branch has no decision
 * to make.  The brain reconstructs those from `dist_cm` — see MAZE_CELL_CM in
 * inc/maze_config.h.
 *
 * The very first call must be at the start node, with dist_cm = 0, so the
 * brain learns the start's exits.  Nothing after that is special-cased.
 *
 * UNITS
 * -----
 * 1 unit = 1 cm everywhere.  `dist_cm` is the distance actually driven since
 * the previous report, measured along the path — not the straight-line
 * distance between the two nodes.  On a turn those differ, and the difference
 * is exactly what the fast-run planner needs, so report the driven distance.
 *
 * It is REQUIRED, not optional: the early-stop proof is geometric
 * (`LB_time(f) = (dist_known(start→f) + ‖f→target‖) / v_max`), so the brain can
 * only place nodes in space by dead-reckoning from reported distances.  Without
 * it the proof degrades to hop count and loses its guarantee.
 *
 * RAM ON TARGET: the brain owns one MazeGraph + one MazeRobot (~1.9 KB at the
 * configured limits) plus two command strings (~390 B).  There is no `malloc`.
 */

#ifndef BRAIN_H
#define BRAIN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * INPUT — one junction report
 *============================================================================*/

/**
 * @brief  What the robot sees when it stops at a junction.
 *
 * Every flag is relative to the robot's own facing, which is how the sensors
 * see the world; the caller does no direction arithmetic.
 */
typedef struct {
    uint8_t  front;      /**< 1 = the line continues straight past this node   */
    uint8_t  left;       /**< 1 = a branch opens to the robot's left           */
    uint8_t  right;      /**< 1 = a branch opens to the robot's right          */
    uint8_t  back;       /**< 1 = reverse is available (normally always 1)     */
    uint8_t  target;     /**< 1 = the big black target area is under the bar   */
    uint16_t dist_cm;    /**< distance DRIVEN since the previous report, cm    */
} BrainIn;

/*============================================================================
 * OUTPUT
 *============================================================================*/

/** Returned by brain_step() when exploration is provably complete.  Query
 *  brain_home_path() and brain_fast_path() for the plans. */
#define BRAIN_DONE  0

/**
 * @brief  Introspection for the operator display — never required for control.
 */
typedef struct {
    uint8_t  move;       /**< 'F'/'L'/'R'/'B', or BRAIN_DONE                 */
    uint8_t  phase;      /**< MazePhase: 0 EXPLORE 1 RETURN_HOME 2 FAST_RUN  */
    uint16_t node;       /**< The brain's node id for the current junction   */
    int16_t  x, y;       /**< Dead-reckoned position, cm (brain's own frame) */
    uint16_t reports;    /**< Number of reports accepted so far              */
    uint16_t cells;      /**< Cells implied by the last dist_cm              */
    uint16_t drift_cm;   /**< |dist_cm − cells×MAZE_CELL_CM|, the last report.
                          *   A growing number means the encoder calibration is
                          *   off, or the robot is not stopping where it should.
                          *   The brain snaps to the lattice regardless, so this
                          *   is a warning, not a fault. */
    uint8_t  target_found; /**< The goal node has been physically reached    */
    uint8_t  finished;     /**< brain_step() has returned BRAIN_DONE         */
    float    fast_time_s;  /**< Time the fast path takes under the accel
                            *   model (s).  Valid once `finished`.  This is the
                            *   quantity the fast run actually minimises — the
                            *   route with the fewest moves is often NOT the
                            *   fastest one.  float because the codebase already
                            *   does its time math in float (maze_fastrun.c). */
} BrainStatus;

/*============================================================================
 * API
 *============================================================================*/

/**
 * @brief  Reset the brain and place the robot at the origin of its own frame.
 *
 * Call once at boot, before the robot leaves the start node.  The initial
 * physical facing is taken to be the frame's "north" — the caller never has to
 * line the robot up with the real compass, and the absolute initial heading
 * never needs to be supplied; the brain's frame is self-consistent from here.
 */
void brain_init(void);

/**
 * @brief  Report one junction and get the next move.
 *
 * @param  in  What the robot sees.  Must not be NULL.
 * @return 'F', 'L', 'R', 'B' to execute, or BRAIN_DONE when exploration is
 *         complete — after which the brain emits no further moves, and the
 *         caller should read the two paths.
 *
 * The returned move is RELATIVE to the robot's current facing: 'F' means carry
 * straight on, 'L' means turn 90° left in place then carry on, 'B' means turn
 * around.
 */
uint8_t brain_step(const BrainIn* in);

/**
 * @brief  The path from wherever the robot is now back to the start node.
 *
 * Valid only after brain_step() has returned BRAIN_DONE.  Empty string if the
 * robot finished exploration already standing on the start.
 *
 * The commands assume the robot's CURRENT heading — i.e. the facing it had when
 * it stopped at the last junction — so they can be executed straight away with
 * no reorientation.
 */
const char* brain_home_path(void);

/**
 * @brief  The time-optimal path from the start node to the target.
 *
 * Valid only after brain_step() has returned BRAIN_DONE.  Computed on the
 * DISCOVERED map only — never the full maze — so a proof of optimality made
 * from partial knowledge stays honest.
 *
 * The commands assume the heading the robot will have once it has finished
 * executing brain_home_path(), so home-then-fast concatenates into one valid
 * command stream.
 */
const char* brain_fast_path(void);

/**
 * @brief  Fill in the operator display.  Safe to call at any time.
 */
void brain_status(BrainStatus* out);

#ifdef __cplusplus
}
#endif

#endif /* BRAIN_H */
