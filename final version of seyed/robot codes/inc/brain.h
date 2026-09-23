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
 * HOW TO FEED IT FROM THE FIRMWARE
 *
 * Read from `firmware/Core/Src/main.c`.  Line numbers are for the copy in this
 * repo; SENSORS.md's citation of "main.c:1202-1214" is stale — that region is
 * now commented-out I2C scan code, and the junction decision lives at 1366-1407
 * (front bank) / 1442-1490 (rear bank).
 *
 * THE MOVE MAPPING IS 1:1 — nothing between the decision and the motors changes.
 * The firmware already has exactly these four actions, selected by `cross`:
 *
 *     brain    firmware `cross`   action            dispatch at
 *     'F'      0                  Forward()/_r()    1362
 *     'L'      1                  turn_left()/_r()  1367
 *     'R'      2                  turn_right()/_r() 1372
 *     'B'      4                  head flip, nav+=2 1389
 *
 * So the integration is: replace the *choice* of `cross`, keep everything after
 * it.  `turn_left()`/`turn_right()` also clear `left_poss`/`right_poss`
 * themselves (main.c:602-603, 640-641), which is why the exits must be read
 * BEFORE the motion primitive — the same rule SENSORS.md already records.
 *
 * CALL POINT: `path_append()` (main.c:939) is the single choke point that all
 * eight junction decisions pass through, and it is where the link length is
 * recorded.  The brain must be invoked just BEFORE the decision, because it
 * needs `dist_cm` to place the node.
 *
 * BUILDING BrainIn FROM THE GLOBALS
 *
 *     in.left    = left_poss;      -- main.c:1366  s[2] && (s[3]||s[4]||s[5]||s[6])
 *     in.right   = right_poss;     -- main.c:1367  s[7] && (s[3]||s[4]||s[5]||s[6])
 *     in.front   = (s[3]||s[4]||s[5]||s[6]);   -- centre on line, main.c:1380
 *     in.back    = 1;
 *     in.target  = OnEndZoon;      -- main.c:1320, the all-black row test
 *     in.dist_cm = <the link just closed, in cm>;
 *
 * Rear bank (`head == 1`): the same, with `s[11]` / `s[12..15]` / `s[16]`.
 *
 * `in.dist_cm` is the firmware's own link length — the encoder delta since the
 * `on_link==0 → ResetEncoder` at the top of the link (main.c:1359-1364), run
 * through the same /(2.467*2) and the same per-command correction that
 * `path_append()` applies (main.c:951-972: +85/+95 after a turn, +25/+30 after a
 * straight, +104 after a 'B').  Factor that arithmetic into one helper called
 * from both places rather than duplicating it: the corrections are empirical and
 * must not drift apart.  The brain needs it BEFORE the decision, `path_append()`
 * records it AFTER — same number, computed once.
 *
 * THE JUNCTION GATE IS A POSITION GATE, AND IT CAN BE INHERITED
 *
 * `s[0] && s[9]` (main.c:1372, 1378) reads like a "black on both sides" test, and
 * an earlier revision of this file wrongly treated it as one.  It is not:
 * `s[0]` and `s[9]` sit in the middle of the robot ON THE AXIS OF ROTATION
 * (measured; see SENSORS.md §2).  So the gate means "my rotation axis is over
 * the node" — the firmware's arrival test, not a test of which branches exist.
 *
 * Read that way the block is a clean left-hand rule and the gate is exactly what
 * the brain wants: `left_poss`/`right_poss` say what the branches are, the gate
 * says when to act.  A junction is therefore declared on
 *
 *     (s[0] && s[9])  &&  (left_poss || right_poss || dead-end)
 *
 * and a node with no lateral is a straight passthrough driven through with no
 * report — which is what the brain already assumes, and reconstructs from
 * `dist_cm`.  Gate and brain agree; inherit it.
 *
 * `front` — DECIDED: CENTRE ON LINE
 *
 * `in.front` is the one field the firmware never computes as such — it is
 * consulted only in the single branch where `right_poss==1 && left_poss==0`, to
 * choose straight over right (main.c:1380).  The brain needs it always, as a map
 * fact: a phantom `front` invents a corridor that does not exist, and the route
 * the brain later calls fastest can then run through a wall.
 *
 * The rule chosen (user, 2026-09-23) is the firmware's own proxy:
 *
 *     in.front = (s[3] || s[4] || s[5] || s[6]);
 *
 * Unambiguous at a crossing and at a T.  The residual doubt is the CORNER: if
 * the robot's own incoming arm is still under the centre group when it stops,
 * this reads "forward open" where there is no forward — and it matters more than
 * it looks, because `Forward()` is not "drive straight", it is the LINE
 * FOLLOWER, so a robot told 'F' there steers round the corner while the brain
 * believes it went straight and the dead reckoning diverges.
 *
 * That doubt is to be settled by MEASUREMENT, not by more geometry — see below.
 * If a corner ever does lie, the fix is local to this one expression.
 *
 * WHAT MEASURES IT: the M1 telemetry build logs every junction the robot
 * actually stops at, with the raw sensor masks AND the flags:
 *
 *     J,<ms>,<ch>,<nav>,<head>,<raw>,<cm>,<front>,<rear>,<L>,<R>,<cross>
 *
 * `<front>`/`<rear>` are the sensor masks as hex, so the bar's black pattern at
 * every real junction is recoverable, and `<L>`,`<R>`,`<cross>` say what the
 * legacy logic concluded from it.  One capture at a known corner is the check.
 *
 * Gate, for the record: `s[0] && s[9]` is inherited AS WRITTEN (user,
 * 2026-09-23) — anded, per main.c:1372/1378.
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
