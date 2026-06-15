/**
 * @file maze_explore.h
 * @brief Target-agnostic frontier exploration (ALGORITHMS.md §1).
 *
 * The exploration step rule (Priority 1→2→3):
 *
 *   P1 — If the current node has unexplored branches, drive down the best
 *        one (turn-minimising: straight > left > right > reverse, with a
 *        fixed compass tie-break).
 *
 *   P2 — Drive to the nearest USEFUL frontier over the known map.  A
 *        frontier is "useful" if the early-stop proof has not pruned it
 *        (see maze_proof.h).  Before the target is found, ALL frontiers
 *        are useful.
 *
 *   P3 — No useful frontiers remain → exploration is complete.  Return
 *        home to the start node.
 *
 * This module is TARGET-AGNOSTIC — it must never use the target's position
 * before `target_found` is true (Architecture rule #1).
 */

#ifndef MAZE_EXPLORE_H
#define MAZE_EXPLORE_H

#include "maze_types.h"
#include "maze_robot.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Execute ONE exploration step.
 *
 * Decides the next move according to the P1→P2→P3 priority system
 * and updates the robot state accordingly.
 *
 * @param  robot        Robot state (position, heading, graph, visited sets).
 * @param  use_proof    If true, filter frontiers through the time-based proof.
 *                      If false, all frontiers are "useful" (full-map mode).
 * @param  v_max_fp     Fixed-point max speed (cm/s × 100) for proof pruning.
 * @param  a_max_fp     Fixed-point max accel (cm/s² × 100) — passed through
 *                      to proof module.
 *
 * @return The command executed (F/L/R/B), or MAZE_CMD_NONE if exploration
 *         is complete (no more useful frontiers → caller should switch to
 *         RETURN_HOME or declare DONE).
 */
MazeCommand maze_explore_step(MazeRobot* robot,
                              bool use_proof,
                              int32_t v_max_fp,
                              int32_t a_max_fp);

/**
 * @brief  Check whether exploration should end (no useful frontiers remain).
 */
bool maze_explore_is_complete(const MazeRobot* robot);

/**
 * @brief  Get the best unexplored branch at the current node (P1).
 * @param  robot   Robot state.
 * @param  chosen  Output: best neighbor node index.
 * @return true if a branch was found, false if none.
 */
bool maze_explore_best_branch(const MazeRobot* robot, uint16_t* chosen);

#ifdef __cplusplus
}
#endif

#endif /* MAZE_EXPLORE_H */
