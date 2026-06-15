/**
 * @file maze_proof.h
 * @brief Time-based early-stop branch & bound (ALGORITHMS.md §2).
 *
 * After the target is found, the robot can stop exploring early if it can
 * PROVE that no undiscovered path could be faster than the best known
 * time-optimal route.  The proof uses a TIME lower bound (never distance),
 * because the fast run uses an acceleration model where a longer straighter
 * route can beat a shorter twisty one.
 *
 * The bound (ALGORITHMS.md §2.2):
 *
 *   LB_time(f) = ( dist_known(start→f) + straight_line(f→target) ) / v_max
 *
 * A frontier f is USEFUL iff  LB_time(f) < best_known_TIME.
 *
 * - If NO useful frontier remains, the best known route is provably optimal.
 * - The robot can then stop exploring and go home.
 *
 * This module is ONLY called after target_found == true.
 */

#ifndef MAZE_PROOF_H
#define MAZE_PROOF_H

#include "maze_types.h"
#include "maze_robot.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Compute the time lower bound for reaching the target through
 *         frontier `f` (in seconds × 100, i.e. centiseconds).
 *
 *         LB_time = (dist_known(start→f) + straight_line(f→target)) / v_max
 *
 * @param  robot     Robot state (must have target_found == true).
 * @param  frontier  Node index of a frontier.
 * @param  v_max_fp  Fixed-point max speed (cm/s × 100).
 * @return Lower-bound time (centiseconds), or UINT32_MAX on error.
 */
uint32_t maze_proof_frontier_lb(const MazeRobot* robot,
                                uint16_t frontier,
                                int32_t v_max_fp);

/**
 * @brief  Compute the best known TIME from start to target on the KNOWN MAP
 *         using the time-optimal path algorithm (acceleration-aware).
 *
 * @param  robot     Robot state.
 * @param  v_max_fp  Fixed-point max speed (cm/s × 100).
 * @param  a_max_fp  Fixed-point max acceleration (cm/s² × 100).
 * @return Best known time (centiseconds), or UINT32_MAX if no path to target.
 */
uint32_t maze_proof_best_time(const MazeRobot* robot,
                              int32_t v_max_fp,
                              int32_t a_max_fp);

/**
 * @brief  Check whether frontier `f` is "useful" — i.e., could it lead to
 *         a FASTER route than the best known one?
 *
 *         useful(f) ⟺ LB_time(f) < best_known_TIME
 *
 * @return true if the frontier is worth visiting, false if it can be pruned.
 */
bool maze_proof_is_useful(const MazeRobot* robot,
                          uint16_t frontier,
                          int32_t v_max_fp,
                          int32_t a_max_fp);

/**
 * @brief  Find all useful frontiers in the frontier array.
 *
 * @param  robot          Robot state.
 * @param  frontiers      All frontier node indices.
 * @param  fcount         Total frontier count.
 * @param  useful         Output: only useful frontiers.
 * @param  max_useful     Capacity of `useful`.
 * @param  v_max_fp       Fixed-point max speed.
 * @param  a_max_fp       Fixed-point max acceleration.
 * @return Number of useful frontiers written.
 */
uint8_t maze_proof_filter_frontiers(const MazeRobot* robot,
                                    const uint16_t* frontiers,
                                    uint8_t fcount,
                                    uint16_t* useful,
                                    uint8_t max_useful,
                                    int32_t v_max_fp,
                                    int32_t a_max_fp);

/**
 * @brief  Determine the proof state after filtering.
 *
 * @return MAZE_PROOF_PROVEN  — stopped early (some frontiers pruned).
 *         MAZE_PROOF_FULL    — all frontiers were useful (= full map).
 */
MazeProofState maze_proof_determine_state(const MazeRobot* robot,
                                          uint8_t total_frontiers,
                                          uint8_t useful_count);

#ifdef __cplusplus
}
#endif

#endif /* MAZE_PROOF_H */
